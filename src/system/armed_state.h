/**
 * @file    armed_state.h
 * @brief   State machine arming + preflight gate + satu-satunya sumber
 *          kebenaran status armed di firmware. Tanggung jawab: Orang 1
 *          (Sistem & Komunikasi). Lihat `armed_state.md` untuk desain
 *          lengkap (state diagram, tabel command gated, alasan tiap
 *          keputusan) -- header ini implementasi konkretnya.
 *
 * === Kenapa modul ini ada ===
 * Sebelum modul ini, logika arming hidup inline di `main.c`
 * (`task_control()`, variabel `s_fc.armed`) dan `command_handler.c`
 * memakai `Armed_IsArmed()` __attribute__((weak)) yang selalu return 1
 * (fail-safe: dianggap armed) karena belum ada modul flight-state
 * sungguhan yang meng-override-nya (lihat catatan di command_handler.h).
 * File ini MENYEDIAKAN definisi kuat `Armed_IsArmed()` tersebut, supaya
 * command gated (CMD_MOTOR_TEST, CMD_SERVO_TEST, CMD_REBOOT_DFU -- daftar
 * di command_handler.c) benar-benar merefleksikan status armed asli,
 * bukan selalu ditolak.
 *
 * TODO INTEGRASI (belum dilakukan di commit ini -- lihat armed_state.md
 * Bagian 7 & catatan di bawah): `task_control()` di main.c saat ini masih
 * menghitung `want_armed` sendiri secara inline dan menulis langsung ke
 * `s_fc.armed`. Supaya `Armed_IsArmed()` di modul ini benar-benar
 * mencerminkan kondisi pesawat, main.c PERLU dimigrasi untuk memanggil
 * ArmedState_Update() dari task_control() dan membaca ArmedState_IsArmed()
 * (bukan field `s_fc.armed` lokal) -- ini pekerjaan terpisah, TIDAK
 * termasuk cakupan armed_state.c/h ini sendiri.
 *
 * === Perbedaan sengaja vs logika lama di main.c ===
 * Kode lama di `task_control()` menghitung ulang SELURUH syarat
 * (termasuk `link_ok` dan `attitude_ok`) setiap cycle dan langsung
 * menulis `s_fc.armed = want_armed` -- efeknya: begitu link RX (dulu CRSF,
 * sekarang iBus) putus
 * saat sedang ARMED, pesawat otomatis disarm SEKETIKA itu juga. Ini
 * bertentangan dengan alur RTH failsafe (Orang 3): RTH butuh motor tetap
 * berputar untuk terbang pulang, jadi disarm otomatis saat link hilang
 * membuat RTH tidak mungkin jalan. Modul ini SENGAJA memisahkan dua hal:
 *   - Syarat gerbang preflight (Bagian di bawah) HANYA dicek saat
 *     transisi DISARMED -> ARMED (mulai arming).
 *   - Selama sudah ARMED, state HANYA berubah lewat arm-switch
 *     dimatikan pilot atau panggilan eksplisit ArmedState_ForceDisarm().
 * Kebijakan "link-loss saat ARMED harus force-disarm atau biarkan RTH
 * jalan dulu" TETAP keputusan terbuka tim (armed_state.md Bagian 7,
 * protocol.md-adjacent) -- Orang 3 yang memutuskan kapan memanggil
 * ArmedState_ForceDisarm() dari alur failsafe-nya sendiri
 * (RxIbus_Update() / Nav_TriggerRTH()), modul ini tidak menebak.
 */

#ifndef ARMED_STATE_H
#define ARMED_STATE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------
 * State machine (armed_state.md Bagian 2)
 * ------------------------------------------------------------------- */

typedef enum {
    ARMED_STATE_DISARMED = 0,   /* default setelah boot/watchdog reset */
    ARMED_STATE_ARMING_CHECK,   /* transien: preflight sedang dievaluasi.
                                  * Di v1 ini sinkron (selesai dalam satu
                                  * panggilan ArmedState_Update(), lihat
                                  * armed_state.md Bagian 3) -- state ini
                                  * disediakan di enum untuk kejelasan API
                                  * dan kalau nanti ada check async
                                  * (mis. tunggu GPS fix), TAPI belum
                                  * pernah persist lintas panggilan di
                                  * implementasi sekarang. */
    ARMED_STATE_ARMED,
} ArmedState_t;

/**
 * @brief Alasan preflight gagal, dicek dalam urutan tetap di
 *        run_preflight() (armed_state.c) -- yang PERTAMA gagal yang
 *        dilaporkan, bukan daftar lengkap semua yang gagal (cukup untuk
 *        satu baris pesan ke pilot/log; kalau butuh semua alasan
 *        sekaligus, panggil ulang setelah masing-masing diperbaiki).
 */
typedef enum {
    PREFLIGHT_OK = 0,
    PREFLIGHT_THROTTLE_NOT_LOW,  /* stick throttle harus rendah untuk
                                   * MULAI arm (tidak berlaku untuk tetap
                                   * armed) -- perilaku yang sudah ada di
                                   * main.c lama, dipertahankan di sini */
    PREFLIGHT_NO_RX_LINK,        /* link RX (iBus) tidak OK / belum pernah
                                   * terima channel data valid */
    PREFLIGHT_IMU_FAULT,         /* AHRS_Fusion_IsAttitudeValid() == false
                                   * (kedua IMU timeout/tidak reliable) */
    PREFLIGHT_IMU_DISAGREE,      /* attitude valid tapi sedang dalam
                                   * status disagreement dual-IMU.
                                   * CATATAN (keputusan tim, bukan bug):
                                   * sejak task_imu_secondary() di main.c
                                   * jadi failover-murni (MPU6050 idle
                                   * selama primer sehat, demi kurangi
                                   * beban I2C1), gate ini EFEKTIF
                                   * NONAKTIF selama MPU6500 sehat --
                                   * disagreement cuma bisa terdeteksi
                                   * kalau kedua IMU sama-sama kebaca di
                                   * cycle yang sama, dan itu sekarang
                                   * cuma terjadi pas primer sudah
                                   * timeout (di mana attitude_valid
                                   * sendiri sudah jadi soal terpisah).
                                   * MPU6500 yang hidup tapi diam-diam
                                   * salah baca TIDAK akan ketahuan lewat
                                   * gate ini lagi. */
    PREFLIGHT_UNCALIBRATED,      /* accel/gyro atau mag belum pernah
                                   * dikalibrasi sejak boot -- lihat
                                   * catatan `calibration_done` di
                                   * ArmedStateInputs_t soal keterbatasan
                                   * saat ini */
    PREFLIGHT_LOW_BATTERY,       /* RESERVED, TIDAK DIPAKAI LAGI -- baterai
                                   * sudah dihapus dari gerbang arming atas
                                   * keputusan pemilik proyek (baterai tetap
                                   * dipakai untuk telemetri/tampilan, lihat
                                   * battery.c). Nilai enum ini SENGAJA
                                   * dipertahankan (bukan dihapus) supaya
                                   * nilai numerik PREFLIGHT_BLACKBOX_FAULT
                                   * di bawah tidak bergeser -- kode di luar
                                   * repo ini (web configurator) mungkin
                                   * membaca nilai numeriknya. Jangan
                                   * dipakai lagi di run_preflight(). */
    PREFLIGHT_BLACKBOX_FAULT,    /* sesi logging blackbox gagal
                                   * init/tulis -- caller yang menentukan
                                   * nilai ini, lihat catatan input */
} ArmedPreflightResult_t;

/* ---------------------------------------------------------------------
 * Input snapshot per-cycle (armed_state.md Bagian 3)
 * ------------------------------------------------------------------- */

/**
 * @brief Snapshot sinyal dari modul lain yang dibutuhkan preflight
 *        check + evaluasi arm/disarm. Diisi caller (task_control() di
 *        main.c setelah migrasi -- lihat TODO INTEGRASI di atas) SETIAP
 *        cycle sebelum memanggil ArmedState_Update().
 *
 * Modul ini SENGAJA tidak #include header Orang 2/3/4 langsung
 * (rx_ibus.h, ahrs_fusion.h, battery.h, blackbox.h, calib_dispatcher.h)
 * -- caller yang bertanggung jawab menerjemahkan state modul-modul itu
 * jadi boolean sederhana di sini. Ini menjaga armed_state.c tetap
 * unit-testable tanpa perlu link seluruh firmware, konsisten dengan pola
 * yang sama dipakai RxIbus_Update() (menerima function pointer, bukan
 * include navigation.h) dan OutputMap_WriteFromMixer() (menerima `bool
 * armed` mentah, bukan include armed_state.h).
 */
typedef struct {
    /** arm_switch dari channel AUX RX (dulu CRSF, sekarang iBus --
     *  main.c lama: channel[4] > IBUS_CH_CENTER) -- level, bukan edge. */
    bool arm_switch_engaged;

    /** Stick throttle sudah cukup rendah untuk MULAI arm (main.c lama:
     *  throttle_stick < 0.05f). Hanya relevan saat transisi
     *  DISARMED -> ARMED; diabaikan selama sudah ARMED. */
    bool throttle_low;

    /** RxIbus_GetLinkStatus(&rx) == IBUS_LINK_OK &&
     *  RxIbus_GetChannels(&rx)->data_valid */
    bool rx_link_ok;

    /** AHRS_Fusion_IsAttitudeValid(&ahrs) */
    bool attitude_valid;

    /** ahrs.attitude.disagreement_flag (hanya bermakna kalau
     *  attitude_valid == true). Sejak MPU6050 failover-murni (lihat
     *  PREFLIGHT_IMU_DISAGREE di atas), nilai ini EFEKTIF SELALU false
     *  selama primer sehat -- caller (main.c) tetap wajib mengisi field
     *  ini apa adanya dari ahrs.attitude.disagreement_flag, JANGAN
     *  hardcode false di sini walau dampaknya sama; kalau nanti
     *  strategi baca IMU berubah lagi, field ini otomatis benar lagi
     *  tanpa perlu diingat untuk di-revert. */
    bool imu_disagreement;

    /** Placeholder v1: belum ada mekanisme persist "sudah pernah
     *  kalibrasi sejak boot" di calib_dispatcher.c/h (dispatcher hanya
     *  melacak kalibrasi yang SEDANG/baru saja berjalan, bukan riwayat
     *  permanen -- lihat calib_dispatcher.h). Sampai itu ada (idealnya
     *  flag tersimpan di SETTING_COMMIT, bukan RAM saja), caller boleh
     *  hardcode `true` di sini SAMA seperti pola "belum blocker
     *  sekarang" yang dipakai untuk item terbuka lain (pinout-fc-stm32f411.md
     *  Bagian 4, kristal LSE). JANGAN diam-diam skip check ini di kode
     *  pemanggil tanpa komentar yang sama jujurnya. */
    bool calibration_done;

    /** Diisi caller dari status terakhir Blackbox_Init() /
     *  Blackbox_WriteRecord() (blackbox.h tidak menyediakan getter
     *  status tunggal -- caller yang melacak return code terakhir).
     *  Kalau blackbox belum diaktifkan sama sekali di scheduler (lihat
     *  catatan di main.c: "Blackbox belum diaktifkan di scheduler ini"),
     *  set true di sini supaya tidak memblokir arming karena fitur yang
     *  memang belum jalan -- perbarui begitu blackbox betulan aktif. */
    bool blackbox_ready;
} ArmedStateInputs_t;

/* ---------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------- */

/** @brief Reset ke ARMED_STATE_DISARMED. WAJIB dipanggil sekali saat
 *         boot (termasuk boot akibat watchdog reset -- lihat
 *         armed_state.md Bagian 5: tidak ada state persist lintas
 *         reset, ini otomatis benar selama fungsi ini dipanggil dari
 *         urutan init normal, bukan butuh deteksi reset-cause khusus). */
void ArmedState_Init(void);

/**
 * @brief Evaluasi satu cycle state machine. Panggil dari task scheduler
 *        yang sama tempat main.c lama menghitung `want_armed`
 *        (task_control(), lihat TODO INTEGRASI di atas), SETELAH modul
 *        AHRS/RX/battery/dsb selesai update di cycle yang sama.
 *
 * Perilaku (armed_state.md Bagian 2 & 3):
 *   - DISARMED, arm_switch_engaged == false: tetap DISARMED, tidak
 *     menyentuh last-preflight-error (idle, bukan kegagalan).
 *   - DISARMED, arm_switch_engaged == true: jalankan preflight check
 *     SINKRON dalam panggilan ini. Lolos semua -> pindah ARMED.
 *     Gagal salah satu -> tetap DISARMED, ArmedState_GetLastPreflightError()
 *     diisi alasan pertama yang gagal.
 *   - ARMED, arm_switch_engaged == false: pindah DISARMED (disarm
 *     normal lewat switch pilot).
 *   - ARMED, arm_switch_engaged == true: TETAP ARMED tanpa
 *     mengevaluasi ulang preflight (lihat catatan "Perbedaan sengaja"
 *     di atas -- link-loss/attitude-invalid saat sudah ARMED TIDAK
 *     memaksa disarm dari sini).
 *
 * @param inputs snapshot sinyal cycle ini, lihat ArmedStateInputs_t.
 *               Tidak boleh NULL.
 */
void ArmedState_Update(const ArmedStateInputs_t *inputs);

/** @brief Paksa disarm dari luar, terlepas dari posisi arm-switch saat
 *         ini. Dipakai alur failsafe Orang 3 (mis. setelah RTH selesai
 *         mendarat, atau kalau tim akhirnya memutuskan link-loss harus
 *         langsung disarm -- lihat catatan "Perbedaan sengaja" di
 *         atas) dan bisa juga dipakai command darurat dari web kalau
 *         nanti ada (belum ada CMD_DISARM eksplisit di protocol.md
 *         v1 -- item terbuka baru, belum tercatat di protocol.md
 *         Bagian 12, tandai kalau tim mau menambahkannya). */
void ArmedState_ForceDisarm(void);

/** @brief State mentah saat ini (lihat ArmedState_t). Untuk kebanyakan
 *         pemanggil, ArmedState_IsArmed() lebih langsung dipakai. */
ArmedState_t ArmedState_Get(void);

/** @brief true kalau state == ARMED_STATE_ARMED. Dipakai internal untuk
 *         mengisi Armed_IsArmed() (lihat di bawah) dan boleh dipanggil
 *         langsung modul lain yang butuh bool sederhana (mis.
 *         OutputMap_WriteFromMixer(), Nav_Update()). */
bool ArmedState_IsArmed(void);

/** @brief Alasan preflight terakhir yang dievaluasi modul ini (bukan
 *         history semua kegagalan, hanya evaluasi PALING TERAKHIR).
 *         PREFLIGHT_OK baik berarti "lolos" MAUPUN "belum pernah
 *         mencoba arm sejak boot" -- bedakan lewat ArmedState_Get()
 *         kalau perlu. */
ArmedPreflightResult_t ArmedState_GetLastPreflightError(void);

/**
 * @brief Implementasi KUAT (override __attribute__((weak)) default di
 *        command_handler.c) untuk hook yang sudah dideklarasikan di
 *        command_handler.h. WAJIB linker link armed_state.c supaya
 *        symbol ini menang atas versi weak -- kalau lupa, firmware
 *        tetap compile+link tapi command gated (MOTOR_TEST/SERVO_TEST/
 *        REBOOT_DFU) akan SELALU ditolak (fail-safe weak default,
 *        lihat command_handler.c), bukan crash. Signature (int, bukan
 *        bool) sengaja disamakan persis dengan deklarasi di
 *        command_handler.h, JANGAN diubah ke bool di sini.
 */
int Armed_IsArmed(void);

#ifdef __cplusplus
}
#endif

#endif /* ARMED_STATE_H */
