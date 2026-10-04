/**
 * calib_dispatcher.h
 *
 * Orkestrator status kalibrasi — satu titik yang dipanggil
 * command_handler.c (Orang 1) untuk start/stop/poll kalibrasi apa pun,
 * supaya command_handler tidak perlu tahu detail internal
 * calib_accel_gyro.c / calib_mag.c satu-satu.
 *
 * Tanggung jawab dispatcher:
 *   1. Pastikan hanya SATU jenis kalibrasi aktif dalam satu waktu
 *      (mulai kalibrasi kedua saat yang pertama masih IN_PROGRESS
 *      ditolak, bukan menimpa diam-diam).
 *   2. Rute sample sensor mentah ke modul kalibrasi yang sedang aktif
 *      saja (FeedImuSample / FeedMagSample no-op kalau jenis itu
 *      sedang tidak aktif — aman dipanggil terus tiap cycle scheduler
 *      tanpa cek kondisi di caller).
 *   3. Bungkus GetStatus() jadi bentuk yang persis sama dengan payload
 *      wire CMD_CALIB_STATUS (protocol.md Bagian 10), supaya
 *      command_handler tinggal serialize tanpa transformasi tambahan.
 *
 * CATATAN: nilai `calib_type` di wire protocol belum dienumerasi
 * eksplisit di protocol.md — dispatcher ini mengasumsikan urutan
 * sesuai pengelompokan command ID (0x0401=accel_gyro -> type 0,
 * 0x0403=mag -> type 1). Konfirmasi ke Orang 1 / tim protokol kalau
 * ternyata nilai yang disepakati beda.
 */

#ifndef CALIB_DISPATCHER_H
#define CALIB_DISPATCHER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CALIB_TYPE_ACCEL_GYRO = 0,
    CALIB_TYPE_MAG        = 1
} CalibType_t;

/**
 * Mulai kalibrasi jenis `type`. Return false (ditolak) kalau ada
 * kalibrasi jenis LAIN yang sedang IN_PROGRESS — caller
 * (command_handler) sebaiknya balas CMD_ERROR ke web kalau ini
 * terjadi. Memulai ulang jenis yang sama yang sedang berjalan
 * diperbolehkan (reset progress dari awal).
 */
bool CalibDispatcher_Start(CalibType_t type);

/** Hentikan kalibrasi jenis `type` kalau sedang berjalan. No-op kalau tidak. */
void CalibDispatcher_Stop(CalibType_t type);

/**
 * Umpankan sample IMU primer (accel+gyro). Dipanggil scheduler tiap
 * cycle baca IMU, tidak perlu cek kalibrasi jenis apa yang aktif —
 * dispatcher yang menentukan apakah sample ini relevan.
 */
void CalibDispatcher_FeedImuSample(int16_t ax, int16_t ay, int16_t az,
                                    int16_t gx, int16_t gy, int16_t gz);

/** Umpankan sample magnetometer. Sama seperti FeedImuSample. */
void CalibDispatcher_FeedMagSample(int16_t x, int16_t y, int16_t z);

/**
 * Isi *out_type/out_state/out_progress sesuai kalibrasi yang PALING
 * TERAKHIR di-start (atau sedang berjalan). Return false kalau belum
 * pernah ada kalibrasi apa pun yang di-start sejak boot (semua field
 * output tidak diisi) — command_handler sebaiknya balas state IDLE
 * dengan calib_type default (mis. 0) kalau ini terjadi, bukan error,
 * karena "belum pernah kalibrasi" adalah kondisi normal di boot awal.
 */
bool CalibDispatcher_GetStatus(uint8_t *out_type, uint8_t *out_state,
                                uint8_t *out_progress);

/**
 * @brief Daftarkan handler CMD_CALIB_ACCEL_GYRO_START/STOP (0x0401/0402),
 *        CMD_CALIB_MAG_START/STOP (0x0403/0404) dan CMD_CALIB_STATUS
 *        (0x0405) ke command_handler.c (Comms #3). Pola sama dengan
 *        Nav_RegisterProtocolHandlers()/OutputMap_RegisterCommands().
 *        WAJIB dipanggil SETELAH CommandHandler_Init() (tabel registrasi
 *        di-reset di sana).
 *
 *        Ketiga kelompok command ini TIDAK armed-gated (sesuai
 *        protocol.md) -- lihat catatan temuan di calib_dispatcher.c.
 *        Kalibrasi lain/pemulihan otomatis sedang berjalan dibalas
 *        CMD_ERROR/ERROR_BUSY.
 */
void CalibDispatcher_RegisterCommands(void);

/**
 * @brief Dependensi eksternal (DIDEFINISIKAN main.c, bukan modul ini):
 *        true kalau task_calib_recover() sedang memakai/akan segera
 *        memakai singleton calib_accel_gyro.c untuk pemulihan otomatis.
 *        Dipakai HANYA oleh guard CMD_CALIB_ACCEL_GYRO_START.
 *        Sengaja bukan weak: kalau main.c lupa mendefinisikannya, link
 *        gagal (keras), bukan guard yang diam-diam hilang.
 */
bool CalibRecover_IsActive(void);

#ifdef __cplusplus
}
#endif

#endif /* CALIB_DISPATCHER_H */
