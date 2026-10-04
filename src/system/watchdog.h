/**
 * @file    watchdog.h
 * @brief   Pembungkus Independent Watchdog (IWDG) + kick yang sadar
 *          kesehatan scheduler, bukan sekadar kick buta tiap loop.
 *          Tanggung jawab: Orang 1 (Sistem & Komunikasi).
 *
 * === Kenapa modul ini ada ===
 * Implementasi lama (`main.c`, `watchdog_init()`/`watchdog_feed()`) memoles
 * register IWDG langsung dengan angka ajaib (PR=3, RLR=500) dan
 * `watchdog_feed()` dipanggil TANPA SYARAT setiap putaran `while(1)`,
 * persis setelah `Scheduler_Tick()`:
 *
 *   Scheduler_Tick(&s_sched);
 *   watchdog_feed();
 *
 * Pola ini menangkap SATU kelas kegagalan (loop benar-benar macet total,
 * mis. terjebak busy-wait I2C tanpa timeout) karena kalau itu terjadi,
 * `Scheduler_Tick()` sendiri tidak pernah return dan baris `watchdog_feed()`
 * tidak tercapai -> IWDG reset. TAPI pola ini TIDAK menangkap kelas
 * kegagalan kedua yang sama berbahayanya: satu task kritis (mis.
 * `task_control`, `task_failsafe`) diam-diam berhenti dijadwalkan atau
 * gagal terus tanpa pernah nge-hang (mis. bug di `next_due_us`, atau
 * kondisi guard yang salah bikin task selalu early-return) — loop utama
 * tetap jalan mulus, task LAIN tetap jalan, `watchdog_feed()` tetap
 * dipanggil tiap loop, tapi kontrol pesawat sudah berhenti update.
 *
 * `scheduler.h` sudah mencatat statistik yang cukup untuk mendeteksi ini
 * (`last_run_us`, `run_count` per task) tapi statistik itu tidak pernah
 * dipakai untuk apa pun oleh kode lama selain dilihat manual sebelum uji
 * terbang. Modul ini menutup celah itu: task kritis didaftarkan lewat
 * `Watchdog_RegisterCriticalTask()`, dan `Watchdog_FeedIfHealthy()`
 * (dipanggil pengganti `watchdog_feed()` mentah) HANYA benar-benar kick
 * IWDG kalau semua task terdaftar terbukti masih berjalan dalam batas
 * waktu yang diharapkan. Kalau tidak, IWDG dibiarkan expire secara
 * sengaja -> MCU reset -> boot ulang selalu `ARMED_STATE_DISARMED`
 * (lihat armed_state.h) -- jauh lebih aman daripada terbang dengan task
 * kontrol yang diam-diam berhenti.
 *
 * === Yang TIDAK berubah dari desain lama ===
 * - IWDG tetap clocked dari LSI internal (~32kHz NOMINAL, tidak
 *   dikalibrasi pabrik -- toleransi bisa dobel/separuh dari nominal
 *   menurut datasheet STM32F411, sama caveat-nya dengan komentar asli
 *   di `main.c`). Nilai timeout dari modul ini adalah PERKIRAAN, bukan
 *   presisi tinggi -- jangan andalkan untuk sesuatu yang butuh timing
 *   ketat.
 * - IWDG TETAP tidak bisa dimatikan setelah `Watchdog_Init()` dipanggil,
 *   kecuali lewat reset. Saat debug step-by-step, panggil
 *   `Watchdog_FreezeDuringDebugHalt()` SEBELUM breakpoint pertama supaya
 *   IWDG berhenti menghitung selama CPU halt (lihat catatan lama di
 *   `main.c`) -- kalau lupa, MCU akan reset tiap kali berhenti di
 *   breakpoint lebih lama dari timeout.
 */

#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdint.h>
#include <stdbool.h>
#include "scheduler.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Timeout default, sama dengan nilai yang dipakai desain lama (~500ms,
 *  lihat catatan akurasi LSI di atas). */
#define WATCHDOG_DEFAULT_TIMEOUT_MS   500U

/** Berapa banyak task kritis yang bisa dipantau health-check-nya. Naikkan
 *  kalau tim mendaftarkan lebih dari ini (mis. control + failsafe + IMU
 *  primer + IMU sekunder + RX drain sekaligus). */
#define WATCHDOG_MAX_MONITORED_TASKS   8U

/**
 * @brief Sumber reset MCU terakhir, dibaca dari `RCC->CSR` SEBELUM
 *        `Watchdog_ClearResetFlags()` dipanggil. Beberapa flag bisa aktif
 *        bersamaan di hardware asli (mis. POR + PIN saat cold-boot) --
 *        enum ini memilih SATU penyebab paling informatif secara prioritas
 *        (lihat urutan prioritas di watchdog.c); pakai
 *        `Watchdog_GetRawResetFlags()` kalau butuh bitmask mentah lengkap.
 */
typedef enum {
    WATCHDOG_RESET_CAUSE_UNKNOWN = 0,  /* RCC->CSR sudah dibersihkan / belum dibaca */
    WATCHDOG_RESET_CAUSE_IWDG,         /* independent watchdog timeout -- lihat
                                        * catatan di atas soal task macet */
    WATCHDOG_RESET_CAUSE_WWDG,         /* window watchdog -- modul ini tidak
                                        * mengaktifkan WWDG, flag ini untuk
                                        * jaga-jaga kalau nanti dipakai */
    WATCHDOG_RESET_CAUSE_SOFTWARE,     /* NVIC_SystemReset() atau semacamnya */
    WATCHDOG_RESET_CAUSE_LOW_POWER,    /* reset dari mode low-power tak wajar */
    WATCHDOG_RESET_CAUSE_BROWNOUT,     /* tegangan supply turun di bawah ambang BOR */
    WATCHDOG_RESET_CAUSE_PIN,          /* tombol reset / NRST eksternal */
    WATCHDOG_RESET_CAUSE_POWER_ON,     /* power-on reset normal */
} WatchdogResetCause_t;

/* ---------------------------------------------------------------------
 * Init & feed dasar
 * ------------------------------------------------------------------- */

/**
 * @brief Baca `RCC->CSR` (SEBELUM di-reset -- lihat Watchdog_ClearResetFlags())
 *        lalu konfigurasi & START IWDG dengan timeout sedekat mungkin ke
 *        `timeout_ms` (dibulatkan ke prescaler/reload terdekat yang bisa
 *        direpresentasikan register 12-bit IWDG, lihat watchdog.c).
 *        WAJIB dipanggil tepat sekali saat boot, SEBELUM loop utama mulai,
 *        dan SETELAH `SysTick_Init()` kalau `Watchdog_RegisterCriticalTask()`
 *        akan dipakai (health check butuh `get_micros()`).
 *
 * PERINGATAN (sama seperti implementasi lama): setelah dipanggil, IWDG
 * TIDAK BISA dimatikan kecuali lewat reset MCU.
 *
 * @param timeout_ms Timeout yang diinginkan, 1 - ~32768 ms (batas atas
 *                    hardware dengan LSI ~32kHz nominal, lihat watchdog.c).
 *                    Nilai di luar rentang akan di-clamp, bukan gagal diam-diam --
 *                    cek log/debug kalau curiga nilai yang dipakai bukan
 *                    yang diminta.
 */
void Watchdog_Init(uint32_t timeout_ms);

/**
 * @brief Kick IWDG mentah, TANPA cek kesehatan apa pun. Setara persis
 *        dengan `watchdog_feed()` lama. Sediakan untuk kasus sebelum
 *        scheduler/task kritis terdaftar (mis. selama urutan init boot),
 *        atau kalau tim sengaja mau kembali ke perilaku lama sementara
 *        untuk debug. Untuk operasi normal, pakai `Watchdog_FeedIfHealthy()`.
 */
void Watchdog_Feed(void);

/* ---------------------------------------------------------------------
 * Health-aware feed (lihat "Kenapa modul ini ada" di atas)
 * ------------------------------------------------------------------- */

/**
 * @brief Daftarkan satu task scheduler sebagai "kritis" -- task yang
 *        TIDAK PERNAH boleh berhenti dijadwalkan selama sistem hidup
 *        (mis. task_control, task_failsafe, task_imu_primary).
 *
 * JANGAN daftarkan task yang SENGAJA dimatikan lewat
 * `Scheduler_SetTaskEnabled()` selama operasi normal (mis. task blackbox
 * yang menurut komentar `scheduler.h` bisa dimatikan saat disarmed) --
 * begitu task terdaftar berhenti dijadwalkan, `last_run_us`-nya berhenti
 * maju dan `Watchdog_IsSystemHealthy()` akan mulai melaporkan tidak sehat
 * sampai IWDG expire. Itu perilaku yang BENAR untuk task yang memang
 * seharusnya selalu jalan, tapi SALAH kalau dipasang ke task yang memang
 * boleh idle.
 *
 * @param sched         Scheduler yang memuat task ini (biasanya `&s_sched`
 *                       satu-satunya instance di main.c).
 * @param task_index     Index balikan `Scheduler_AddTask()` untuk task ini.
 * @param max_gap_us     Batas waktu maksimum sejak eksekusi terakhir
 *                        task ini sebelum dianggap macet/berhenti. Beri
 *                        margin di atas periode nominal task (mis. 3-5x
 *                        periode) supaya satu-dua overrun wajar tidak
 *                        langsung memicu reset -- lihat statistik
 *                        `overrun_count` scheduler untuk kalibrasi angka
 *                        ini sebelum uji terbang.
 * @return true kalau berhasil didaftarkan, false kalau tabel penuh
 *         (naikkan WATCHDOG_MAX_MONITORED_TASKS) atau `task_index` tidak
 *         valid di `sched` (dicek lewat Scheduler_GetTask()).
 */
bool Watchdog_RegisterCriticalTask(const Scheduler_t *sched, int task_index,
                                    uint32_t max_gap_us);

/**
 * @brief true kalau SEMUA task terdaftar (`Watchdog_RegisterCriticalTask()`)
 *        sudah berjalan setidaknya sekali DAN eksekusi terakhirnya masih
 *        dalam batas `max_gap_us` masing-masing, dievaluasi terhadap
 *        `get_micros()` saat ini. false kalau ADA SATU SAJA yang gagal
 *        syarat ini -- lihat `Watchdog_GetLastUnhealthyTaskIndex()` untuk
 *        tahu yang mana.
 *
 * Bisa dipanggil terpisah dari `Watchdog_FeedIfHealthy()` kalau caller
 * cuma mau baca status (mis. untuk ditulis ke blackbox
 * `BLACKBOX_RECORD_EVENT` sebelum akhirnya reset), tapi memanggil ini
 * TIDAK menunda IWDG -- pemanggil tetap harus memanggil
 * `Watchdog_Feed()`/`Watchdog_FeedIfHealthy()` sendiri.
 */
bool Watchdog_IsSystemHealthy(void);

/**
 * @brief Index task (urutan pendaftaran ke modul ini, BUKAN task_index
 *        scheduler) yang menyebabkan evaluasi `Watchdog_IsSystemHealthy()`
 *        PALING TERAKHIR mengembalikan false. -1 kalau evaluasi terakhir
 *        sehat atau belum pernah dievaluasi.
 */
int Watchdog_GetLastUnhealthyTaskIndex(void);

/**
 * @brief Kick IWDG HANYA kalau `Watchdog_IsSystemHealthy()` true. Ini
 *        yang dipanggil pengganti `watchdog_feed()` mentah di loop utama
 *        setelah `Scheduler_Tick()`, kalau ada task kritis terdaftar.
 *        Kalau tidak ada task terdaftar sama sekali, perilakunya sama
 *        dengan `Watchdog_Feed()` tanpa syarat (tidak ada yang bisa
 *        dicek "tidak sehat") -- daftarkan task kritis lebih dulu supaya
 *        fungsi ini benar-benar berguna.
 */
void Watchdog_FeedIfHealthy(void);

/* ---------------------------------------------------------------------
 * Reset cause (untuk telemetry/blackbox, bukan wajib dipakai)
 * ------------------------------------------------------------------- */

/**
 * @brief Bitmask mentah `RCC->CSR` yang DIBACA saat `Watchdog_Init()`
 *        dipanggil (sebelum dibersihkan). Berguna kalau butuh tahu
 *        kombinasi flag persis, bukan cuma satu penyebab dominan.
 */
uint32_t Watchdog_GetRawResetFlags(void);

/** @brief Penyebab reset dominan hasil interpretasi `Watchdog_GetRawResetFlags()`. */
WatchdogResetCause_t Watchdog_GetResetCause(void);

/**
 * @brief Bersihkan flag reset di `RCC->CSR` (tulis bit RMVF). Panggil
 *        SETELAH selesai membaca `Watchdog_GetResetCause()`/
 *        `Watchdog_GetRawResetFlags()` (mis. setelah ditulis ke log boot
 *        atau blackbox) -- flag hardware TIDAK otomatis clear sendiri
 *        dan akan terus terbaca "true" di reset berikutnya kalau tidak
 *        dibersihkan.
 */
void Watchdog_ClearResetFlags(void);

/**
 * @brief Set `DBGMCU->APB1FZ` supaya IWDG berhenti menghitung selama CPU
 *        halt oleh debugger (breakpoint). Panggil SEBELUM breakpoint
 *        pertama kalau sedang debug step-by-step -- TIDAK dipanggil
 *        otomatis oleh `Watchdog_Init()` karena efeknya harus sengaja
 *        (mode ini tidak boleh aktif di build produksi, lihat catatan
 *        lama di `main.c`).
 */
void Watchdog_FreezeDuringDebugHalt(void);

#ifdef __cplusplus
}
#endif

#endif /* WATCHDOG_H */
