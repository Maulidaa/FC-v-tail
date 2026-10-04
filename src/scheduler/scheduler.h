/**
 * @file    scheduler.h
 * @brief   Scheduler cooperative sederhana berbasis get_micros(), untuk
 *          menjalankan task periodik dengan laju berbeda-beda dari satu
 *          main loop tunggal.
 *
 * Kenapa cooperative, bukan RTOS:
 * - Semua task di firmware ini pendek dan tidak pernah menunggu (driver
 *   I2C/SPI-nya blocking tapi bounded). Tidak ada yang butuh preemption,
 *   jadi biaya RTOS (stack per task, context switch, sinkronisasi) tidak
 *   terbayar.
 * - Cooperative = tidak ada race antar task. State bersama (attitude,
 *   channel RX, setpoint nav) bisa diakses tanpa mutex. Ini menghilangkan
 *   seluruh kelas bug yang paling sulit di-debug di firmware kecil.
 *
 * Konsekuensinya: satu task yang lama akan menunda SEMUA task lain. Karena
 * itu scheduler ini mencatat waktu eksekusi maksimum dan jumlah overrun per
 * task — angka itu wajib dilihat sebelum uji terbang, jangan diabaikan.
 *
 * Kontrak dt:
 *   Callback menerima dt_seconds = jarak waktu SEBENARNYA sejak task ini
 *   terakhir jalan, bukan periode nominalnya. Ini yang harus diteruskan ke
 *   PID_Compute() dan AHRS_Fusion_Update() — keduanya sudah punya guard
 *   untuk dt di luar batas wajar, dan guard itu baru berguna kalau yang
 *   diberi memang dt terukur.
 */

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SCHEDULER_MAX_TASKS   16U

/**
 * @brief Signature callback task.
 * @param ctx        Pointer konteks yang didaftarkan bersama task.
 * @param dt_seconds Waktu terukur sejak task ini terakhir dijalankan.
 *                   Pada eksekusi pertama, nilainya = periode nominal.
 */
typedef void (*SchedulerTaskFn)(void *ctx, float dt_seconds);

typedef struct {
    const char     *name;        /* untuk telemetry/debug, tidak dipakai logic */
    SchedulerTaskFn fn;
    void           *ctx;

    uint32_t period_us;
    uint32_t next_due_us;
    uint32_t last_run_us;

    /* --- Statistik, read-only dari luar --- */
    uint32_t max_exec_us;        /* eksekusi terlama yang pernah tercatat */
    uint32_t overrun_count;      /* berapa kali task terlambat > 1 periode penuh */
    uint32_t run_count;

    bool enabled;
    bool _has_run_once;
} SchedulerTask_t;

typedef struct {
    SchedulerTask_t tasks[SCHEDULER_MAX_TASKS];
    uint8_t  task_count;

    /* Statistik loop keseluruhan. loop_count naik tiap satu putaran penuh
     * Scheduler_Tick(); kalau angkanya jauh lebih kecil dari yang
     * diharapkan, berarti ada task yang memakan waktu. */
    uint32_t loop_count;
    uint32_t idle_loop_count;    /* putaran yang tidak menjalankan task apa pun */

    bool initialized;
} Scheduler_t;

/** Reset scheduler ke kondisi kosong. Panggil sebelum Scheduler_AddTask(). */
void Scheduler_Init(Scheduler_t *sched);

/**
 * @brief Daftarkan satu task periodik.
 *
 * @param rate_hz Laju eksekusi yang diinginkan. Harus > 0. Periode dihitung
 *                sebagai 1e6/rate_hz mikrodetik (dibulatkan ke bawah).
 * @return Index task (>= 0) kalau berhasil, -1 kalau tabel penuh atau
 *         parameter tidak valid. Caller WAJIB mengecek nilai balik ini —
 *         task yang diam-diam gagal didaftarkan berarti loop kontrol atau
 *         failsafe yang tidak pernah jalan.
 */
int Scheduler_AddTask(Scheduler_t *sched, const char *name, SchedulerTaskFn fn,
                      void *ctx, uint32_t rate_hz);

/** Aktif/nonaktifkan task tanpa menghapusnya (mis. matikan blackbox saat
 *  disarmed). Saat diaktifkan kembali, jadwalnya di-resync ke waktu
 *  sekarang supaya tidak langsung "mengejar" ratusan eksekusi tertunda. */
void Scheduler_SetTaskEnabled(Scheduler_t *sched, int task_index, bool enabled);

/**
 * @brief Jalankan satu putaran scheduler: cek semua task, eksekusi yang
 *        sudah jatuh tempo. Tidak memblokir — panggil terus-menerus dari
 *        while(1) di main().
 *
 * Kebijakan saat task terlambat: jadwal berikutnya dimajukan satu periode
 * dari jadwal sebelumnya (bukan dari waktu sekarang), supaya laju rata-rata
 * tetap akurat dan tidak melenceng pelan-pelan. TAPI kalau ketertinggalannya
 * sudah lebih dari satu periode penuh, jadwal di-resync ke waktu sekarang
 * dan overrun_count dinaikkan — tanpa ini, task yang sempat tertinggal jauh
 * akan mencoba mengejar dengan mengeksekusi diri berkali-kali beruntun dan
 * mengunci seluruh loop (death spiral).
 */
void Scheduler_Tick(Scheduler_t *sched);

/** Akses statistik satu task, NULL kalau index tidak valid. */
const SchedulerTask_t *Scheduler_GetTask(const Scheduler_t *sched, int task_index);

/** Total overrun seluruh task. Angka yang terus naik saat terbang = sinyal
 *  budget CPU sudah habis; turunkan rate task atau percepat driver-nya. */
uint32_t Scheduler_GetTotalOverruns(const Scheduler_t *sched);

#ifdef __cplusplus
}
#endif

#endif /* SCHEDULER_H */
