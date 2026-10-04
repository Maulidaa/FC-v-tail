#include "watchdog.h"
#include "stm32f4xx.h"
#include "systick.h"

/* ============================================================================
 * watchdog.c
 * Lihat watchdog.h untuk filosofi desain (kenapa feed harus sadar
 * kesehatan scheduler, bukan kick buta) dan peringatan-peringatan penting
 * (IWDG tidak bisa dimatikan, akurasi LSI kasar, dll).
 * ============================================================================ */

/* LSI STM32F411: nominal 32 kHz, TIDAK dikalibrasi pabrik -- datasheet
 * memberi rentang lebar (bisa dobel/separuh nominal tergantung suhu/part).
 * Dipakai sebagai basis perhitungan prescaler/reload; timeout hasil
 * Watchdog_Init() adalah PERKIRAAN, sama seperti asumsi di implementasi
 * lama main.c. */
#define WATCHDOG_LSI_HZ_NOMINAL   32000UL

#define WATCHDOG_RLR_MAX          4095UL   /* register 12-bit */
#define WATCHDOG_PR_MAX           6UL      /* PR: 0..6, divider 4..256 */

static uint32_t             s_raw_reset_flags;
static WatchdogResetCause_t s_reset_cause;

typedef struct {
    const Scheduler_t *sched;
    int                task_index;
    uint32_t           max_gap_us;
} watchdog_monitored_task_t;

static watchdog_monitored_task_t s_monitored[WATCHDOG_MAX_MONITORED_TASKS];
static uint8_t                   s_monitored_count;
static int                       s_last_unhealthy_index = -1;

/* ---------------------------------------------------------------------------
 * Reset cause
 *
 * Prioritas interpretasi (paling spesifik/paling "menarik untuk didiagnosa"
 * duluan): kalau beberapa flag menyala bersamaan (wajar di hardware asli,
 * mis. POR + PIN saat cold-boot dari mati total), yang paling berguna
 * dilaporkan sebagai satu-satunya "penyebab" adalah watchdog/software reset
 * (indikasi bug), bukan power-on yang justru paling sering terjadi dan
 * paling tidak menarik untuk dilihat di log.
 * ------------------------------------------------------------------------- */
static WatchdogResetCause_t interpret_reset_flags(uint32_t csr)
{
    if (csr & RCC_CSR_IWDGRSTF)  { return WATCHDOG_RESET_CAUSE_IWDG; }
    if (csr & RCC_CSR_WWDGRSTF)  { return WATCHDOG_RESET_CAUSE_WWDG; }
    if (csr & RCC_CSR_SFTRSTF)   { return WATCHDOG_RESET_CAUSE_SOFTWARE; }
    if (csr & RCC_CSR_LPWRRSTF)  { return WATCHDOG_RESET_CAUSE_LOW_POWER; }
    if (csr & RCC_CSR_BORRSTF)   { return WATCHDOG_RESET_CAUSE_BROWNOUT; }
    if (csr & RCC_CSR_PINRSTF)   { return WATCHDOG_RESET_CAUSE_PIN; }
    if (csr & RCC_CSR_PORRSTF)   { return WATCHDOG_RESET_CAUSE_POWER_ON; }
    return WATCHDOG_RESET_CAUSE_UNKNOWN;
}

uint32_t Watchdog_GetRawResetFlags(void)
{
    return s_raw_reset_flags;
}

WatchdogResetCause_t Watchdog_GetResetCause(void)
{
    return s_reset_cause;
}

void Watchdog_ClearResetFlags(void)
{
    RCC->CSR |= RCC_CSR_RMVF;
}

void Watchdog_FreezeDuringDebugHalt(void)
{
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
}

/* ---------------------------------------------------------------------------
 * Init & feed dasar
 * ------------------------------------------------------------------------- */

/* Cari (prescaler, reload) terkecil error-nya untuk timeout_ms yang
 * diminta. Dicoba dari divider PALING KECIL (PR=0, /4) supaya resolusi
 * (langkah waktu per unit RLR) paling halus dulu -- baru naik ke divider
 * lebih besar kalau RLR meluap di atas WATCHDOG_RLR_MAX.
 *
 * Formula (RM0383): t_ms = (4 * 2^PR) * (RLR + 1) * 1000 / LSI_Hz
 *   => RLR = t_ms * LSI_Hz / (1000 * 4 * 2^PR) - 1
 */
static void compute_prescaler_reload(uint32_t timeout_ms, uint32_t *out_pr, uint32_t *out_rlr)
{
    uint32_t pr = 0;
    uint32_t rlr = 0;

    for (pr = 0; pr <= WATCHDOG_PR_MAX; pr++) {
        uint32_t divider = 4UL << pr; /* 4,8,16,32,64,128,256 */
        uint64_t numerator = (uint64_t)timeout_ms * WATCHDOG_LSI_HZ_NOMINAL;
        uint64_t denominator = 1000ULL * divider;
        uint64_t rlr64 = (numerator + (denominator / 2ULL)) / denominator; /* dibulatkan */

        if (rlr64 == 0) {
            rlr64 = 1; /* timeout diminta lebih kecil dari resolusi divider ini --
                         * paksa minimum 1 tick daripada RLR=0 (yang berarti
                         * hampir-instan, hampir pasti bukan maksud caller) */
        }
        rlr64 -= 1;

        if (rlr64 <= WATCHDOG_RLR_MAX) {
            rlr = (uint32_t)rlr64;
            break;
        }
        /* rlr64 meluap di divider ini -- coba PR berikutnya (divider lebih
         * besar). Kalau sudah di PR_MAX dan masih meluap, clamp ke maksimum
         * absolut yang bisa direpresentasikan (~32.7 detik @ LSI 32kHz). */
        if (pr == WATCHDOG_PR_MAX) {
            rlr = (uint32_t)WATCHDOG_RLR_MAX;
        }
    }

    *out_pr = pr;
    *out_rlr = rlr;
}

void Watchdog_Init(uint32_t timeout_ms)
{
    /* Tangkap penyebab reset SEBELUM disentuh siapa pun -- ini kesempatan
     * satu-satunya, flag ini bertahan lintas reset sampai RMVF ditulis. */
    s_raw_reset_flags = RCC->CSR;
    s_reset_cause = interpret_reset_flags(s_raw_reset_flags);

    if (timeout_ms == 0U) {
        timeout_ms = WATCHDOG_DEFAULT_TIMEOUT_MS; /* 0 bukan permintaan valid
                                                      * ("timeout instan" tidak
                                                      * ada gunanya) -- pakai
                                                      * default daripada
                                                      * menghasilkan RLR=0 yang
                                                      * langsung reset saat start */
    }

    uint32_t pr, rlr;
    compute_prescaler_reload(timeout_ms, &pr, &rlr);

    IWDG->KR  = 0x5555U;  /* buka akses tulis ke PR/RLR */
    IWDG->PR  = pr;
    IWDG->RLR = rlr;
    IWDG->KR  = 0xAAAAU;  /* reload counter dari RLR */
    IWDG->KR  = 0xCCCCU;  /* start -- tidak bisa di-undo kecuali reset MCU */
}

void Watchdog_Feed(void)
{
    IWDG->KR = 0xAAAAU;
}

/* ---------------------------------------------------------------------------
 * Health-aware feed
 * ------------------------------------------------------------------------- */

bool Watchdog_RegisterCriticalTask(const Scheduler_t *sched, int task_index,
                                    uint32_t max_gap_us)
{
    if (sched == 0) {
        return false;
    }
    if (Scheduler_GetTask(sched, task_index) == 0) {
        return false; /* task_index tidak valid di scheduler ini */
    }
    if (s_monitored_count >= WATCHDOG_MAX_MONITORED_TASKS) {
        return false; /* naikkan WATCHDOG_MAX_MONITORED_TASKS */
    }

    s_monitored[s_monitored_count].sched      = sched;
    s_monitored[s_monitored_count].task_index = task_index;
    s_monitored[s_monitored_count].max_gap_us = max_gap_us;
    s_monitored_count++;
    return true;
}

bool Watchdog_IsSystemHealthy(void)
{
    uint32_t now_us = get_micros();

    s_last_unhealthy_index = -1;

    for (uint8_t i = 0; i < s_monitored_count; i++) {
        const watchdog_monitored_task_t *mon = &s_monitored[i];
        const SchedulerTask_t *task = Scheduler_GetTask(mon->sched, mon->task_index);

        if (task == 0) {
            /* task_index yang dulu valid sekarang tidak -- seharusnya tidak
             * terjadi (scheduler tidak pernah menghapus task terdaftar),
             * tapi kalau terjadi, anggap tidak sehat daripada diam-diam
             * dilewati. */
            s_last_unhealthy_index = (int)i;
            return false;
        }

        /* task->run_count == 0 (belum pernah jalan sama sekali sejak
         * boot/registrasi) ditangani lewat cek staleness yang sama di
         * bawah, bukan dikecualikan -- lihat catatan timing awal boot
         * di Watchdog_RegisterCriticalTask(). */
        uint32_t elapsed_us = now_us - task->last_run_us; /* aritmatika
            * unsigned wrap-safe selama selisihnya < ~71 menit, sama
            * seperti catatan get_micros() di systick.h */

        if (task->run_count == 0U || elapsed_us > mon->max_gap_us) {
            s_last_unhealthy_index = (int)i;
            return false;
        }
    }

    return true;
}

int Watchdog_GetLastUnhealthyTaskIndex(void)
{
    return s_last_unhealthy_index;
}

void Watchdog_FeedIfHealthy(void)
{
    if (Watchdog_IsSystemHealthy()) {
        Watchdog_Feed();
    }
    /* else: sengaja TIDAK di-feed -- biarkan IWDG expire dan MCU reset.
     * Lihat armed_state.h: boot berikutnya otomatis ARMED_STATE_DISARMED,
     * jadi ini fail-safe, bukan fail-dangerous. */
}
