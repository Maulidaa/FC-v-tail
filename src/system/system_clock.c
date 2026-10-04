/**
 * @file    system_clock.c
 * @brief   Implementasi SystemClock_Config(). Lihat system_clock.h untuk
 *          kontrak API dan angka clock tree yang diasumsikan modul BSP.
 *
 * Target: HSE 25 MHz (kristal WeAct Blackpill CEU6)
 *         PLLM=25, PLLN=192, PLLP=2  -> SYSCLK = HCLK = 96 MHz
 *         PLLQ=4                     -> USB OTG FS = 48 MHz tepat
 *         AHB /1, APB1 /2 (48 MHz), APB2 /1 (96 MHz)
 *
 * Catatan urutan (penting, jangan ditukar):
 *   1. HSE dulu — kalau kristal tidak ada, kita bail out SEBELUM menyentuh
 *      flash latency atau PLL, jadi MCU tetap hidup di HSI dan masih bisa
 *      dipakai untuk blink LED pola error.
 *   2. VOS scale 1 SEBELUM PLL — wajib untuk SYSCLK > 64 MHz di F411.
 *   3. Flash latency dinaikkan SEBELUM switch ke clock lebih cepat.
 *      (Kalau diturunkan, urutannya harus dibalik: switch dulu, baru
 *      turunkan latency. Di sini kita cuma naik, jadi aman.)
 *   4. Prescaler APB SEBELUM switch, supaya APB1 tidak pernah sesaat
 *      jalan di 96 MHz (batasnya 50 MHz di F411).
 */

#include "system_clock.h"

/* ---------------------------------------------------------------------------
 * Parameter PLL. Semua nilai ini HARUS konsisten dengan SYSTEM_CORE_CLOCK_HZ
 * di system_clock.h dan dengan APB1_CLOCK_MHZ/APB2_CLOCK_HZ_LOCAL yang
 * di-hardcode di bsp_i2c.c dan bsp_uart.c. Kalau salah satu diubah, ketiganya
 * wajib diubah bersamaan — belum ada satu sumber kebenaran tunggal untuk ini.
 * ------------------------------------------------------------------------- */
#define HSE_FREQ_HZ        25000000UL

#define PLL_M              25U      /* VCO input  = 25 MHz / 25 = 1 MHz   */
#define PLL_N              192U     /* VCO output = 1 MHz * 192 = 192 MHz */
#define PLL_P              2U       /* SYSCLK     = 192 MHz / 2 = 96 MHz  */
#define PLL_Q              4U       /* USB        = 192 MHz / 4 = 48 MHz  */

/* Posisi bit di RCC_PLLCFGR (ditulis eksplisit, tidak bergantung ke macro
 * *_Pos yang hanya ada di CMSIS versi baru). */
#define PLLCFGR_PLLM_SHIFT   0U
#define PLLCFGR_PLLN_SHIFT   6U
#define PLLCFGR_PLLP_SHIFT   16U
#define PLLCFGR_PLLSRC_HSE   (1UL << 22)
#define PLLCFGR_PLLQ_SHIFT   24U

/* Timeout busy-wait. Nilainya loop-count, bukan waktu — di titik ini
 * SysTick belum jalan, jadi tidak ada time-base yang bisa dipakai.
 * ~0.5 juta iterasi @16 MHz HSI ≈ puluhan ms, jauh di atas waktu wajar
 * HSE startup (beberapa ms) dan PLL lock (<200 us). */
#define CLOCK_TIMEOUT_LOOPS  500000UL

/* CMSIS SystemCoreClock: proyek ini TIDAK menyertakan system_stm32f4xx.c,
 * jadi variabelnya kita sediakan di sini. Kalau nanti file CMSIS itu
 * ditambahkan ke build, HAPUS definisi di bawah ini supaya tidak jadi
 * duplicate symbol saat link. */
uint32_t SystemCoreClock = SYSTEM_CORE_CLOCK_HZ;

/* ---------------------------------------------------------------------------
 * SystemInit() — dipanggil startup_stm32f411ceux.s sebelum main(), sebelum
 * .data/.bss di-copy dan sebelum __libc_init_array.
 *
 * *** KENAPA INI WAJIB ADA ***
 * Proyek dikompilasi dengan -mfloat-abi=hard -mfpu=fpv4-sp-d16, artinya
 * kompiler bebas mengeluarkan instruksi VFP (vmul.f32, vldr.32, dst).
 * Di Cortex-M4F, coprocessor CP10/CP11 DISABLED setelah reset. Instruksi
 * float pertama yang dieksekusi akan memicu UsageFault (bit NOCP) ->
 * HardFault, sebelum main() sempat berbuat apa-apa. Sebelum patch ini,
 * TIDAK ADA satu baris pun di repo yang menulis ke CPACR — jadi begitu
 * modul apa pun yang memakai float (pid.c, mixer.c, ahrs_fusion.c,
 * battery.c, navigation.c — praktis semuanya) dieksekusi, firmware mati.
 *
 * Jangan taruh konfigurasi clock di sini: SystemInit() tidak bisa
 * mengembalikan status error, sementara kegagalan HSE perlu ditangani
 * caller. Clock tetap lewat SystemClock_Config() dari main().
 * ------------------------------------------------------------------------- */
void SystemInit(void)
{
#if defined(__FPU_PRESENT) && (__FPU_PRESENT == 1U)
    /* CPACR (0xE000ED88): set CP10 & CP11 ke full access (0b11 masing-masing,
     * bit 20-23). */
    SCB->CPACR |= ((3UL << 20) | (3UL << 22));

    /* Barrier: pastikan penulisan CPACR sudah efektif sebelum instruksi
     * berikutnya di-fetch — tanpa ini, instruksi float yang sudah masuk
     * pipeline bisa tetap kena fault. */
    __DSB();
    __ISB();
#endif

    /* Vector table di awal flash (0x08000000). Relevan kalau nanti ada
     * bootloader yang menggeser aplikasi ke offset lain. */
    SCB->VTOR = FLASH_BASE;
}

/* ---------------------------------------------------------------------------
 * SystemClock_Config()
 * ------------------------------------------------------------------------- */
system_clock_status_t SystemClock_Config(void)
{
    uint32_t timeout;

    /* --- 1. Nyalakan HSE, tunggu ready --- */
    RCC->CR |= RCC_CR_HSEON;

    timeout = CLOCK_TIMEOUT_LOOPS;
    while ((RCC->CR & RCC_CR_HSERDY) == 0U) {
        if (--timeout == 0U) {
            /* Kristal tidak terpasang / rusak. Belum ada yang diubah
             * (flash latency, PLL, prescaler masih default reset), jadi
             * MCU aman lanjut di HSI 16 MHz. Caller yang memutuskan
             * tindakan — JANGAN diam-diam lanjut seolah 96 MHz. */
            RCC->CR &= ~RCC_CR_HSEON;
            return SYSTEM_CLOCK_ERR_HSE_TIMEOUT;
        }
    }

    /* --- 2. Power scale 1 (wajib untuk >64 MHz di F411) --- */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR; /* dummy read: jaminan clock gating sudah efektif
                          * sebelum register PWR disentuh (errata umum F4) */

    PWR->CR |= PWR_CR_VOS; /* field 2-bit, 0b11 = Scale 1 */

    /* JANGAN poll VOSRDY di sini. Ini bug yang sudah dikonfirmasi lewat
     * dua sumber independen (forum resmi ST + thread CubeMX): di
     * STM32F411, VOSRDY tidak akan pernah menjadi 1 sebelum PLL
     * diaktifkan -- bit ini baru valid/live setelah LL_RCC_PLL_Enable()
     * (tahap 6 di bawah). Polling di sini SELALU timeout terlepas dari
     * kondisi hardware, dan itulah yang menghasilkan pola kedip 2x
     * (SYSTEM_CLOCK_ERR_PLL_TIMEOUT) meski HSE dan kristal 25MHz sudah
     * benar. Set VOS saja lalu lanjut; verifikasi VOSRDY yang sebenarnya
     * berguna dipindah ke setelah PLL enable (lihat tahap 6). */

    /* --- 3. Flash: prefetch + I/D cache + 3 wait state untuk 96 MHz @3.3V ---
     * Tabel F411 (2.7-3.6V): 0WS <=30MHz, 1WS <=64, 2WS <=90, 3WS <=100.
     * Dinaikkan SEBELUM SYSCLK naik — kalau tidak, CPU akan membaca flash
     * terlalu cepat dan mengeksekusi instruksi sampah. */
    FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN |
                 FLASH_ACR_LATENCY_3WS;

    /* Verifikasi latency benar-benar tertulis. Kalau tidak, jangan lanjut
     * menaikkan clock. */
    if ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_3WS) {
        return SYSTEM_CLOCK_ERR_PLL_TIMEOUT;
    }

    /* --- 4. Prescaler bus, SEBELUM switch ke PLL ---
     * APB1 maks 50 MHz di F411, jadi /2 (48 MHz) harus sudah aktif
     * sebelum SYSCLK menyentuh 96 MHz. */
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1    /* AHB  = 96 MHz */
               | RCC_CFGR_PPRE1_DIV2   /* APB1 = 48 MHz */
               | RCC_CFGR_PPRE2_DIV1;  /* APB2 = 96 MHz */

    /* --- 5. Konfigurasi PLL (harus dalam kondisi PLL mati) --- */
    RCC->CR &= ~RCC_CR_PLLON;
    timeout = CLOCK_TIMEOUT_LOOPS;
    while ((RCC->CR & RCC_CR_PLLRDY) != 0U) {
        if (--timeout == 0U) {
            return SYSTEM_CLOCK_ERR_PLL_TIMEOUT;
        }
    }

    RCC->PLLCFGR = ((uint32_t)PLL_M << PLLCFGR_PLLM_SHIFT)
                 | ((uint32_t)PLL_N << PLLCFGR_PLLN_SHIFT)
                 | ((uint32_t)((PLL_P >> 1U) - 1U) << PLLCFGR_PLLP_SHIFT)
                 | ((uint32_t)PLL_Q << PLLCFGR_PLLQ_SHIFT)
                 | PLLCFGR_PLLSRC_HSE;

    /* --- 6. Nyalakan PLL, tunggu lock --- */
    RCC->CR |= RCC_CR_PLLON;
    timeout = CLOCK_TIMEOUT_LOOPS;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0U) {
        if (--timeout == 0U) {
            return SYSTEM_CLOCK_ERR_PLL_TIMEOUT;
        }
    }

    /* --- 6b. Verifikasi VOSRDY (opsional) -- BARU valid dicek di sini,
     * setelah PLL enable, sesuai perilaku hardware F411 yang sebenarnya
     * (bukan di tahap 2 seperti kode lama). Kalau timeout di sini,
     * berarti benar-benar ada masalah voltage scaling, bukan cuma
     * false alarm seperti sebelumnya. */
    timeout = CLOCK_TIMEOUT_LOOPS;
    while ((PWR->CSR & PWR_CSR_VOSRDY) == 0U) {
        if (--timeout == 0U) {
            return SYSTEM_CLOCK_ERR_PLL_TIMEOUT;
        }
    }

    /* --- 7. Pindahkan SYSCLK ke PLL, tunggu konfirmasi SWS --- */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    timeout = CLOCK_TIMEOUT_LOOPS;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
        if (--timeout == 0U) {
            return SYSTEM_CLOCK_ERR_PLL_TIMEOUT;
        }
    }

    /* --- 8. HSI sudah tidak dipakai, matikan untuk hemat daya. Boleh
     *        dibiarkan ON kalau nanti ada fitur clock-failure fallback. --- */
    RCC->CR &= ~RCC_CR_HSION;

    SystemCoreClock = SYSTEM_CORE_CLOCK_HZ;

    return SYSTEM_CLOCK_OK;
}
