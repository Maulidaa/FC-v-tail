#ifndef SYSTEM_CLOCK_H
#define SYSTEM_CLOCK_H

#include "stm32f4xx.h"   /* CMSIS device header, target STM32F411xE */

/* ============================================================================
 * system_clock.h
 * Konfigurasi RCC aktual (HSE -> PLL -> SYSCLK/AHB/APB1/APB2) sesuai clock
 * tree final yang sudah diasumsikan oleh SELURUH modul BSP (bsp_spi.h,
 * bsp_i2c.h, bsp_uart.h, bsp_adc.h, bsp_timer_pwm.h):
 *
 *   HSE = 25 MHz (kristal onboard WeAct Blackpill CEU6)
 *   PLLM = 25, PLLN = 192, PLLP = 2  -> SYSCLK = HCLK = 96 MHz
 *   PLLQ = 4                         -> USB OTG FS clock = 48 MHz tepat
 *   APB1 prescaler /2                -> PCLK1 = 48 MHz
 *   APB2 prescaler /1                -> PCLK2 = 96 MHz
 *
 * PENTING: fungsi di modul ini WAJIB dipanggil PALING AWAL di main(),
 * SEBELUM BSP_PinMap_Init() atau modul BSP lain mana pun -- semua
 * kalkulasi baud rate (bsp_uart.c), prescaler SPI (bsp_spi.c), CCR/TRISE
 * I2C (bsp_i2c.c), dan tick timer (bsp_timer_pwm.c) mengasumsikan clock
 * ini sudah aktif SEBELUM peripheral-nya di-init. Tanpa ini, MCU tetap
 * jalan di HSI 16MHz default reset dan semua angka baud/prescaler yang
 * sudah dihitung di modul BSP lain akan SALAH TOTAL (bukan sekadar
 * kurang presisi).
 * ============================================================================ */

#define SYSTEM_CORE_CLOCK_HZ   96000000UL

typedef enum {
    SYSTEM_CLOCK_OK             = 0,
    SYSTEM_CLOCK_ERR_HSE_TIMEOUT = -1, /* HSE tidak ready -- kristal 25MHz tidak terdeteksi/tidak terpasang */
    SYSTEM_CLOCK_ERR_PLL_TIMEOUT = -2, /* PLL tidak lock -- seharusnya tidak terjadi kalau HSE sudah ready dan parameter PLL valid */
} system_clock_status_t;

/**
 * @brief Konfigurasi RCC penuh: enable HSE, set power scale voltage
 *        (VOS scale 1, wajib untuk clock >64MHz di STM32F411), set flash
 *        latency (3 wait states untuk 96MHz @ 2.7-3.6V), konfigurasi PLL
 *        (M=25,N=192,P=2,Q=4), pindah SYSCLK ke PLL, set prescaler
 *        AHB=/1, APB1=/2, APB2=/1. Update juga variabel global CMSIS
 *        `SystemCoreClock` supaya fungsi CMSIS lain yang bergantung
 *        padanya (mis. SysTick_Config) tetap akurat.
 *
 *        WAJIB dipanggil pertama kali di main(), sebelum modul BSP
 *        apa pun (lihat catatan di atas).
 *
 * @return SYSTEM_CLOCK_OK kalau berhasil, kode error kalau HSE/PLL gagal
 *         start (mis. kristal tidak terpasang) -- caller (main.c)
 *         bertanggung jawab memutuskan tindakan lanjut (mis. LED blink
 *         pola error, karena watchdog/error.c/h belum tentu siap sepagi
 *         ini di urutan boot).
 */
system_clock_status_t SystemClock_Config(void);

#endif /* SYSTEM_CLOCK_H */
