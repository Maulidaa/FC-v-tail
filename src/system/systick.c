#include "systick.h"

/* ---- SysTick registers (ARM Cortex-M4 generic, bukan spesifik ST) ---- */
#define SYSTICK_BASE     0xE000E010UL
#define SYSTICK_CTRL     (*(volatile uint32_t *)(SYSTICK_BASE + 0x00))
#define SYSTICK_LOAD     (*(volatile uint32_t *)(SYSTICK_BASE + 0x04))
#define SYSTICK_VAL      (*(volatile uint32_t *)(SYSTICK_BASE + 0x08))

#define SYSTICK_CTRL_ENABLE     (1UL << 0)
#define SYSTICK_CTRL_TICKINT    (1UL << 1)
#define SYSTICK_CTRL_CLKSOURCE  (1UL << 2) // 1 = pakai processor clock langsung

/* SCB_SHPR3 dipakai untuk set prioritas SysTick lewat register langsung
 * (angka besar = prioritas rendah) supaya ISR peripheral yang benar-benar
 * time-critical (USART1 CRSF @420k, DMA DShot) tidak pernah tertunda oleh
 * tick counter. Sebelumnya tidak pernah di-set sama sekali — artinya
 * SysTick jalan di prioritas 0, prioritas TERTINGGI, mendahului semuanya. */
#define SCB_SHPR3        (*(volatile uint32_t *)0xE000ED20UL)

static volatile uint32_t s_tick_ms = 0;

/* Jumlah tick SysTick per 1ms, disimpan supaya get_micros() tidak perlu
 * membaca ulang SYSTICK_LOAD tiap kali. */
static uint32_t s_ticks_per_ms = 1;

void SysTick_Init(uint32_t core_clock_hz)
{
    s_tick_ms      = 0;
    s_ticks_per_ms = core_clock_hz / 1000UL;

    SYSTICK_LOAD = s_ticks_per_ms - 1UL; // reload tiap 1ms

    SCB_SHPR3 = (SCB_SHPR3 & 0x00FFFFFFUL) | (0xF0UL << 24); // prioritas terendah

    SYSTICK_VAL = 0;
    SYSTICK_CTRL = SYSTICK_CTRL_CLKSOURCE | SYSTICK_CTRL_TICKINT | SYSTICK_CTRL_ENABLE;
}

/* Nama fungsi ini HARUS persis "SysTick_Handler" supaya terpasang ke
 * vector table (weak symbol override di startup_stm32f411xe.s). */
void SysTick_Handler(void)
{
    s_tick_ms++;
}

uint32_t get_tick_ms(void)
{
    return s_tick_ms;
}

uint32_t get_micros(void)
{
    uint32_t ms, val;

    /* Baca ms dan VAL sebagai pasangan yang konsisten: kalau SysTick_Handler
     * menyelip di antara dua pembacaan, s_tick_ms berubah dan kita ulangi.
     * Tanpa loop ini, hasilnya bisa meleset satu milidetik penuh di momen
     * counter wrap — persis momen yang paling sering kena, karena scheduler
     * memanggil fungsi ini ribuan kali per detik. */
    do {
        ms  = s_tick_ms;
        val = SYSTICK_VAL;
    } while (ms != s_tick_ms);

    /* VAL menghitung TURUN dari LOAD = (s_ticks_per_ms - 1) ke 0. Jadi
     * jumlah tick yang SUDAH berlalu di dalam milidetik ini adalah
     * (LOAD - val), bukan (s_ticks_per_ms - val). Versi lama meleset satu
     * tick ke atas: tepat di awal milidetik ia melaporkan 1 tick berlalu,
     * dan di akhir milidetik ia melaporkan 1000 us -- yaitu nilai yang
     * seharusnya sudah milik milidetik BERIKUTNYA. Efeknya kecil (~1 us)
     * tapi membuat get_micros() sesaat sama dengan batas ms berikutnya. */
    uint32_t load = s_ticks_per_ms - 1UL;
    uint32_t elapsed_ticks = (val <= load) ? (load - val) : 0UL;

    return (ms * 1000UL) + ((elapsed_ticks * 1000UL) / s_ticks_per_ms);
}

void delay_ms(uint32_t ms)
{
    uint32_t start = s_tick_ms;
    while ((s_tick_ms - start) < ms) {
        // busy-wait; nanti bisa diganti __WFI() untuk hemat daya kalau perlu
    }
}

void delay_us(uint32_t us)
{
    uint32_t start = get_micros();
    while ((get_micros() - start) < us) {
        // busy-wait
    }
}
