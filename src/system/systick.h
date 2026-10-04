#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>

/** Init SysTick supaya interrupt tiap 1ms. Panggil sekali di awal boot,
 * SETELAH SystemClock_Config() dan SEBELUM modul sensor manapun.
 *
 * @param core_clock_hz HARUS sama dengan clock CPU yang sebenarnya aktif.
 *        Pakai SYSTEM_CORE_CLOCK_HZ dari system_clock.h — jangan menulis
 *        angka literal di sini, itu sumber bug kalau clock tree berubah. */
void SysTick_Init(uint32_t core_clock_hz);

/** Tick counter dalam ms sejak boot -- dipakai semua modul sensor untuk timestamp. */
uint32_t get_tick_ms(void);

/** Waktu dalam mikrodetik sejak boot, dibaca dari kombinasi tick counter +
 * isi register SysTick VAL. Resolusi ~1/96 us, wrap tiap ~71 menit
 * (aritmatika unsigned 32-bit, jadi selisih (now - then) tetap benar
 * melewati wrap selama intervalnya < 71 menit).
 *
 * Dipakai scheduler untuk penjadwalan sub-milidetik — tick 1ms terlalu
 * kasar untuk loop 1kHz.
 *
 * PERINGATAN: panggil dari thread context (main loop), bukan dari dalam
 * ISR yang prioritasnya lebih tinggi dari SysTick. Kalau counter wrap
 * sementara SysTick_Handler masih pending, hasilnya bisa meleset ~1ms. */
uint32_t get_micros(void);

/** Delay blocking sederhana, berbasis tick counter SysTick. */
void delay_ms(uint32_t ms);

/** Delay blocking resolusi mikrodetik. Untuk delay pendek saja (< 1ms,
 * mis. timing bit-bang atau settling sensor); jangan dipakai di dalam
 * task scheduler karena benar-benar memblokir. */
void delay_us(uint32_t us);

#endif // SYSTICK_H
