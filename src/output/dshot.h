/**
 * @file    dshot.h
 * @brief   Driver DShot (protokol digital ESC) untuk channel MOTOR —
 *          PA8/TIM1_CH1, dipicu DMA. Lihat firmware-architecture-stm32f411.md
 *          Bagian 3.8 dan hasil cek DMA2 Stream1/Ch6 untuk TIM1_CH1
 *          (dibahas sebelumnya di percakapan koordinasi tim).
 *
 * v1 hanya 1 motor aktif (airframe V-tail), tapi API dibuat array-based
 * (DSHOT_MAX_MOTORS) mengikuti pola output_map.h yang generic — kalau
 * airframe lain butuh >1 motor, driver ini tidak perlu didesain ulang.
 *
 * === Dependency ke bsp_timer_pwm.c/h (Orang 1) ===
 * Encoding bit DShot (murni matematika) dipisah dari akses
 * register TIM1/DMA — supaya logic encode/CRC bisa dites host-side tanpa
 * hardware. Fungsi berikut diasumsikan disediakan `bsp_timer_pwm.c/h`:
 *
 *   bool BSP_DShotTimer_Init(void);
 *   bool BSP_DShotTimer_IsTransferBusy(void);
 *   bool BSP_DShotTimer_SendFrameDMA(const uint16_t *duty_ticks, uint16_t length);
 *
 * `duty_ticks` isinya nilai CCR1 per bit periode (sudah dikonversi dari
 * bit 1/0 DShot ke tick timer oleh dshot.c) — bsp layer tinggal DMA-kan
 * array ini ke register CCR1 satu kali per frame, TIDAK perlu tahu soal
 * arti tiap nilainya.
 *
 * === Item terbuka yang perlu diverifikasi tim (BUKAN blocker desain) ===
 * Varian kecepatan DShot (DSHOT300 vs DSHOT600) belum dikonfirmasi cocok
 * dengan ESC yang dipakai — dshot.h di-set default DSHOT600 lewat
 * DSHOT_BIT_PERIOD_TICKS, tapi WAJIB dicek ke datasheet/manual ESC
 * sebelum dipakai untuk terbang. Lihat komentar di dshot.c.
 */

#ifndef DSHOT_H
#define DSHOT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DSHOT_MAX_MOTORS (1u) /* airframe v1 (V-tail): 1 motor */

/**
 * @brief Inisialisasi timer+DMA untuk DShot (lewat BSP_DShotTimer_Init())
 *        dan reset seluruh nilai throttle internal ke disarmed (0).
 *        WAJIB dipanggil sebelum DShot_SetThrottle()/DShot_SendFrames().
 */
bool DShot_Init(void);

/**
 * @brief Set nilai throttle yang ingin dikirim untuk satu motor. TIDAK
 *        langsung mengirim frame ke ESC — hanya menyimpan nilai; pengiriman
 *        aktual terjadi saat DShot_SendFrames() dipanggil scheduler
 *        (lihat catatan rate di dshot.c).
 *
 * @param motor_index    0..DSHOT_MAX_MOTORS-1.
 * @param throttle_0_1   [0.0, 1.0]. 0.0 = motor stop (mengirim DShot value 0,
 *                        bukan throttle minimum 48 — value 0 = "disarmed/off"
 *                        di protokol DShot, konsisten dengan cara
 *                        output_map.c memaksa 0.0f saat disarmed).
 */
void DShot_SetThrottle(uint8_t motor_index, float throttle_0_1);

/**
 * @brief Encode & kirim satu frame DShot untuk setiap motor yang
 *        DSHOT_MAX_MOTORS > 0, berdasarkan nilai terakhir dari
 *        DShot_SetThrottle(). Dipanggil scheduler pada rate tetap (mis.
 *        1kHz — lihat catatan minimum inter-frame gap di dshot.c).
 *
 * Non-blocking: kalau transfer DMA frame sebelumnya belum selesai
 * (BSP_DShotTimer_IsTransferBusy() == true), panggilan ini di-skip untuk
 * motor tersebut (frame lama tetap "terkirim", tidak menumpuk antrian) —
 * mencegah scheduler task ini nge-block task lain kalau DMA telat.
 *
 * @return true kalau semua motor berhasil dikirim frame baru pada
 *         panggilan ini, false kalau ada yang di-skip karena busy.
 */
bool DShot_SendFrames(void);

#ifdef __cplusplus
}
#endif

#endif /* DSHOT_H */
