#ifndef BSP_TIMER_PWM_H
#define BSP_TIMER_PWM_H

#include "stm32f4xx.h"   /* CMSIS device header, target STM32F411xE */
#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
 * bsp_timer_pwm.h
 * TIM1 -> MOTOR0 DShot (PA8/CH1), digerakkan DMA burst ke CCR1.
 * TIM3 -> SERVO0/1 Aileron (PB0=CH3/AIL_L, PB1=CH4/AIL_R), PWM 50Hz biasa.
 * TIM4 -> SERVO2/3 Ruddervator (PB8=CH3/VTAIL_L, PB9=CH4/VTAIL_R), PWM 50Hz.
 *
 * GPIO mode/AF pin-pin di atas SUDAH dikonfigurasi oleh BSP_PinMap_Init()
 * -- modul ini hanya menyentuh register TIM1/TIM3/TIM4 + DMA2 Stream1.
 *
 * Clock timer (final, lihat bsp_spi.h utk sumber kebenaran clock tree):
 *   TIM1 (APB2, prescaler /1)          -> clock timer = 96 MHz
 *   TIM3/TIM4 (APB1, prescaler /2)     -> clock timer = 2 x 48MHz = 96 MHz
 *   (aturan STM32F4: kalau prescaler APBx != 1, clock timer = 2x pclk)
 *   Kebetulan keduanya sama-sama 96MHz -- memudahkan perhitungan tick.
 *
 * ----------------------------------------------------------------------
 * PEMBAGIAN TANGGUNG JAWAB (penting, baca sebelum pakai dari output_map.c):
 * Modul ini HANYA menyediakan:
 *   1. Servo: fungsi set pulsa langsung dalam mikrodetik (servo tidak
 *      butuh encoding apa pun, jadi API-nya sudah lengkap di sini).
 *   2. DShot: TIMING dasar (bit period, threshold tick untuk bit 1/0) dan
 *      fungsi BSP_TIM_DShot_SendFrame() yang MENGIRIM array 16 nilai duty
 *      yang SUDAH dihitung oleh caller.
 * ENCODING paket DShot (susun throttle 11-bit + telemetry-request bit +
 * checksum 4-bit jadi 16 nilai duty tick, pakai DSHOT_HIGH_TICKS_BIT1/0
 * di bawah) TIDAK dilakukan di sini -- itu logika protokol DShot murni,
 * jadi tanggung jawab output_map.c (Orang 4), bukan BSP.
 * ----------------------------------------------------------------------
 * DMA mapping DShot (final, closing item checklist Bagian G):
 *   TIM1_CH1 -> DMA2 Stream1 / Channel6 (tidak bentrok dengan ADC1 yang
 *   dipetakan ke DMA2 Stream0 / Channel0 -- lihat bsp_adc.h).
 *   Teknik: CCDS=1 (CR2) supaya request DMA "CC1" ini sebenarnya dipicu
 *   di setiap UPDATE EVENT (bukan capture-compare match) -- ini teknik
 *   standar untuk DShot bit-banging via timer (dipakai banyak firmware FC
 *   open-source), supaya tiap awal periode bit timer otomatis dapat nilai
 *   CCR1 baru dari DMA sebelum bit itu mulai dibentuk.
 * ============================================================================ */

/* ---------------------------------------------------------------------------
 * Servo (TIM3/TIM4) - PWM 50Hz standar
 * ------------------------------------------------------------------------- */

#define SERVO_PWM_FREQ_HZ      50u
#define SERVO_PULSE_MIN_US     1000u
#define SERVO_PULSE_MAX_US     2000u
#define SERVO_PULSE_CENTER_US  1500u

typedef enum {
    SERVO_CH_AIL_L   = 0, /* TIM3_CH3, PB0 */
    SERVO_CH_AIL_R   = 1, /* TIM3_CH4, PB1 */
    SERVO_CH_VTAIL_L = 2, /* TIM4_CH3, PB8 */
    SERVO_CH_VTAIL_R = 3, /* TIM4_CH4, PB9 */
} servo_channel_t;

/**
 * @brief Init TIM3 & TIM4 untuk PWM 50Hz (period 20ms, resolusi tick 1us)
 *        di keempat channel servo, preload+auto-reload preload aktif,
 *        lalu start counter. Nilai awal semua channel: SERVO_PULSE_CENTER_US
 *        (netral/tengah) -- BUKAN 0, supaya servo tidak "mengejar" ke
 *        posisi ekstrem/acak begitu power-up sebelum control loop jalan.
 *        Panggil sekali saat boot, setelah BSP_PinMap_Init().
 */
void BSP_TIM_Servo_Init(void);

/**
 * @brief Set lebar pulsa satu channel servo. Nilai di luar rentang
 *        [SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US] di-clamp otomatis --
 *        caller (mixer.c, Orang 4) tidak perlu clamp sendiri, tapi juga
 *        tidak akan mendapat error kalau input di luar rentang.
 * @param ch channel servo
 * @param pulse_us lebar pulsa dalam mikrodetik
 */
void BSP_TIM_Servo_SetPulseUs(servo_channel_t ch, uint16_t pulse_us);

/* ---------------------------------------------------------------------------
 * DShot (TIM1 + DMA2 Stream1)
 * ------------------------------------------------------------------------- */

/* Ganti nilai ini sesuai ESC yang dipakai tim (150/300/600 umum dipakai).
 * DSHOT300 dipilih sebagai default -- kompromi aman: cukup cepat untuk
 * update rate kontrol wajar, tapi tidak serewel DSHOT600 soal kualitas
 * sinyal di kabel panjang / wiring prototipe. */
#define DSHOT_PROTOCOL_KHZ      300u
#define DSHOT_FRAME_BITS        16u

#define TIM1_CLOCK_HZ_LOCAL     96000000u

/* Tick per bit period, dan threshold tick untuk merepresentasikan bit 1
 * (75% duty) vs bit 0 (37.5% duty) -- standar timing protokol DShot.
 * Dihitung sebagai konstanta lewat preprocessor supaya konsisten dengan
 * DSHOT_PROTOCOL_KHZ tanpa perlu fungsi runtime. */
#define DSHOT_BIT_PERIOD_TICKS  ((uint16_t)(TIM1_CLOCK_HZ_LOCAL / (DSHOT_PROTOCOL_KHZ * 1000u)))
#define DSHOT_HIGH_TICKS_BIT1   ((uint16_t)((DSHOT_BIT_PERIOD_TICKS * 3u) / 4u))   /* 75% */
#define DSHOT_HIGH_TICKS_BIT0   ((uint16_t)((DSHOT_BIT_PERIOD_TICKS * 3u) / 8u))   /* 37.5% */

/**
 * @brief Init TIM1 CH1 untuk PWM mode1 di frekuensi bit DShot
 *        (DSHOT_BIT_PERIOD_TICKS per periode), CCDS=1 supaya DMA request
 *        terpicu di update event (lihat catatan header). Timer langsung
 *        di-start dan berjalan TERUS-MENERUS (CCR1=0 saat idle, artinya
 *        line ditahan low / tidak ada frame valid) -- ini normal untuk
 *        DShot, ESC menunggu frame valid berikutnya.
 *        Panggil sekali saat boot, setelah BSP_PinMap_Init().
 */
void BSP_TIM_DShot_Init(void);

/**
 * @brief Kirim satu frame DShot (DMA burst 16 nilai duty ke CCR1).
 *        Buffer HARUS sudah berisi tick duty per bit (pakai
 *        DSHOT_HIGH_TICKS_BIT1/BIT0 dari header ini) -- fungsi ini murni
 *        mengangkut array tsb ke hardware, tidak melakukan encoding.
 *        Non-blocking: fungsi return segera setelah DMA di-arm, tidak
 *        menunggu transfer selesai.
 * @param duty_ticks array tick duty, panjang HARUS DSHOT_FRAME_BITS (16)
 * @param sample_count panjang array (safety check, harus == DSHOT_FRAME_BITS)
 * @return 1 kalau frame berhasil di-arm ke DMA, 0 kalau ditolak karena
 *         frame sebelumnya masih dalam proses transfer (caller sebaiknya
 *         skip siklus ini, jangan retry paksa dalam loop yang sama) atau
 *         karena sample_count tidak sesuai.
 */
int BSP_TIM_DShot_SendFrame(const uint16_t *duty_ticks, uint16_t sample_count);

/* ---------------------------------------------------------------------------
 * Wrapper kompatibilitas: pwm_servo.c dan dshot.c (Orang 4) ditulis dengan
 * asumsi nama/kontrak fungsi sedikit berbeda dari yang diimplementasikan di
 * atas (BSP_TIM_Servo_* / BSP_TIM_DShot_*). Wrapper ini menjembatani tanpa
 * mengubah implementasi asli di atas.
 * ------------------------------------------------------------------------- */
bool BSP_TimerPWM_ServoInit(void);
void BSP_TimerPWM_ServoSetPulseUs(uint8_t channel_index, uint16_t pulse_us);
void BSP_TIM_EscPwm_Init(void);
void BSP_TIM_EscPwm_SetPulseUs(uint16_t pulse_us);
bool BSP_DShotTimer_Init(void);
bool BSP_DShotTimer_IsTransferBusy(void);
bool BSP_DShotTimer_SendFrameDMA(const uint16_t *duty_ticks, uint16_t length);

#endif /* BSP_TIMER_PWM_H */
