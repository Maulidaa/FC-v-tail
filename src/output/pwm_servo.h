/**
 * @file    pwm_servo.h
 * @brief   Driver PWM 50Hz untuk 4 channel servo (2 aileron + 2 ruddervator)
 *          — TIM3_CH3/CH4 (PB0/PB1) dan TIM4_CH3/CH4 (PB8/PB9). Lihat
 *          firmware-architecture-stm32f411.md Bagian 3.8 dan
 *          pinout-fc-stm32f411.md Bagian 1.
 *
 * Urutan enum PwmServoChannel_t SENGAJA disamakan dengan konvensi
 * driver_index yang dipakai OutputMap_InitDefault() di output_map.c
 * (index 0-3 = AIL_L, AIL_R, VTAIL_L, VTAIL_R) — kalau urutan ini diubah,
 * output_map.c WAJIB ikut disesuaikan.
 *
 * === Dependency ke bsp_timer_pwm.c/h (Orang 1) ===
 * Sama seperti dshot.h, konversi posisi->microsecond dipisah dari akses
 * register TIM3/TIM4. Fungsi berikut diasumsikan disediakan
 * `bsp_timer_pwm.c/h`:
 *
 *   bool BSP_TimerPWM_ServoInit(void);
 *   void BSP_TimerPWM_ServoSetPulseUs(uint8_t channel_index, uint16_t pulse_us);
 *
 * `channel_index` di sini adalah nilai PwmServoChannel_t (0-3) — bsp layer
 * yang tahu index mana masuk ke TIM3 vs TIM4, channel mana.
 */

#ifndef PWM_SERVO_H
#define PWM_SERVO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PWM_SERVO_CH_AIL_L = 0,   /* PB0 / TIM3_CH3 */
    PWM_SERVO_CH_AIL_R,       /* PB1 / TIM3_CH4 */
    PWM_SERVO_CH_VTAIL_L,     /* PB8 / TIM4_CH3 */
    PWM_SERVO_CH_VTAIL_R,     /* PB9 / TIM4_CH4 */
    PWM_SERVO_CH_COUNT
} PwmServoChannel_t;

/**
 * @brief Pulse width servo standar dalam microsecond. 1500us = netral.
 *        Rentang 1000-2000us adalah standar hobby servo paling umum;
 *        kalau servo tertentu di board kalian butuh rentang beda (mis.
 *        800-2200us untuk throw lebih lebar), ubah lewat
 *        PwmServo_SetPulseRange() per channel, JANGAN ubah #define ini
 *        (supaya default tetap aman/konservatif untuk servo yang belum
 *        dikalibrasi).
 */
#define PWM_SERVO_PULSE_MIN_US     (1000u)
#define PWM_SERVO_PULSE_CENTER_US  (1500u)
#define PWM_SERVO_PULSE_MAX_US     (2000u)

/**
 * @brief Rentang pulse per channel — dipisah dari kode supaya bisa
 *        dikalibrasi per servo (beberapa servo throw fisiknya beda) lewat
 *        setting, tanpa mengubah pwm_servo.c.
 */
typedef struct {
    uint16_t pulse_min_us;
    uint16_t pulse_center_us;
    uint16_t pulse_max_us;
} PwmServoRange_t;

/**
 * @brief Inisialisasi TIM3/TIM4 sebagai PWM 50Hz (lewat
 *        BSP_TimerPWM_ServoInit()) dan reset semua channel ke posisi netral
 *        (PWM_SERVO_PULSE_CENTER_US). WAJIB dipanggil sebelum
 *        PwmServo_SetPosition().
 */
bool PwmServo_Init(void);

/**
 * @brief Override rentang pulse default untuk satu channel. Opsional —
 *        kalau tidak dipanggil, channel pakai default MIN/CENTER/MAX di
 *        atas.
 */
void PwmServo_SetPulseRange(PwmServoChannel_t channel, uint16_t pulse_min_us,
                             uint16_t pulse_center_us, uint16_t pulse_max_us);

/**
 * @brief Set posisi servo, dinormalisasi [-1.0, +1.0] (0.0 = netral),
 *        konsisten dengan konvensi output servo di mixer.h. Dikonversi ke
 *        pulse width sesuai PwmServoRange_t channel ini, lalu ditulis
 *        lewat BSP_TimerPWM_ServoSetPulseUs().
 *
 * Interpolasi linear terpisah untuk sisi negatif (min..center) dan
 * positif (center..max), supaya rentang tidak simetris (mis. kalibrasi
 * servo yang center-nya bukan tepat di tengah min/max) tetap presisi di
 * kedua arah.
 */
void PwmServo_SetPosition(uint8_t channel_index, float position_minus1_1);

#ifdef __cplusplus
}
#endif

#endif /* PWM_SERVO_H */
