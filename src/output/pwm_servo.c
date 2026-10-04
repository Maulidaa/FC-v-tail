/**
 * @file    pwm_servo.c
 * @brief   Implementasi PWM servo. Lihat pwm_servo.h untuk dokumentasi API
 *          dan kontrak dependency ke bsp_timer_pwm.c.
 */

#include "pwm_servo.h"

/* --- Dependency dari bsp_timer_pwm.c (lihat kontrak di pwm_servo.h) --- */
extern bool BSP_TimerPWM_ServoInit(void);
extern void BSP_TimerPWM_ServoSetPulseUs(uint8_t channel_index, uint16_t pulse_us);

static PwmServoRange_t s_ranges[PWM_SERVO_CH_COUNT];
static bool s_initialized = false;

static inline float pwm_servo_clampf(float value, float min, float max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

static void pwm_servo_set_default_range(PwmServoRange_t *range)
{
    range->pulse_min_us = PWM_SERVO_PULSE_MIN_US;
    range->pulse_center_us = PWM_SERVO_PULSE_CENTER_US;
    range->pulse_max_us = PWM_SERVO_PULSE_MAX_US;
}

bool PwmServo_Init(void)
{
    for (uint8_t i = 0; i < PWM_SERVO_CH_COUNT; i++) {
        pwm_servo_set_default_range(&s_ranges[i]);
    }

    s_initialized = BSP_TimerPWM_ServoInit();

    if (s_initialized) {
        /* Posisikan semua servo netral segera setelah init, jangan tunggu
         * update pertama dari mixer/output_map — servo tidak boleh diam di
         * posisi listrik-mati (0us) atau posisi acak power-on. */
        for (uint8_t i = 0; i < PWM_SERVO_CH_COUNT; i++) {
            BSP_TimerPWM_ServoSetPulseUs(i, s_ranges[i].pulse_center_us);
        }
    }

    return s_initialized;
}

void PwmServo_SetPulseRange(PwmServoChannel_t channel, uint16_t pulse_min_us,
                             uint16_t pulse_center_us, uint16_t pulse_max_us)
{
    if (channel >= PWM_SERVO_CH_COUNT) {
        return;
    }
    if (!(pulse_min_us < pulse_center_us && pulse_center_us < pulse_max_us)) {
        /* Rentang tidak masuk akal (mis. dari input setting yang korup) —
         * diabaikan, channel tetap pakai rentang sebelumnya, jangan sampai
         * servo dapat pulse yang tidak monoton. */
        return;
    }
    s_ranges[channel].pulse_min_us = pulse_min_us;
    s_ranges[channel].pulse_center_us = pulse_center_us;
    s_ranges[channel].pulse_max_us = pulse_max_us;
}

void PwmServo_SetPosition(uint8_t channel_index, float position_minus1_1)
{
    if (!s_initialized || channel_index >= PWM_SERVO_CH_COUNT) {
        return;
    }

    float position = pwm_servo_clampf(position_minus1_1, -1.0f, 1.0f);
    const PwmServoRange_t *range = &s_ranges[channel_index];

    uint16_t pulse_us;
    if (position >= 0.0f) {
        uint16_t half_range = (uint16_t)(range->pulse_max_us - range->pulse_center_us);
        pulse_us = (uint16_t)(range->pulse_center_us + (uint16_t)(position * (float)half_range));
    } else {
        uint16_t half_range = (uint16_t)(range->pulse_center_us - range->pulse_min_us);
        pulse_us = (uint16_t)(range->pulse_center_us + (int16_t)(position * (float)half_range));
    }

    BSP_TimerPWM_ServoSetPulseUs(channel_index, pulse_us);
}
