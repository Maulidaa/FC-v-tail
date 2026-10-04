/**
 * @file    dshot.c
 * @brief   Adapter output PWM ESC. Nama file/API dipertahankan untuk
 *          kompatibilitas dengan output_map.c.
 */

#include "dshot.h"

/* Hobbywing Skywalker memakai PWM servo-style. Nama API DShot dipertahankan
 * sementara supaya kontrak output_map tidak berubah. */
extern void BSP_TIM_EscPwm_Init(void);
extern void BSP_TIM_EscPwm_SetPulseUs(uint16_t pulse_us);

#define ESC_PWM_MIN_US (1000u)
#define ESC_PWM_MAX_US (2000u)

static float s_throttle_command[DSHOT_MAX_MOTORS];
static bool s_initialized = false;

static inline float dshot_clampf(float value, float min, float max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

/** Konversi throttle normalisasi menjadi pulsa PWM ESC 1000..2000us. */
static uint16_t esc_throttle_to_pulse_us(float throttle_0_1)
{
    float clamped = dshot_clampf(throttle_0_1, 0.0f, 1.0f);

    if (clamped <= 0.0f) {
        return ESC_PWM_MIN_US;
    }

    return (uint16_t)(ESC_PWM_MIN_US +
                      (uint16_t)(clamped * (float)(ESC_PWM_MAX_US - ESC_PWM_MIN_US)));
}

bool DShot_Init(void)
{
    for (uint8_t i = 0; i < DSHOT_MAX_MOTORS; i++) {
        s_throttle_command[i] = 0.0f;
    }

    BSP_TIM_EscPwm_Init();
    s_initialized = true;
    return s_initialized;
}

void DShot_SetThrottle(uint8_t motor_index, float throttle_0_1)
{
    if (motor_index >= DSHOT_MAX_MOTORS) {
        return;
    }
    s_throttle_command[motor_index] = dshot_clampf(throttle_0_1, 0.0f, 1.0f);
}

bool DShot_SendFrames(void)
{
    if (!s_initialized) {
        return false;
    }

    for (uint8_t i = 0; i < DSHOT_MAX_MOTORS; i++) {
        if (i == 0u) {
            BSP_TIM_EscPwm_SetPulseUs(esc_throttle_to_pulse_us(s_throttle_command[i]));
        }
    }
    return true;
}
