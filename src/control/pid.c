/**
 * @file    pid.c
 * @brief   Implementasi PID generik. Lihat pid.h untuk dokumentasi API.
 */

#include "pid.h"
#include <math.h>
#include <stddef.h>   /* NULL -- sebelumnya hanya ikut terbawa lewat math.h di
                        * toolchain tertentu; bukan jaminan, dan gagal kompilasi
                        * di gcc host. */

/* dt di luar rentang ini dianggap tidak masuk akal (scheduler overrun/jitter
 * parah atau first-call sebelum dt sempat terukur) — lihat catatan di
 * firmware-architecture-stm32f411.md 3.15 soal deteksi overrun/jitter scheduler. */
#define PID_DT_MIN_SECONDS   (1.0e-5f)
#define PID_DT_MAX_SECONDS   (0.5f)

static inline float pid_clampf(float value, float min, float max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->output_min = -1.0f;
    pid->output_max = 1.0f;

    pid->integral_min = pid->output_min;
    pid->integral_max = pid->output_max;

    pid->derivative_on_measurement = true;
    pid->d_filter_alpha = 0.0f;

    PID_Reset(pid);
}

void PID_SetGains(PID_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_SetOutputLimits(PID_t *pid, float min, float max)
{
    if (pid == NULL || min >= max) {
        return;
    }
    pid->output_min = min;
    pid->output_max = max;

    /* Kalau integral limits belum pernah di-set eksplisit lewat
     * PID_SetIntegralLimits(), ikutkan supaya tetap konsisten. */
    pid->integral_min = min;
    pid->integral_max = max;

    /* Jaga integral state yang sudah ada tetap dalam batas baru. */
    pid->_integral = pid_clampf(pid->_integral, pid->integral_min, pid->integral_max);
}

void PID_SetIntegralLimits(PID_t *pid, float min, float max)
{
    if (pid == NULL || min >= max) {
        return;
    }
    pid->integral_min = min;
    pid->integral_max = max;
    pid->_integral = pid_clampf(pid->_integral, pid->integral_min, pid->integral_max);
}

void PID_SetOptions(PID_t *pid, bool derivative_on_measurement, float d_filter_alpha)
{
    if (pid == NULL) {
        return;
    }
    pid->derivative_on_measurement = derivative_on_measurement;
    pid->d_filter_alpha = pid_clampf(d_filter_alpha, 0.0f, 0.999f);
}

void PID_Reset(PID_t *pid)
{
    if (pid == NULL) {
        return;
    }
    pid->_integral = 0.0f;
    pid->_prev_measurement = 0.0f;
    pid->_prev_error = 0.0f;
    pid->_prev_derivative_filtered = 0.0f;
    pid->_has_prev_measurement = false;
    pid->_has_prev_error = false;
}

PID_Result_t PID_Compute(PID_t *pid, float setpoint, float measurement, float dt_seconds)
{
    PID_Result_t result = {0};

    if (pid == NULL) {
        return result;
    }

    /* dt tidak masuk akal (overrun/jitter atau first-call) -> jangan update
     * integral/derivative, cukup kembalikan P term murni supaya loop tidak
     * "meracuni" state dengan dt yang salah. */
    bool dt_valid = (dt_seconds >= PID_DT_MIN_SECONDS) && (dt_seconds <= PID_DT_MAX_SECONDS);

    float error = setpoint - measurement;

    /* --- Proportional --- */
    result.p_term = pid->kp * error;

    if (!dt_valid) {
        result.i_term = pid->ki * pid->_integral;
        result.d_term = 0.0f;
        result.output = pid_clampf(result.p_term + result.i_term + result.d_term,
                                    pid->output_min, pid->output_max);
        return result;
    }

    /* --- Integral (dengan anti-windup via clamp) --- */
    pid->_integral += error * dt_seconds;
    pid->_integral = pid_clampf(pid->_integral, pid->integral_min, pid->integral_max);
    result.i_term = pid->ki * pid->_integral;

    /* --- Derivative --- */
    float raw_derivative;
    if (pid->derivative_on_measurement) {
        /* Derivative-on-measurement: hindari derivative kick saat setpoint
         * melompat (mis. step input dari RX stick). */
        if (pid->_has_prev_measurement) {
            raw_derivative = -(measurement - pid->_prev_measurement) / dt_seconds;
        } else {
            raw_derivative = 0.0f;
        }
        pid->_prev_measurement = measurement;
        pid->_has_prev_measurement = true;
    } else {
        /* Derivative-on-error = d(error)/dt, yaitu SELISIH error antar
         * sample dibagi dt.
         *
         * BUG yang diperbaiki: sebelumnya baris ini menulis
         * `error / dt_seconds` -- itu bukan turunan, itu error dibagi dt.
         * Akibatnya d_term ikut membesar sebanding error yang KONSTAN
         * (padahal turunan error konstan = 0) dan nilainya melonjak
         * ~1/dt, mis. 200x pada dt=5ms. Jalur ini TIDAK aktif di firmware
         * saat ini (PID_Init() menyetel derivative_on_measurement = true
         * dan PID_SetOptions() tidak pernah dipanggil), jadi tidak ada
         * perubahan perilaku terbang -- tapi jebakannya nyata begitu ada
         * yang mengubah opsi itu lewat setting. */
        if (pid->_has_prev_error) {
            raw_derivative = (error - pid->_prev_error) / dt_seconds;
        } else {
            raw_derivative = 0.0f;
        }
        pid->_prev_error = error;
        pid->_has_prev_error = true;

        /* prev_measurement tidak relevan di mode ini, tapi tetap disimpan
         * supaya konsisten kalau mode di-switch belakangan. */
        pid->_prev_measurement = measurement;
        pid->_has_prev_measurement = true;
    }

    float filtered_derivative;
    if (pid->d_filter_alpha > 0.0f) {
        filtered_derivative = (pid->d_filter_alpha * pid->_prev_derivative_filtered) +
                               ((1.0f - pid->d_filter_alpha) * raw_derivative);
    } else {
        filtered_derivative = raw_derivative;
    }
    pid->_prev_derivative_filtered = filtered_derivative;

    result.d_term = pid->kd * filtered_derivative;

    /* --- Jumlahkan & clamp output akhir --- */
    result.output = pid_clampf(result.p_term + result.i_term + result.d_term,
                                pid->output_min, pid->output_max);

    return result;
}
