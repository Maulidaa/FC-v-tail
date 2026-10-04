/**
 * @file    pid.h
 * @brief   PID generik — dipakai bersama oleh loop stabilisasi attitude
 *          (roll/pitch) dan altitude hold. Lihat firmware-architecture-stm32f411.md
 *          Bagian 3.7.
 *
 * Catatan desain:
 * - STM32F411 (Cortex-M4F) punya FPU hardware single-precision (hardfp) —
 *   modul ini pakai float, BUKAN double, konsisten dengan kebijakan di
 *   firmware-architecture-stm32f411.md Bagian 3.6 (hindari emulasi software
 *   float untuk perhitungan yang jalan tiap siklus).
 * - Satu instance PID_t = satu axis/loop (roll, pitch, altitude, dst).
 *   Buat instance terpisah per loop, jangan reuse satu instance untuk banyak axis.
 * - Output dinormalisasi ke rentang yang di-set lewat PID_SetOutputLimits()
 *   (mis. -500..+500 sesuai konvensi channel di Bagian 3.7), sebelum diteruskan
 *   ke mixer.c/h.
 */

#ifndef PID_H
#define PID_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief State + parameter satu loop PID.
 *
 * Semua field publik supaya bisa dibaca modul lain (mis. untuk telemetry/debug),
 * tapi field yang diawali underscore dianggap internal — jangan ditulis manual
 * dari luar modul, hanya dibaca.
 */
typedef struct {
    /* --- Parameter (boleh diubah runtime, mis. dari SETTING_SET) --- */
    float kp;
    float ki;
    float kd;

    float output_min;
    float output_max;

    /* Anti-windup: batas integral term secara terpisah dari output limit,
     * supaya kontribusi I tidak "membanjiri" P+D saat output sudah saturasi. */
    float integral_min;
    float integral_max;

    /* Derivative-on-measurement (bukan derivative-on-error) untuk menghindari
     * "derivative kick" saat setpoint berubah mendadak (mis. RX stick step). */
    bool derivative_on_measurement;

    /* Low-pass filter sederhana (single-pole) untuk term derivative, supaya
     * tidak ikut memperkuat noise sensor mentah. 0.0 = filter mati (raw),
     * mendekati 1.0 = filter berat/lambat. */
    float d_filter_alpha;

    /* --- State internal (jangan ditulis manual dari luar) --- */
    float _integral;
    float _prev_measurement;
    float _prev_error;                 /* hanya dipakai saat
                                        * derivative_on_measurement == false */
    float _prev_derivative_filtered;
    bool  _has_prev_measurement;
    bool  _has_prev_error;
} PID_t;

/**
 * @brief Hasil satu kali eksekusi PID_Compute(), dipecah per term.
 *        Berguna untuk logging blackbox / debug tuning tanpa perlu
 *        menghitung ulang kontribusi tiap term secara manual.
 */
typedef struct {
    float p_term;
    float i_term;
    float d_term;
    float output; /* p_term + i_term + d_term, sudah di-clamp ke output_min/max */
} PID_Result_t;

/**
 * @brief Inisialisasi instance PID dengan gain awal. Mereset seluruh state
 *        internal (integral, prev_measurement) ke nol/kosong.
 *
 * @param pid Instance yang akan diinisialisasi.
 * @param kp, ki, kd Gain awal (boleh 0, diisi/diubah belakangan lewat
 *                   PID_SetGains() begitu skema setting tunable siap).
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd);

/**
 * @brief Update gain PID saat runtime (mis. dipanggil dari SETTING_SET
 *        handler). Tidak mereset integral state — supaya perubahan gain
 *        pas terbang tidak menyebabkan lonjakan output mendadak.
 */
void PID_SetGains(PID_t *pid, float kp, float ki, float kd);

/**
 * @brief Set batas output akhir (clamp p+i+d).
 */
void PID_SetOutputLimits(PID_t *pid, float min, float max);

/**
 * @brief Set batas integral term secara terpisah (anti-windup).
 *        Kalau tidak pernah dipanggil, default integral_min/max mengikuti
 *        output_min/max yang di-set lewat PID_SetOutputLimits().
 */
void PID_SetIntegralLimits(PID_t *pid, float min, float max);

/**
 * @brief Konfigurasi opsi tambahan: derivative-on-measurement dan koefisien
 *        low-pass filter untuk term D.
 *
 * @param d_filter_alpha Rentang [0.0, 1.0). 0.0 = tidak ada filtering.
 */
void PID_SetOptions(PID_t *pid, bool derivative_on_measurement, float d_filter_alpha);

/**
 * @brief Reset seluruh state internal (integral, prev_measurement) tanpa
 *        mengubah gain/limits. Wajib dipanggil saat:
 *        - transisi disarmed -> armed (mulai loop kontrol dari kondisi bersih)
 *        - mode switch yang mengubah makna setpoint/measurement loop ini
 *          (mis. keluar dari altitude hold)
 *        supaya integral term lama tidak "menempel" dan menyebabkan lonjakan.
 */
void PID_Reset(PID_t *pid);

/**
 * @brief Jalankan satu iterasi perhitungan PID.
 *
 * @param pid          Instance PID (state di-update in-place).
 * @param setpoint     Target (mis. target roll rate, target altitude).
 * @param measurement  Nilai terukur saat ini (mis. dari AHRS/baro).
 * @param dt_seconds   Delta waktu sejak panggilan terakhir, dalam detik.
 *                     Harus > 0; kalau <= 0 atau tidak masuk akal (mis. akibat
 *                     scheduler overrun/jitter), fungsi ini mengembalikan
 *                     output terakhir tanpa update integral/derivative,
 *                     supaya jitter timing tidak meracuni loop kontrol.
 * @return Struct berisi kontribusi tiap term + output akhir (sudah di-clamp).
 */
PID_Result_t PID_Compute(PID_t *pid, float setpoint, float measurement, float dt_seconds);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
