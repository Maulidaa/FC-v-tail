/**
 * calib_accel_gyro.c
 *
 * Lihat calib_accel_gyro.h untuk kontrak publik.
 *
 * Algoritma: kumpulkan N sample selama device didiamkan, hitung rata-
 * rata (bias) accel & gyro. Untuk memastikan device benar-benar diam
 * (bukan cuma dipercaya user), dihitung juga variance kasar gyro
 * selama window — kalau variance di atas ambang, kalibrasi ditandai
 * FAILED daripada menyimpan bias yang salah dari device yang bergerak.
 */

#include "calib_accel_gyro.h"
#include <stddef.h>

#define TARGET_SAMPLES     (500u)  /* @100Hz ~ 5 detik pengumpulan */
#define GYRO_VAR_THRESHOLD (1600u)  /* ambang variance kasar, tuning empiris */

static CalibAccelGyroState_t s_state = CALIB_AG_STATE_IDLE;
static uint32_t s_sample_count;

static int64_t s_sum_ax, s_sum_ay, s_sum_az;
static int64_t s_sum_gx, s_sum_gy, s_sum_gz;
static int64_t s_sum_gx2, s_sum_gy2, s_sum_gz2; /* untuk variance kasar */

static AccelGyroBias_t s_result;

void CalibAccelGyro_Start(void)
{
    s_state = CALIB_AG_STATE_IN_PROGRESS;
    s_sample_count = 0;
    s_sum_ax = s_sum_ay = s_sum_az = 0;
    s_sum_gx = s_sum_gy = s_sum_gz = 0;
    s_sum_gx2 = s_sum_gy2 = s_sum_gz2 = 0;
}

void CalibAccelGyro_Stop(void)
{
    s_state = CALIB_AG_STATE_IDLE;
}

static uint32_t rough_variance(int64_t sum, int64_t sum_sq, uint32_t n)
{
    /* Variance populasi kasar: E[x^2] - (E[x])^2, dibulatkan ke uint32
     * karena cuma dipakai sebagai ambang relatif, bukan nilai presisi
     * statistik yang perlu akurat sampai desimal. */
    int64_t mean = sum / (int64_t)n;
    int64_t mean_sq = sum_sq / (int64_t)n;
    int64_t var = mean_sq - (mean * mean);
    return (var > 0) ? (uint32_t)var : 0u;
}

void CalibAccelGyro_FeedSample(int16_t ax, int16_t ay, int16_t az,
                                int16_t gx, int16_t gy, int16_t gz)
{
    if (s_state != CALIB_AG_STATE_IN_PROGRESS) {
        return;
    }

    s_sum_ax += ax;
    s_sum_ay += ay;
    s_sum_az += az;
    s_sum_gx += gx;
    s_sum_gy += gy;
    s_sum_gz += gz;
    s_sum_gx2 += (int64_t)gx * (int64_t)gx;
    s_sum_gy2 += (int64_t)gy * (int64_t)gy;
    s_sum_gz2 += (int64_t)gz * (int64_t)gz;
    s_sample_count++;

    if (s_sample_count >= TARGET_SAMPLES) {
        uint32_t var_x = rough_variance(s_sum_gx, s_sum_gx2, s_sample_count);
        uint32_t var_y = rough_variance(s_sum_gy, s_sum_gy2, s_sample_count);
        uint32_t var_z = rough_variance(s_sum_gz, s_sum_gz2, s_sample_count);

        if (var_x > GYRO_VAR_THRESHOLD || var_y > GYRO_VAR_THRESHOLD
            || var_z > GYRO_VAR_THRESHOLD) {
            /* Device bergerak selama pengumpulan — bias gyro tidak
             * bisa dipercaya, jangan disimpan */
            s_state = CALIB_AG_STATE_FAILED;
            return;
        }

        s_result.accel_bias_x = (int16_t)(s_sum_ax / (int64_t)s_sample_count);
        s_result.accel_bias_y = (int16_t)(s_sum_ay / (int64_t)s_sample_count);
        s_result.accel_bias_z = (int16_t)(s_sum_az / (int64_t)s_sample_count);
        s_result.gyro_bias_x  = (int16_t)(s_sum_gx / (int64_t)s_sample_count);
        s_result.gyro_bias_y  = (int16_t)(s_sum_gy / (int64_t)s_sample_count);
        s_result.gyro_bias_z  = (int16_t)(s_sum_gz / (int64_t)s_sample_count);

        s_state = CALIB_AG_STATE_DONE;
    }
}

CalibAccelGyroState_t CalibAccelGyro_GetState(void)
{
    return s_state;
}

uint8_t CalibAccelGyro_GetProgressPercent(void)
{
    if (s_state == CALIB_AG_STATE_DONE) {
        return 100u;
    }
    if (s_state != CALIB_AG_STATE_IN_PROGRESS) {
        return 0u;
    }
    uint32_t pct = (s_sample_count * 100u) / TARGET_SAMPLES;
    return (pct > 100u) ? 100u : (uint8_t)pct;
}

bool CalibAccelGyro_GetResult(AccelGyroBias_t *out)
{
    if (s_state != CALIB_AG_STATE_DONE || out == NULL) {
        return false;
    }
    *out = s_result;
    return true;
}
