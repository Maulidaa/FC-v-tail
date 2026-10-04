/* Test host-side untuk fusion/ahrs_fusion.c, memakai implementasi ASLI.
 * Modul ini hanya bergantung ke math.h/string.h, jadi bisa dilink di PC.
 */
#include "test_common.h"
#include <string.h>
#include "ahrs_fusion.h"

/* Sample "pesawat rata & diam": accel Z = 1g, gyro nol, kecuali yaw rate
 * yang diberikan pemanggil. */
static AHRS_ImuSample_t level_sample(float gyro_z_rad_s, bool valid)
{
    AHRS_ImuSample_t s;
    memset(&s, 0, sizeof(s));
    s.accel_x = 0.0f;
    s.accel_y = 0.0f;
    s.accel_z = 1.0f;
    s.gyro_z  = gyro_z_rad_s;
    s.data_valid = valid;
    return s;
}

int main(void)
{
    /* mag tidak dites di sini -- compass.c dites/diverifikasi terpisah
     * (butuh verifikasi bangku fisik, lihat compass.c). no_mag dipakai
     * di semua panggilan supaya seluruh test file ini tetap memverifikasi
     * regresi nol jalur gyro-only (fallback mag->valid=false). */
    const AHRS_MagSample_t no_mag = { .heading_deg = 0.0f, .valid = false };

    /* --- 1. Validitas attitude ---------------------------------------- */
    {
        AHRS_FusionState_t st;
        AHRS_Fusion_Init(&st);

        AHRS_ImuSample_t bad = level_sample(0.0f, false);
        AHRS_Fusion_Update(&st, &bad, &bad, &no_mag, 100U);
        CHECK(!AHRS_Fusion_IsAttitudeValid(&st));   /* keduanya timeout */

        AHRS_ImuSample_t good = level_sample(0.0f, true);
        AHRS_Fusion_Update(&st, &good, &bad, &no_mag, 105U);
        CHECK(AHRS_Fusion_IsAttitudeValid(&st));
        CHECK(AHRS_Fusion_GetAttitude(&st)->active_imu == AHRS_IMU_MPU6500);

        /* Primer mati, sekunder hidup -> failover */
        AHRS_Fusion_Update(&st, &bad, &good, &no_mag, 110U);
        CHECK(AHRS_Fusion_IsAttitudeValid(&st));
        CHECK(AHRS_Fusion_GetAttitude(&st)->active_imu == AHRS_IMU_MPU6050);
    }

    /* --- 2. Pesawat rata -> roll & pitch mendekati nol ----------------- */
    {
        AHRS_FusionState_t st;
        AHRS_Fusion_Init(&st);
        AHRS_ImuSample_t s = level_sample(0.0f, true);
        for (uint32_t t = 0; t < 2000U; t += 5U) {
            AHRS_Fusion_Update(&st, &s, &s, &no_mag, t);
        }
        const AHRS_Attitude_t *a = AHRS_Fusion_GetAttitude(&st);
        CHECK_NEAR(a->roll_deg,  0.0f, 0.5);
        CHECK_NEAR(a->pitch_deg, 0.0f, 0.5);
    }

    /* --- 3. REGRESI: yaw selalu berada di [0,360) ---------------------- *
     * Yaw di modul ini murni integrasi gyro selama mag tidak valid. Tanpa
     * wrap, memutar pesawat terus-menerus (atau sekadar membiarkan drift
     * berjalan lama) membuat yaw_deg tumbuh tanpa batas dan melanggar
     * kontrak "heading 0..360" yang dipegang OLED_View_t dan CMD_ATTITUDE. */
    {
        AHRS_FusionState_t st;
        AHRS_Fusion_Init(&st);

        /* ~57.3 deg/s (1 rad/s) selama 40 detik = ~2292 derajat, lebih dari
         * enam putaran penuh. */
        AHRS_ImuSample_t spin = level_sample(1.0f, true);
        AHRS_ImuSample_t dead = level_sample(0.0f, false);

        for (uint32_t t = 0; t < 40000U; t += 5U) {
            AHRS_Fusion_Update(&st, &spin, &dead, &no_mag, t);
            const AHRS_Attitude_t *a = AHRS_Fusion_GetAttitude(&st);
            if (!(a->yaw_deg >= 0.0f && a->yaw_deg < 360.0f)) {
                printf("  yaw keluar rentang di t=%u: %.3f\n",
                       (unsigned)t, (double)a->yaw_deg);
                CHECK(false);
                break;
            }
        }
        const AHRS_Attitude_t *a = AHRS_Fusion_GetAttitude(&st);
        CHECK(a->yaw_deg >= 0.0f && a->yaw_deg < 360.0f);

        /* Arah putaran negatif juga harus tetap di dalam rentang, bukan
         * berubah jadi nilai negatif. */
        AHRS_FusionState_t st2;
        AHRS_Fusion_Init(&st2);
        AHRS_ImuSample_t spin_neg = level_sample(-1.0f, true);
        for (uint32_t t = 0; t < 40000U; t += 5U) {
            AHRS_Fusion_Update(&st2, &spin_neg, &dead, &no_mag, t);
        }
        const AHRS_Attitude_t *a2 = AHRS_Fusion_GetAttitude(&st2);
        CHECK(a2->yaw_deg >= 0.0f && a2->yaw_deg < 360.0f);
    }

    TEST_SUMMARY("ahrs_fusion");
}
