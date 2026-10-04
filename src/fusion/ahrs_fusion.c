/**
 * @file    ahrs_fusion.c
 * @brief   Implementasi AHRS/fusion dual-IMU + arbitrasi disagreement.
 *
 * Status: SKELETON — struktur logika sudah lengkap, tapi bagian
 * complementary filter masih pakai persamaan dasar (belum quaternion,
 * belum kompensasi bias gyro). Cukup untuk mulai integrasi dengan
 * Orang 2 (driver) dan Orang 1 (scheduler), tapi TODO tuning/akurasi
 * masih terbuka sebelum uji terbang.
 */

#include "ahrs_fusion.h"
#include <math.h>
#include <string.h>

/* ------------------------------------------------------------------- */
/* Helper statis                                                        */
/* ------------------------------------------------------------------- */

#define RAD_TO_DEG_F   (57.29577951308232f)

/** Hitung roll/pitch dari accel saja (referensi gravitasi).
 *  Yaw tidak bisa dihitung dari accel — perlu mag atau gyro-integrated,
 *  untuk skeleton ini yaw sementara full gyro-integration (akan drift,
 *  TODO: fusion dengan HMC5883/QMC5883 dari Orang 2 setelah kalibrasi
 *  mag selesai — lihat calib_mag.c). */
/** Bawa sudut ke rentang [0,360). fmodf() sendiri masih bisa mengembalikan
 *  nilai negatif untuk input negatif, jadi hasilnya digeser sekali lagi. */
static float wrap_deg_0_360(float deg)
{
    if (!(deg == deg)) {        /* NaN: jangan sebarkan, kembalikan 0 */
        return 0.0f;
    }
    deg = fmodf(deg, 360.0f);
    if (deg < 0.0f) {
        deg += 360.0f;
    }
    return deg;
}

/** Selisih sudut terpendek SIGNED antara dua representasi sudut yang
 *  mungkin berbeda banyak putaran (mis. satu sisi mendekati 350°,
 *  sisi lain 10°) -- hasil selalu [-180,180]. Dipakai KHUSUS untuk
 *  blending yaw wrap-safe di ComputeSingleImuAttitude() (mag heading
 *  vs gyro-integrated yang belum di-wrap). SENGAJA fungsi terpisah
 *  dari AngleDiffDeg() di bawah (unsigned, dipakai deteksi disagreement
 *  dual-IMU) -- AngleDiffDeg() TIDAK diubah supaya perilaku disagreement
 *  tetap persis sama. */
static float WrapAngleSignedDeg(float deg)
{
    if (!(deg == deg)) {        /* NaN: jangan sebarkan */
        return 0.0f;
    }
    deg = fmodf(deg, 360.0f);
    if (deg > 180.0f)  deg -= 360.0f;
    if (deg < -180.0f) deg += 360.0f;
    return deg;
}

static void AccelToRollPitch(const AHRS_ImuSample_t *s, float *roll_deg, float *pitch_deg)
{
    *roll_deg  = atan2f(s->accel_y, s->accel_z) * RAD_TO_DEG_F;
    *pitch_deg = atan2f(-s->accel_x,
                         sqrtf(s->accel_y * s->accel_y + s->accel_z * s->accel_z))
                 * RAD_TO_DEG_F;
}

/** Complementary filter satu sumbu: gabung integrasi gyro (jangka
 *  pendek, bebas noise-getaran) dengan referensi accel (jangka
 *  panjang, koreksi drift). */
static float ComplementaryFilter(float angle_prev_deg,
                                  float gyro_rate_rad_s,
                                  float accel_ref_deg,
                                  float dt_s)
{
    float gyro_integrated = angle_prev_deg + (gyro_rate_rad_s * RAD_TO_DEG_F * dt_s);
    return (AHRS_COMP_FILTER_GYRO_WEIGHT * gyro_integrated)
         + ((1.0f - AHRS_COMP_FILTER_GYRO_WEIGHT) * accel_ref_deg);
}

/** Hitung satu attitude estimate dari satu sample IMU + state sebelumnya. */
static void ComputeSingleImuAttitude(const AHRS_ImuSample_t *sample,
                                      const AHRS_Attitude_t *prev,
                                      const AHRS_MagSample_t *mag,
                                      float dt_s,
                                      float *out_roll,
                                      float *out_pitch,
                                      float *out_yaw)
{
    float accel_roll_deg, accel_pitch_deg;
    AccelToRollPitch(sample, &accel_roll_deg, &accel_pitch_deg);

    *out_roll  = ComplementaryFilter(prev->roll_deg,  sample->gyro_x, accel_roll_deg,  dt_s);
    *out_pitch = ComplementaryFilter(prev->pitch_deg, sample->gyro_y, accel_pitch_deg, dt_s);

    /* Yaw: integrasi gyro dulu (belum di-wrap -- prev->yaw_deg sudah
     * [0,360) dari cycle sebelumnya, jadi nilai ini paling banter
     * sedikit di luar itu, bukan tumbuh tanpa batas). */
    float gyro_yaw_unwrapped = prev->yaw_deg + (sample->gyro_z * RAD_TO_DEG_F * dt_s);

    float yaw_result;
    if (mag != NULL && mag->valid) {
        /* mag->heading_deg searah jarum jam dari utara (kontrak
         * compass.c / AHRS_MagSample_t), sedangkan representasi yaw di
         * sini naik BERLAWANAN jarum jam kalau gyro_z positif --
         * negasi dulu supaya arah putarannya sama sebelum dibandingkan.
         *
         * Delta dihitung SEBELUM di-wrap (WrapAngleSignedDeg mencari
         * selisih terpendek terlepas dari representasi absolut kedua
         * sisi berbeda kelipatan 360) -- kalau tidak, heading 350° vs
         * yaw terintegrasi 10° akan terlihat seperti lompatan ~340°
         * padahal aslinya cuma ~20° di sekitar utara. */
        float mag_yaw_ccw_equiv = -mag->heading_deg;
        float delta = WrapAngleSignedDeg(mag_yaw_ccw_equiv - gyro_yaw_unwrapped);
        float mag_weight = 1.0f - AHRS_COMP_FILTER_GYRO_WEIGHT;
        yaw_result = gyro_yaw_unwrapped + (mag_weight * delta);
    } else {
        /* Fallback gyro-only -- regresi nol dari perilaku sebelum mag
         * fusion ada. TODO lama (fusion dengan HMC5883/QMC5883) sudah
         * ditutup lewat compass.c, sisanya di sini murni jalur fallback. */
        yaw_result = gyro_yaw_unwrapped;
    }

    /* Hasilnya di-WRAP ke [0,360). Tanpa wrap, integrasi gyro-only membuat
     * yaw_deg tumbuh tanpa batas (beberapa putaran yaw saja sudah lewat
     * 360, dan drift membuatnya terus naik walau pesawat diam). Itu
     * melanggar kontrak "heading, 0..360" yang dipegang OLED_View_t dan
     * CMD_ATTITUDE, membuat tampilan heading tidak berarti, dan
     * AngleDiffDeg() di bawah juga hanya benar untuk selisih dalam satu
     * putaran. Wrap TIDAK mengubah roll/pitch maupun jalur kontrol. */
    *out_yaw = wrap_deg_0_360(yaw_result);
}

/** Selisih sudut terpendek antara dua attitude (derajat), dipakai untuk
 *  deteksi disagreement. Sederhana — cukup untuk roll/pitch/yaw dalam
 *  rentang wajar, belum handle wrap-around 360 secara ketat untuk yaw. */
static float AngleDiffDeg(float a, float b)
{
    float diff = a - b;
    if (diff > 180.0f)  diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return fabsf(diff);
}

/* ------------------------------------------------------------------- */
/* API publik                                                           */
/* ------------------------------------------------------------------- */

void AHRS_Fusion_Init(AHRS_FusionState_t *state)
{
    memset(state, 0, sizeof(*state));
    state->attitude.active_imu = AHRS_IMU_MPU6500; /* default: primer */
    state->status_primary      = AHRS_IMU_STATUS_OK;
    state->status_secondary    = AHRS_IMU_STATUS_OK;
    state->initialized         = true;
}

void AHRS_Fusion_Update(AHRS_FusionState_t *state,
                         const AHRS_ImuSample_t *primary,
                         const AHRS_ImuSample_t *secondary,
                         const AHRS_MagSample_t *mag,
                         uint32_t tick_ms)
{
    if (!state->initialized) {
        AHRS_Fusion_Init(state);
    }

    float dt_s = AHRS_FUSION_DT_S;
    if (state->last_update_tick_ms != 0U) {
        float measured_dt = (float)(tick_ms - state->last_update_tick_ms) / 1000.0f;
        /* Guard terhadap dt aneh (loop pertama, atau scheduler jitter
         * ekstrem) — jangan biarkan integrasi gyro meledak. */
        if (measured_dt > 0.0f && measured_dt < 0.5f) {
            dt_s = measured_dt;
        }
    }
    state->last_update_tick_ms = tick_ms;

    state->status_primary   = primary->data_valid   ? AHRS_IMU_STATUS_OK : AHRS_IMU_STATUS_TIMEOUT;
    state->status_secondary = secondary->data_valid ? AHRS_IMU_STATUS_OK : AHRS_IMU_STATUS_TIMEOUT;

    float roll_p = 0, pitch_p = 0, yaw_p = 0;
    float roll_s = 0, pitch_s = 0, yaw_s = 0;
    bool have_primary   = primary->data_valid;
    bool have_secondary = secondary->data_valid;

    /* mag dipakai SAMA di kedua cabang -- satu kompas, bukan satu per IMU. */
    if (have_primary) {
        ComputeSingleImuAttitude(primary, &state->attitude, mag, dt_s, &roll_p, &pitch_p, &yaw_p);
    }
    if (have_secondary) {
        ComputeSingleImuAttitude(secondary, &state->attitude, mag, dt_s, &roll_s, &pitch_s, &yaw_s);
    }

    /* --- Arbitrasi --- */

    if (have_primary && !have_secondary) {
        /* Hanya primer hidup: pakai primer, tidak ada yang bisa dibandingkan. */
        state->attitude.active_imu        = AHRS_IMU_MPU6500;
        state->attitude.roll_deg          = roll_p;
        state->attitude.pitch_deg         = pitch_p;
        state->attitude.yaw_deg           = yaw_p;
        state->attitude.disagreement_flag = false;
        state->disagreement_count         = 0U;

    } else if (!have_primary && have_secondary) {
        /* Hanya sekunder hidup: failover otomatis. */
        state->attitude.active_imu        = AHRS_IMU_MPU6050;
        state->attitude.roll_deg          = roll_s;
        state->attitude.pitch_deg         = pitch_s;
        state->attitude.yaw_deg           = yaw_s;
        state->attitude.disagreement_flag = false;
        state->disagreement_count         = 0U;

    } else if (have_primary && have_secondary) {
        /* Keduanya hidup: cek disagreement, bukan cuma pakai primer buta-buta. */
        float diff_roll  = AngleDiffDeg(roll_p, roll_s);
        float diff_pitch = AngleDiffDeg(pitch_p, pitch_s);
        bool disagree = (diff_roll > AHRS_DISAGREEMENT_THRESHOLD_DEG) ||
                         (diff_pitch > AHRS_DISAGREEMENT_THRESHOLD_DEG);

        if (disagree) {
            state->disagreement_count++;
        } else {
            state->disagreement_count = 0U;
        }

        bool confirmed_disagreement =
            (state->disagreement_count >= AHRS_DISAGREEMENT_CONFIRM_COUNT);

        state->attitude.disagreement_flag = confirmed_disagreement;

        /* Kebijakan arbitrasi saat disagreement terkonfirmasi:
         * tetap pada IMU yang SEDANG aktif (jangan langsung lompat ke
         * yang lain tanpa tahu mana yang salah) dan naikkan flag supaya
         * command_handler.c bisa expose ke web (CMD_ATTITUDE) untuk
         * diagnosa operator. TODO: kriteria tie-break yang lebih baik
         * (mis. voting pakai gyro noise floor / self-test register)
         * bisa ditambahkan di sini kalau disagreement sering terjadi
         * di lapangan. */
        if (state->attitude.active_imu == AHRS_IMU_MPU6500) {
            state->attitude.roll_deg  = roll_p;
            state->attitude.pitch_deg = pitch_p;
            state->attitude.yaw_deg   = yaw_p;
        } else {
            state->attitude.roll_deg  = roll_s;
            state->attitude.pitch_deg = pitch_s;
            state->attitude.yaw_deg   = yaw_s;
        }

    } else {
        /* Keduanya timeout: jangan update attitude dengan data basi
         * secara diam-diam — caller wajib cek AHRS_Fusion_IsAttitudeValid()
         * sebelum pakai hasil ini untuk kontrol/failsafe. */
        state->attitude.disagreement_flag = false;
    }
}

const AHRS_Attitude_t *AHRS_Fusion_GetAttitude(const AHRS_FusionState_t *state)
{
    return &state->attitude;
}

bool AHRS_Fusion_IsAttitudeValid(const AHRS_FusionState_t *state)
{
    bool both_timeout = (state->status_primary   == AHRS_IMU_STATUS_TIMEOUT) &&
                         (state->status_secondary == AHRS_IMU_STATUS_TIMEOUT);
    return state->initialized && !both_timeout;
}
