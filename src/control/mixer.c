/**
 * @file    mixer.c
 * @brief   Implementasi mixer V-tail. Lihat mixer.h untuk dokumentasi API.
 */

#include "mixer.h"
#include <math.h>
#include <stddef.h>   /* NULL -- lihat catatan yang sama di pid.c */

#define MIXER_SERVO_MIN   (-1.0f)
#define MIXER_SERVO_MAX   (1.0f)
#define MIXER_MOTOR_MIN   (0.0f)
#define MIXER_MOTOR_MAX   (1.0f)

#define MIXER_DIFFERENTIAL_MIN (0.0f)
#define MIXER_DIFFERENTIAL_MAX (1.0f)

/* Arah servo: +1.0f = normal, -1.0f = dibalik */
#define MIXER_DIR_AIL_L   (-1.0f)   /* dibalik -> searah aileron kanan */
#define MIXER_DIR_AIL_R   (+1.0f)

/* Arah fisik servo ruddervator (V-tail). Servo kiri & kanan biasanya
 * terpasang saling berhadapan (mirror), jadi perintah +1 di kiri
 * menggerakkan permukaan ke arah yang BERLAWANAN dengan +1 di kanan.
 * Tanpa pembalikan satu sisi, pitch malah jadi gerak berlawanan
 * (kelihatan seperti yaw) dan yaw jadi gerak bareng (kelihatan seperti
 * pitch) -- gejala "pitch & yaw tertukar". Membalik pitch/yaw secara
 * global TIDAK memperbaiki itu. Kalau setelah tes bench pitch/yaw masih
 * tertukar, pindahkan -1.0f ke sisi KIRI (dan +1.0f ke kanan). */
#define MIXER_DIR_VTAIL_L (+1.0f)
#define MIXER_DIR_VTAIL_R (-1.0f)

/* Arah sumbu pitch/yaw setelah mixing. Kalau pitch atau yaw sudah
 * "searah" (bukan tertukar) tapi arahnya salah, balik salah satunya di
 * sini. Default +1 = rumus asli di mixer.h. */
#define MIXER_DIR_PITCH   (+1.0f)
#define MIXER_DIR_YAW     (+1.0f)

static inline float mixer_clampf(float value, float min, float max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

void Mixer_ConfigDefault(MixerConfig_t *config)
{
    if (config == NULL) {
        return;
    }
    config->aileron_differential = 0.0f;
    config->ruddervator_gain = 1.0f;
}

void Mixer_SetAileronDifferential(MixerConfig_t *config, float differential)
{
    if (config == NULL || isnan(differential) || isinf(differential)) {
        return;
    }
    config->aileron_differential =
        mixer_clampf(differential, MIXER_DIFFERENTIAL_MIN, MIXER_DIFFERENTIAL_MAX);
}

void Mixer_SetRuddervatorGain(MixerConfig_t *config, float gain)
{
    if (config == NULL || isnan(gain) || isinf(gain)) {
        return;
    }
    config->ruddervator_gain = gain;
}

/**
 * @brief Terapkan aileron differential ke satu sisi servo.
 *
 * Konvensi: "defleksi naik" = nilai command searah command roll di sisi ini
 * (mis. aileron kanan naik saat roll kanan), "defleksi turun" = arah
 * berlawanan. Differential mengurangi besar defleksi turun sebesar fraksi
 * `differential`, tanpa mengubah defleksi naik — tujuan standarnya meredam
 * adverse yaw (drag berlebih dari aileron yang turun).
 *
 * @param command      Command mentah untuk sisi ini, sebelum differential
 *                      (mis. +roll untuk sisi kanan, -roll untuk sisi kiri).
 * @param differential [0.0, 1.0].
 */
static float mixer_apply_aileron_differential(float command, float differential)
{
    if (command < 0.0f) {
        /* Defleksi ke arah "turun" untuk sisi ini -> kurangi besarnya. */
        return command * (1.0f - differential);
    }
    /* Defleksi "naik" atau netral -> tidak diubah. */
    return command;
}

MixerOutput_t Mixer_Compute(const MixerConfig_t *config, const MixerInput_t *input)
{
    MixerOutput_t out = {0};

    if (config == NULL || input == NULL) {
        /* Fail-safe: semua channel netral/idle kalau input tidak valid,
         * bukan nilai acak dari stack. */
        out.channels[MIXER_CH_MOTOR0] = MIXER_MOTOR_MIN;
        return out;
    }

    float roll  = mixer_clampf(input->roll,  -1.0f, 1.0f);
    float pitch = mixer_clampf(input->pitch, -1.0f, 1.0f) * MIXER_DIR_PITCH;
    float yaw   = mixer_clampf(input->yaw,   -1.0f, 1.0f) * MIXER_DIR_YAW;
    float throttle = mixer_clampf(input->throttle, MIXER_MOTOR_MIN, MIXER_MOTOR_MAX);

    /* --- Motor: langsung dari throttle, tanpa mixing tambahan
     *     (firmware-architecture-stm32f411.md 3.7, poin "1 channel motor"). --- */
    out.channels[MIXER_CH_MOTOR0] = throttle;

    /* --- Aileron kiri/kanan: dari roll, dengan differential opsional
     *     (firmware-architecture-stm32f411.md 3.7, poin "2 channel aileron"). ---
     * Konvensi tanda: roll positif = roll ke kanan -> aileron kanan naik
     * (+), aileron kiri turun (-). Kalau konvensi arah servo fisik di
     * board kalian terbalik, balik tanda di sini saja — jangan di RX
     * parser atau PID, supaya konvensi sign tetap konsisten di modul lain. */
    /* ail_r_raw/ail_l_raw sebelumnya dihitung di sini lalu tidak pernah
     * dipakai (dua baris mati yang memicu -Wunused-variable). Nilai yang
     * benar-benar dipakai adalah hasil mixer_apply_aileron_differential()
     * di bawah, jadi baris mentahnya dihapus supaya tidak ada yang mengira
     * ada dua jalur perhitungan aileron di sini. */
    float ail_r = mixer_apply_aileron_differential(roll,  config->aileron_differential);
    float ail_l = mixer_apply_aileron_differential(-roll, config->aileron_differential);

    out.channels[MIXER_CH_AIL_R] = mixer_clampf(ail_r * MIXER_DIR_AIL_R, MIXER_SERVO_MIN, MIXER_SERVO_MAX);
    out.channels[MIXER_CH_AIL_L] = mixer_clampf(ail_l * MIXER_DIR_AIL_L, MIXER_SERVO_MIN, MIXER_SERVO_MAX);

    /* --- Ruddervator kiri/kanan: mixing pitch (elevator) + yaw (rudder)
     *     (firmware-architecture-stm32f411.md 3.7, poin "2 channel ruddervator").
     * Formula: kiri = pitch + gain*yaw, kanan = pitch - gain*yaw
     * (tanda mengikuti konvensi arah servo yang dipasang — sama seperti
     * aileron, kalau terbalik di board kalian, balik di sini saja). --- */
    float vtail_l = pitch + (config->ruddervator_gain * yaw);
    float vtail_r = pitch - (config->ruddervator_gain * yaw);

    out.channels[MIXER_CH_VTAIL_L] = mixer_clampf(vtail_l * MIXER_DIR_VTAIL_L, MIXER_SERVO_MIN, MIXER_SERVO_MAX);
    out.channels[MIXER_CH_VTAIL_R] = mixer_clampf(vtail_r * MIXER_DIR_VTAIL_R, MIXER_SERVO_MIN, MIXER_SERVO_MAX);

    return out;
}