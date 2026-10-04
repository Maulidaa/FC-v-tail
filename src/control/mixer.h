/**
 * @file    mixer.h
 * @brief   Mixer untuk airframe default v1: fixed-wing V-tail
 *          (1 motor + 2 aileron + 2 ruddervator). Lihat
 *          firmware-architecture-stm32f411.md Bagian 3.7 dan
 *          pinout-fc-stm32f411.md Bagian 3 (channel logis).
 *
 * Alur data: pid.c (roll/pitch loop) + RX (yaw, throttle) -> Mixer_Compute()
 * -> nilai per channel logis, dinormalisasi -> output_map.c/h -> dshot.c
 * atau pwm_servo.c (konversi ke unit protokol aktuator).
 *
 * Konvensi normalisasi:
 * - Channel servo (aileron, ruddervator): float [-1.0, +1.0], 0.0 = netral.
 * - Channel motor: float [0.0, 1.0], 0.0 = idle/stop, 1.0 = full throttle.
 *   (Bukan [-1,1] karena motor brushless v1 tidak reversible.)
 *
 * Parameter tunable (aileron_differential, ruddervator_gain) sengaja
 * dipisah dari struct input/output supaya bisa diekspos langsung ke skema
 * SETTING_GET/SETTING_SET (protocol.md) tanpa mixer.c perlu tahu apa-apa
 * soal framing protokol — lihat keputusan desain di
 * pembagian-tugas-firmware-4-orang.md Bagian 2, item #1: "grup setting
 * mixer adalah tunable setting, bukan konstanta tetap".
 */

#ifndef MIXER_H
#define MIXER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Channel logis output, urutannya mengikuti tabel channel V-tail di
 *        pinout-fc-stm32f411.md Bagian 3. Dipakai sebagai index array
 *        MixerOutput_t.channels[] dan juga selaras dengan index logis yang
 *        dipakai output_map.c/h (role MOTOR/SERVO + index).
 */
typedef enum {
    MIXER_CH_MOTOR0 = 0, /* PA8  / TIM1_CH1 — DShot ke ESC */
    MIXER_CH_AIL_L,      /* PB0  / TIM3_CH3 — aileron kiri */
    MIXER_CH_AIL_R,      /* PB1  / TIM3_CH4 — aileron kanan */
    MIXER_CH_VTAIL_L,    /* PB8  / TIM4_CH3 — ruddervator kiri */
    MIXER_CH_VTAIL_R,    /* PB9  / TIM4_CH4 — ruddervator kanan */
    MIXER_CH_COUNT
} MixerChannel_t;

/**
 * @brief Input mixer: command hasil PID (roll/pitch) + command dari RX/nav
 *        (yaw, throttle). Semua sudah dinormalisasi sebelum masuk ke sini —
 *        mixer tidak tahu soal PID gain atau parsing RX.
 *
 * roll, pitch, yaw: float [-1.0, +1.0].
 * throttle: float [0.0, 1.0] — bisa dari RX manual atau logic auto-throttle
 *           navigasi (altitude hold/RTH), lihat Bagian 3.6 & 3.7.
 */
typedef struct {
    float roll;
    float pitch;
    float yaw;
    float throttle;
} MixerInput_t;

/**
 * @brief Parameter tunable mixer. Nilai default aman (differential=0,
 *        ruddervator_gain=1) dipakai sebelum tim menentukan nilai final
 *        pasca uji terbang — lihat item terbuka di
 *        firmware-architecture-stm32f411.md Bagian 5.
 */
typedef struct {
    /**
     * Aileron differential: fraksi pengurangan defleksi pada arah "turun"
     * untuk meredam adverse yaw. Rentang [0.0, 1.0]:
     *   0.0 = tidak ada differential (defleksi naik/turun sama besar)
     *   1.0 = defleksi turun ditiadakan total (ekstrem, jarang dipakai)
     */
    float aileron_differential;

    /**
     * Gain mixing ruddervator untuk kontribusi yaw ke servo ekor kiri/kanan,
     * formula standar: kiri = pitch + gain*yaw, kanan = pitch - gain*yaw
     * (lihat firmware-architecture-stm32f411.md 3.7). Nilai wajar sekitar
     * [0.0, 1.0], tapi tidak di-clamp keras di sini karena unit kalibrasi
     * bisa saja butuh nilai di luar itu tergantung throw servo fisik —
     * clamp akhir tetap terjadi di tahap normalisasi output per channel.
     */
    float ruddervator_gain;
} MixerConfig_t;

/**
 * @brief Output mixer: satu nilai per channel logis, sudah dinormalisasi
 *        dan di-clamp ke rentang channel masing-masing (lihat konvensi di
 *        atas). Index array mengikuti MixerChannel_t.
 */
typedef struct {
    float channels[MIXER_CH_COUNT];
} MixerOutput_t;

/**
 * @brief Isi MixerConfig_t dengan nilai default aman (differential=0.0,
 *        ruddervator_gain=1.0 — mixing linear tanpa atenuasi/penguatan).
 *        Dipanggil saat boot sebelum nilai dari Settings storage dimuat,
 *        atau kalau nilai di flash ternyata korup/incompatible (lihat
 *        firmware-architecture-stm32f411.md 3.12 soal validasi setting).
 */
void Mixer_ConfigDefault(MixerConfig_t *config);

/**
 * @brief Set aileron differential, sudah di-clamp ke [0.0, 1.0].
 *        Dipanggil dari handler SETTING_SET grup "mixer" begitu skema
 *        protokolnya siap di sisi Orang 1.
 */
void Mixer_SetAileronDifferential(MixerConfig_t *config, float differential);

/**
 * @brief Set ruddervator gain. Tidak di-clamp keras di sini (lihat
 *        catatan di MixerConfig_t), tapi NaN/Inf ditolak (diabaikan,
 *        config tidak berubah) supaya input korup dari protokol tidak
 *        meracuni mixer.
 */
void Mixer_SetRuddervatorGain(MixerConfig_t *config, float gain);

/**
 * @brief Hitung nilai tiap channel output dari input roll/pitch/yaw/throttle
 *        + parameter mixer saat ini. Fungsi murni (tidak ada state
 *        internal/side effect) — aman dipanggil tiap siklus scheduler
 *        tanpa perlu init/reset terpisah seperti pid.c.
 *
 * @param config Parameter mixer saat ini (read-only, tidak diubah fungsi ini).
 * @param input  Command roll/pitch/yaw/throttle, sudah dinormalisasi.
 * @return       Nilai tiap channel logis, sudah di-clamp ke rentang aman.
 */
MixerOutput_t Mixer_Compute(const MixerConfig_t *config, const MixerInput_t *input);

#ifdef __cplusplus
}
#endif

#endif /* MIXER_H */
