/**
 * @file    stabilize.h
 * @brief   Loop stabilisasi attitude fixed-wing — menyambungkan pid.c
 *          (roll & pitch loop) -> mixer.c -> output_map.c dalam satu
 *          fungsi update per siklus scheduler. Lihat
 *          firmware-architecture-stm32f411.md Bagian 3.7 dan
 *          pembagian-tugas-firmware-4-orang.md (tugas Orang 4, item
 *          "PID generik + mixer + stabilize").
 *
 * Cakupan v1: mode self-level sederhana untuk roll & pitch (setpoint =
 * target sudut, measurement = sudut terukur dari AHRS). Yaw TIDAK
 * di-PID-kan di v1 — nilai yaw command diteruskan langsung dari RX/nav ke
 * mixer (khas airframe fixed-wing sederhana: rudder/ruddervator dikendalikan
 * langsung, bukan lewat rate/angle controller terpisah). Kalau nanti
 * dibutuhkan coordinated-turn atau yaw damper, tambahkan PID_t ketiga di
 * StabilizeContext_t tanpa mengubah struktur modul lain.
 *
 * === Dependency ===
 * - Measurement roll/pitch (derajat) berasal dari `fusion/ahrs.c` (Orang 3)
 *   — modul ini TIDAK mem-parsing sensor mentah, hanya menerima sudut
 *   yang sudah di-estimasi lewat StabilizeInput_t.
 * - Setpoint roll/pitch (derajat) berasal dari RX stick (mode self-level)
 *   atau dari nav (mis. altitude_hold.c untuk pitch, RTH untuk roll/pitch)
 *   — juga diterima lewat StabilizeInput_t, modul ini tidak tahu sumbernya.
 */

#ifndef STABILIZE_H
#define STABILIZE_H

#include <stdbool.h>
#include <stdint.h>

#include "pid.h"
#include "mixer.h"
#include "output_map.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief State lengkap loop stabilisasi: PID roll & pitch, config mixer,
 *        dan output map. Satu instance mewakili satu "jalur kontrol"
 *        lengkap dari attitude sampai sinyal aktuator fisik.
 *
 * Field diekspos (bukan opaque pointer) supaya modul lain (mis. command
 * handler untuk SETTING_GET/SET grup mixer/PID, atau blackbox untuk log
 * gain aktif) bisa membaca/menulis pid_roll.kp dst secara langsung lewat
 * PID_SetGains()/Mixer_SetAileronDifferential() dkk — bukan lewat field
 * mentah, supaya tetap lewat validasi di setter masing-masing modul.
 */
typedef struct {
    PID_t roll_pid;
    PID_t pitch_pid;
    MixerConfig_t mixer_config;
    OutputMap_t output_map;
    bool initialized;
} StabilizeContext_t;

/**
 * @brief Input satu siklus stabilize. Semua sudut dalam derajat, semua
 *        command lain dalam konvensi normalisasi yang sama dengan
 *        mixer.h (roll/pitch/yaw [-1,1] untuk yaw, throttle [0,1]).
 */
typedef struct {
    float roll_setpoint_deg;
    float pitch_setpoint_deg;
    float roll_measured_deg;
    float pitch_measured_deg;

    /* Diteruskan langsung ke mixer, TIDAK di-PID-kan di v1 (lihat catatan
     * di atas file ini). */
    float yaw_command;      /* [-1.0, 1.0] */
    float throttle_command; /* [0.0, 1.0] */

    bool armed;
    float dt_seconds;
} StabilizeInput_t;

/**
 * @brief Inisialisasi context: PID roll/pitch dengan gain default aman
 *        (0,0,0 — TIDAK akan menggerakkan apa pun sampai
 *        Stabilize_SetRollGains()/SetPitchGains() dipanggil dengan nilai
 *        hasil tuning, mis. dari Settings storage), mixer config default,
 *        dan output map default V-tail.
 *
 * WAJIB dipanggil sekali saat boot, setelah OutputMap tidak perlu
 * konfigurasi tambahan (v1 pakai default V-tail langsung).
 */
void Stabilize_Init(StabilizeContext_t *ctx);

/**
 * @brief Set gain PID roll. Wrapper tipis ke PID_SetGains() supaya
 *        pemanggil (command handler SETTING_SET) tidak perlu tahu field
 *        internal StabilizeContext_t.
 */
void Stabilize_SetRollGains(StabilizeContext_t *ctx, float kp, float ki, float kd);

/**
 * @brief Set gain PID pitch.
 */
void Stabilize_SetPitchGains(StabilizeContext_t *ctx, float kp, float ki, float kd);

/**
 * @brief WAJIB dipanggil pada transisi disarmed -> armed. Mereset kedua
 *        PID (roll & pitch) supaya integral term yang menumpuk saat
 *        disarmed/di darat tidak menyebabkan lonjakan output begitu motor
 *        mulai berputar — lihat PID_Reset() di pid.h untuk alasan detail.
 */
void Stabilize_OnArmedTransition(StabilizeContext_t *ctx, bool now_armed);

/**
 * @brief Reset integrator kedua PID (roll & pitch) tanpa mengubah
 *        gain/limits maupun state lain di ctx — pola persis sama dengan
 *        cabang `now_armed == true` di Stabilize_OnArmedTransition() di
 *        atas (sama-sama PID_Reset() ke roll_pid & pitch_pid), tapi
 *        dipisah jadi fungsi publik sendiri supaya pemanggilannya tidak
 *        terikat ke transisi armed/disarmed.
 *
 * WAJIB dipanggil tepat saat transisi KEMBALI dari mode yang melewati PID
 * (mis. full manual/passthrough di main.c) ke stabilize aktif — supaya
 * integral term yang sempat diam/basi selama PID tidak jalan tidak
 * menyebabkan lonjakan output begitu PID mulai lagi mengoreksi error.
 * Tidak perlu dipanggil untuk arah sebaliknya (masuk ke mode yang
 * melewati PID), karena PID_Compute() memang tidak dipanggil sama sekali
 * selama itu — tidak ada state yang perlu dibersihkan sebelum "keluar".
 */
void Stabilize_ResetIntegrators(StabilizeContext_t *ctx);

/**
 * @brief Jalankan satu siklus penuh: PID roll & pitch -> Mixer_Compute()
 *        -> OutputMap_WriteFromMixer(). Dipanggil scheduler pada rate
 *        loop stabilisasi (biasanya rate tertinggi setelah IMU/AHRS).
 */
void Stabilize_Update(StabilizeContext_t *ctx, const StabilizeInput_t *input);

#ifdef __cplusplus
}
#endif

#endif /* STABILIZE_H */
