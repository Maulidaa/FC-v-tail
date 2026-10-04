/**
 * @file    stabilize.c
 * @brief   Implementasi loop stabilisasi. Lihat stabilize.h untuk
 *          dokumentasi API.
 */

#include "stabilize.h"
#include <stddef.h>

void Stabilize_Init(StabilizeContext_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    /* Gain 0,0,0: sengaja tidak "menebak" nilai default yang kelihatan
     * masuk akal — PID dengan gain nol tidak menghasilkan output apa pun
     * dari error, jadi kalau tuning belum di-load dari Settings storage,
     * perilakunya jelas "diam", bukan bergerak dengan gain acak yang
     * berisiko. PID_Init() sendiri juga sudah mereset seluruh state
     * internal (integral dkk). */
    PID_Init(&ctx->roll_pid, 0.0f, 0.0f, 0.0f);
    PID_Init(&ctx->pitch_pid, 0.0f, 0.0f, 0.0f);

    /* Output limit PID default dari PID_Init() sudah [-1,1], sama persis
     * dengan rentang input roll/pitch yang diharapkan MixerInput_t — tidak
     * perlu PID_SetOutputLimits() tambahan di sini. */

    Mixer_ConfigDefault(&ctx->mixer_config);
    OutputMap_InitDefault(&ctx->output_map);

    ctx->initialized = true;
}

void Stabilize_SetRollGains(StabilizeContext_t *ctx, float kp, float ki, float kd)
{
    if (ctx == NULL) {
        return;
    }
    PID_SetGains(&ctx->roll_pid, kp, ki, kd);
}

void Stabilize_SetPitchGains(StabilizeContext_t *ctx, float kp, float ki, float kd)
{
    if (ctx == NULL) {
        return;
    }
    PID_SetGains(&ctx->pitch_pid, kp, ki, kd);
}

void Stabilize_OnArmedTransition(StabilizeContext_t *ctx, bool now_armed)
{
    if (ctx == NULL) {
        return;
    }

    if (now_armed) {
        /* Disarmed -> armed: buang integral lama supaya tidak ada lonjakan
         * output begitu motor mulai berputar. Lihat dokumentasi
         * PID_Reset() di pid.h. */
        PID_Reset(&ctx->roll_pid);
        PID_Reset(&ctx->pitch_pid);
    }
    /* Armed -> disarmed: TIDAK perlu reset di sini — motor sudah dipaksa
     * idle oleh armed-state guard di output_map.c terlepas dari nilai PID,
     * dan servo tetap boleh bergerak saat disarmed (lihat catatan guard di
     * output_map.h), jadi integral yang masih "hidup" sesaat setelah
     * disarm tidak berbahaya. */
}

void Stabilize_ResetIntegrators(StabilizeContext_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    /* Sama persis dengan cabang now_armed==true di
     * Stabilize_OnArmedTransition() di atas — lihat dokumentasi di sana
     * dan di PID_Reset() (pid.h) untuk alasan detail kenapa integral lama
     * perlu dibuang sebelum PID mulai mengoreksi error lagi. */
    PID_Reset(&ctx->roll_pid);
    PID_Reset(&ctx->pitch_pid);
}

void Stabilize_Update(StabilizeContext_t *ctx, const StabilizeInput_t *input)
{
    if (ctx == NULL || input == NULL || !ctx->initialized) {
        return;
    }

    PID_Result_t roll_result =
        PID_Compute(&ctx->roll_pid, input->roll_setpoint_deg, input->roll_measured_deg,
                    input->dt_seconds);
    PID_Result_t pitch_result =
        PID_Compute(&ctx->pitch_pid, input->pitch_setpoint_deg, input->pitch_measured_deg,
                    input->dt_seconds);

    MixerInput_t mixer_input = {
        .roll = roll_result.output,
        .pitch = pitch_result.output,
        .yaw = input->yaw_command,
        .throttle = input->throttle_command,
    };

    MixerOutput_t mixer_output = Mixer_Compute(&ctx->mixer_config, &mixer_input);

    OutputMap_WriteFromMixer(&ctx->output_map, &mixer_output, input->armed);
}
