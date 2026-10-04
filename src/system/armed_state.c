#include "armed_state.h"

/* ============================================================================
 * armed_state.c
 * Lihat armed_state.h untuk filosofi desain, kenapa modul ini menerima
 * snapshot input (bukan include langsung header modul lain), dan catatan
 * penting soal perbedaan sengaja vs logika arming lama di main.c.
 * ============================================================================ */

static ArmedState_t           s_state;
static ArmedPreflightResult_t s_last_preflight_result;

/* ---------------------------------------------------------------------------
 * Preflight check (armed_state.md Bagian 3)
 *
 * Urutan pengecekan disengaja: throttle/link/attitude dicek lebih dulu
 * karena itu kondisi yang paling sering berubah-ubah di lapangan (pilot
 * lupa turunkan throttle, radio belum nyala) -- taruh di depan supaya
 * pesan kegagalan yang paling sering dilihat juga yang paling actionable.
 * Blackbox/kalibrasi ditaruh belakang karena biasanya sudah "beres" jauh
 * sebelum pilot mencoba arm (dicek sekali di darat), bukan kondisi yang
 * berubah-ubah tiap detik. (Baterai TIDAK lagi jadi syarat gerbang ini --
 * keputusan pemilik proyek; lihat PREFLIGHT_LOW_BATTERY di armed_state.h.)
 * ------------------------------------------------------------------------- */
static ArmedPreflightResult_t run_preflight(const ArmedStateInputs_t *in)
{
    if (!in->throttle_low) {
        return PREFLIGHT_THROTTLE_NOT_LOW;
    }
    if (!in->rx_link_ok) {
        return PREFLIGHT_NO_RX_LINK;
    }
    if (!in->attitude_valid) {
        return PREFLIGHT_IMU_FAULT;
    }
    if (in->imu_disagreement) {
        return PREFLIGHT_IMU_DISAGREE;
    }
    if (!in->calibration_done) {
        return PREFLIGHT_UNCALIBRATED;
    }
    if (!in->blackbox_ready) {
        return PREFLIGHT_BLACKBOX_FAULT;
    }
    return PREFLIGHT_OK;
}

/* ---------------------------------------------------------------------------
 * API publik
 * ------------------------------------------------------------------------- */

void ArmedState_Init(void)
{
    s_state = ARMED_STATE_DISARMED;
    s_last_preflight_result = PREFLIGHT_OK; /* idle, belum pernah dicoba arm */
}

void ArmedState_Update(const ArmedStateInputs_t *inputs)
{
    if (inputs == 0) {
        return; /* caller salah pakai -- jangan sentuh state, lebih baik
                  * diam-diam tidak progress daripada baca memori acak */
    }

    if (s_state == ARMED_STATE_ARMED) {
        if (!inputs->arm_switch_engaged) {
            s_state = ARMED_STATE_DISARMED;
        }
        /* Sengaja TIDAK mengevaluasi ulang preflight di sini selama
         * masih ARMED -- lihat catatan "Perbedaan sengaja" di
         * armed_state.h. Link-loss/attitude-invalid saat terbang jadi
         * tanggung jawab alur failsafe Orang 3 (panggil
         * ArmedState_ForceDisarm() eksplisit kalau memang itu
         * keputusannya). */
        return;
    }

    /* s_state == ARMED_STATE_DISARMED (ARMED_STATE_ARMING_CHECK tidak
     * pernah persist keluar dari fungsi ini di v1 -- lihat catatan enum
     * di armed_state.h). */
    if (!inputs->arm_switch_engaged) {
        return; /* idle, arm-switch belum diminta -- jangan timpa
                  * s_last_preflight_result dengan apa pun */
    }

    s_state = ARMED_STATE_ARMING_CHECK;

    ArmedPreflightResult_t result = run_preflight(inputs);
    s_last_preflight_result = result;

    s_state = (result == PREFLIGHT_OK) ? ARMED_STATE_ARMED : ARMED_STATE_DISARMED;
}

void ArmedState_ForceDisarm(void)
{
    s_state = ARMED_STATE_DISARMED;
}

ArmedState_t ArmedState_Get(void)
{
    return s_state;
}

bool ArmedState_IsArmed(void)
{
    return s_state == ARMED_STATE_ARMED;
}

ArmedPreflightResult_t ArmedState_GetLastPreflightError(void)
{
    return s_last_preflight_result;
}

/* ---------------------------------------------------------------------------
 * Override kuat hook command_handler.h (lihat command_handler.c untuk
 * versi __attribute__((weak)) yang di-kalahkan symbol ini saat armed_state.c
 * ikut di-link).
 * ------------------------------------------------------------------------- */
int Armed_IsArmed(void)
{
    return ArmedState_IsArmed() ? 1 : 0;
}
