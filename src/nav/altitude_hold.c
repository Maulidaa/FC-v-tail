/**
 * @file    altitude_hold.c
 * @brief   Implementasi capture & hold target altitude.
 *
 * Status: SELESAI untuk skeleton ini — logikanya memang sesederhana
 * kelihatannya (simpan satu angka, kembalikan lagi), nilai modul ini
 * ada di KAPAN angka itu boleh berubah (cuma di ALT_HOLD_Enter(),
 * tidak pernah diam-diam di tempat lain), bukan di kompleksitas
 * hitungannya. Lihat altitude_hold.h untuk alasan lengkap kenapa ini
 * dipisah dari navigation.c.
 *
 * TIDAK melakukan kontrol PID climb-rate atau apa pun — itu tetap
 * tanggung jawab Orang 4 (control/pid.c, control/stabilize.c) yang
 * membandingkan target ini dengan altitude aktual dan menghasilkan
 * output throttle/elevator.
 */

#include "altitude_hold.h"
#include <string.h>

void ALT_HOLD_Init(ALT_HOLD_State_t *s)
{
    memset(s, 0, sizeof(*s));
    s->is_captured = false;
}

void ALT_HOLD_Enter(ALT_HOLD_State_t *s, float current_alt_m)
{
    s->target_altitude_m = current_alt_m;
    s->is_captured        = true;
}

float ALT_HOLD_GetTarget(const ALT_HOLD_State_t *s)
{
    return s->target_altitude_m;
}
