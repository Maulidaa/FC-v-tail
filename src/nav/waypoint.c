/**
 * @file    waypoint.c
 * @brief   Implementasi tracking waypoint mission.
 *
 * Status: SKELETON — mengisi TODO yang tadinya tertulis di navigation.c
 * (kasus NAV_MODE_WAYPOINT):
 *   1. Ambil mission[*current_index]                          -> DONE
 *   2. Hitung distance/bearing ke item itu                    -> DONE
 *      (reuse RTH_CalcDistanceBearing(), lihat rth.h)
 *   3. Advance ke item berikutnya kalau distance < radius      -> DONE
 *      (untuk action_type=WAYPOINT; selesai kalau item terakhir)
 *   4. Handle LOITER terpisah dari WAYPOINT biasa              -> SEBAGIAN
 *      (deteksi masuk radius loiter sudah ada, tapi belum ada
 *      logic ORBIT sungguhan maupun kondisi exit loiter — lihat
 *      catatan di WAYPOINT_RESULT_LOITERING di bawah)
 *
 * Yang BELUM diimplementasikan (jangan dianggap sudah jalan):
 *   - Orbit control sungguhan saat LOITERING (sekarang cuma "arahkan
 *     yaw ke titik loiter dan tahan altitude" — bukan orbit beneran,
 *     itu kemungkinan besar tanggung jawab Orang 4 di level PID/mixer
 *     begitu diberi target+radius, atau butuh logic tambahan di sini).
 *   - Kondisi exit loiter (protocol.md tidak mendefinisikan durasi
 *     loiter) — sekali masuk LOITERING, modul ini tidak akan pernah
 *     memajukan index sendiri. Exit harus lewat mekanisme lain (mis.
 *     command manual dari web, atau field durasi yang belum ada di
 *     skema CMD_MISSION_UPLOAD) — TODO tim, dicatat juga di protocol.md
 *     kalau nanti mau ditambahkan.
 *   - Kehilangan GPS fix di tengah mission: sekarang cuma level+hold
 *     di tempat (WAYPOINT_RESULT_NO_GPS_FALLBACK), TIDAK auto-trigger
 *     RTH. Ini konsisten dengan kebijakan RTH sendiri (tidak menebak
 *     arah tanpa posisi), tapi apakah kehilangan fix saat mission
 *     seharusnya justru auto-RTH begitu fix balik adalah keputusan tim
 *     yang belum diambil.
 */

#include "waypoint.h"
#include "rth.h"     /* reuse RTH_CalcDistanceBearing() */
#include <string.h>

/* ------------------------------------------------------------------- */
/* Helper statis                                                        */
/* ------------------------------------------------------------------- */

static bool GpsHasUsableFix(const NAV_GpsSample_t *gps)
{
    return gps->data_valid &&
           (gps->fix_type == NAV_GPS_FIX_2D || gps->fix_type == NAV_GPS_FIX_3D);
}

/* ------------------------------------------------------------------- */
/* API publik                                                           */
/* ------------------------------------------------------------------- */

void WAYPOINT_Compute(const NAV_MissionItem_t *mission,
                       uint8_t mission_count,
                       uint8_t *current_index,
                       const NAV_GpsSample_t *gps,
                       float current_alt_m,
                       WAYPOINT_Output_t *out)
{
    memset(out, 0, sizeof(*out));

    if (mission_count == 0U) {
        out->result = WAYPOINT_RESULT_NO_MISSION;
        return;
    }

    if (*current_index >= mission_count) {
        /* Sudah lewat item terakhir dari panggilan sebelumnya —
         * caller (navigation.c) seharusnya sudah pindah mode saat
         * MISSION_COMPLETE pertama kali dilaporkan, ini jaga-jaga. */
        out->result = WAYPOINT_RESULT_MISSION_COMPLETE;
        return;
    }

    if (!GpsHasUsableFix(gps)) {
        /* Sama kebijakan dengan RTH: tanpa fix, jangan menebak arah —
         * cuma stabilkan dan tahan altitude saat ini. Index TIDAK
         * dimajukan/direset, mission tetap di posisi yang sama begitu
         * fix kembali. */
        out->result                     = WAYPOINT_RESULT_NO_GPS_FALLBACK;
        out->active_index               = *current_index;
        out->setpoint.target_roll_deg   = 0.0f;
        out->setpoint.target_pitch_deg  = 0.0f;
        out->setpoint.target_altitude_m = current_alt_m;
        return;
    }

    const NAV_MissionItem_t *item = &mission[*current_index];
    out->active_index = *current_index;

    if (item->action_type == NAV_ACTION_RTH) {
        /* Item eksplisit minta RTH. Modul ini TIDAK boleh mengubah
         * NAV_State_t->mode langsung (itu hak navigation.c) — cuma
         * lapor status, caller yang panggil Nav_TriggerRTH(). */
        out->result = WAYPOINT_RESULT_ACTION_RTH;
        return;
    }

    float distance_m, bearing_deg;
    RTH_CalcDistanceBearing(gps->lat_e7, gps->lon_e7,
                             item->lat_e7, item->lon_e7,
                             &distance_m, &bearing_deg);
    out->distance_to_target_m   = distance_m;
    out->bearing_to_target_deg  = bearing_deg;

    bool is_loiter = (item->action_type == NAV_ACTION_LOITER);
    /* Radius acceptance: untuk LOITER pakai action_param (meter, sesuai
     * protocol.md Bagian 10); untuk WAYPOINT biasa pakai konstanta
     * NAV_WAYPOINT_ACCEPT_RADIUS_M yang sama dipakai RTH. */
    float accept_radius_m = is_loiter ? (float)item->action_param
                                       : NAV_WAYPOINT_ACCEPT_RADIUS_M;

    if (distance_m <= accept_radius_m) {
        if (is_loiter) {
            /* Masuk radius loiter. Belum ada orbit control sungguhan
             * atau kondisi exit — lihat catatan panjang di header file
             * ini. Untuk skeleton: arahkan yaw ke titik pusat loiter
             * dan tahan altitude item, index TIDAK dimajukan. */
            out->result                     = WAYPOINT_RESULT_LOITERING;
            out->setpoint.target_yaw_deg    = bearing_deg;
            out->setpoint.target_altitude_m = (float)item->altitude_m;
            out->setpoint.target_roll_deg   = 0.0f;
            out->setpoint.target_pitch_deg  = 0.0f;
            return;
        }

        /* WAYPOINT fly-through tercapai — advance. */
        (*current_index)++;
        if (*current_index >= mission_count) {
            out->result = WAYPOINT_RESULT_MISSION_COMPLETE;
        } else {
            out->result = WAYPOINT_RESULT_ADVANCED;
        }
        /* Setpoint transisi untuk frame ini: tahan altitude di item
         * yang baru tercapai, roll/pitch level. Frame berikutnya
         * (dipanggil lagi oleh navigation.c) akan mulai tracking ke
         * item baru (atau ALTITUDE_HOLD kalau MISSION_COMPLETE). */
        out->setpoint.target_altitude_m = (float)item->altitude_m;
        out->setpoint.target_roll_deg   = 0.0f;
        out->setpoint.target_pitch_deg  = 0.0f;
        return;
    }

    /* Masih menuju item aktif. */
    out->result                     = WAYPOINT_RESULT_TRACKING;
    out->setpoint.target_yaw_deg    = bearing_deg;
    out->setpoint.target_altitude_m = (float)item->altitude_m;
    out->setpoint.target_roll_deg   = 0.0f; /* koordinasi turn: Orang 4/PID yang urus roll-to-turn */
    out->setpoint.target_pitch_deg  = 0.0f;
}
