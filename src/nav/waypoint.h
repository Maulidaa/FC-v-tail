/**
 * @file    waypoint.h
 * @brief   Logic tracking waypoint mission — mengisi STUB yang tadinya
 *          ada di navigation.c (kasus NAV_MODE_WAYPOINT), mengikuti
 *          pola pemisahan yang sama dengan rth.c/h.
 *
 * Tanggung jawab: Orang 3 (Fusion, Navigasi & Failsafe)
 *
 * Sama seperti rth.c/h: modul ini fungsi (hampir) murni — TIDAK
 * menyimpan mission array atau index sendiri (itu tetap tinggal di
 * NAV_State_t, punya navigation.c), cuma menerima pointer ke index
 * aktif dan boleh memajukannya (in/out) saat satu item tercapai.
 * navigation.c tetap satu-satunya pemegang mode state machine
 * (NAV_MODE_WAYPOINT vs mode lain).
 *
 * Reuse RTH_CalcDistanceBearing() dari rth.h untuk distance/bearing —
 * flat-earth math yang sama dipakai RTH, tidak ditulis ulang di sini
 * (lihat catatan di rth.h soal alasan itu diekspos publik).
 *
 * Item mission & action_type mengikuti kontrak protocol.md Bagian 10
 * (CMD_MISSION_UPLOAD): action_param berarti radius (meter) untuk
 * action_type=LOITER, diabaikan untuk WAYPOINT/RTH.
 */

#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <stdint.h>
#include <stdbool.h>
#include "navigation.h"   /* reuse NAV_MissionItem_t, NAV_GpsSample_t, NAV_Setpoint_t, NAV_ActionType_t */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WAYPOINT_RESULT_NO_MISSION = 0,   /* mission_count == 0, tidak ada yang di-track */
    WAYPOINT_RESULT_NO_GPS_FALLBACK,  /* fix hilang -> level+hold, sama kebijakan dengan RTH.
                                        * TODO: apakah kehilangan fix di tengah mission harus
                                        * auto-trigger RTH alih-alih cuma hold di tempat? Belum
                                        * diputuskan tim — lihat catatan di waypoint.c. */
    WAYPOINT_RESULT_TRACKING,         /* menuju item aktif (fly-through, action_type=WAYPOINT) */
    WAYPOINT_RESULT_LOITERING,        /* sudah masuk radius item LOITER, orbit di situ */
    WAYPOINT_RESULT_ADVANCED,         /* baru saja pindah ke item berikutnya (event, 1 frame) */
    WAYPOINT_RESULT_MISSION_COMPLETE, /* item terakhir sudah tercapai */
    WAYPOINT_RESULT_ACTION_RTH,       /* item aktif action_type=RTH — caller (navigation.c)
                                        * wajib panggil Nav_TriggerRTH(), modul ini sendiri
                                        * tidak boleh mengubah NAV_State_t->mode. */
} WAYPOINT_Result_t;

typedef struct {
    NAV_Setpoint_t     setpoint;
    WAYPOINT_Result_t  result;
    uint8_t            active_index;       /* index item yang sedang di-track (sebelum advance) */
    float              distance_to_target_m;
    float              bearing_to_target_deg;
} WAYPOINT_Output_t;

/**
 * @brief  Hitung satu langkah tracking waypoint, dan majukan
 *         *current_index kalau item aktif sudah tercapai (fly-through).
 *
 * @param  mission         Array mission item (dari NAV_State_t->mission,
 *                           diisi Nav_LoadMissionItem()).
 * @param  mission_count   Jumlah item valid di array (NAV_State_t->mission_count).
 * @param  current_index   In/out — index item yang sedang di-track.
 *                           Dimajukan otomatis oleh fungsi ini saat item
 *                           fly-through (WAYPOINT) tercapai. TIDAK dimajukan
 *                           untuk LOITER (lihat catatan WAYPOINT_RESULT_LOITERING
 *                           di waypoint.c — belum ada kondisi exit loiter).
 * @param  gps             Sample GPS terbaru dari parser UBX (Orang 2).
 * @param  current_alt_m   Estimasi altitude saat ini (baro, bukan GPS altitude).
 * @param  out             Output setpoint + status (wajib non-NULL).
 */
void WAYPOINT_Compute(const NAV_MissionItem_t *mission,
                       uint8_t mission_count,
                       uint8_t *current_index,
                       const NAV_GpsSample_t *gps,
                       float current_alt_m,
                       WAYPOINT_Output_t *out);

#ifdef __cplusplus
}
#endif

#endif /* WAYPOINT_H */
