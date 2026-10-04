/**
 * @file    navigation.c
 * @brief   Implementasi navigasi: RTH (+ fallback tanpa GPS), altitude
 *          hold, waypoint tracking.
 *
 * Status: SKELETON.
 *   - Altitude hold: capture & hold target DIEKSTRAK ke altitude_hold.c/h
 *     (lihat file itu untuk kenapa — ada celah "capture on entry" yang
 *     diperbaiki di sana). File ini cuma panggil ALT_HOLD_GetTarget()
 *     tiap frame dan Nav_EnterAltitudeHold() saat transisi masuk mode.
 *   - RTH: perhitungan bearing/distance + logic navigating/arrived/
 *     fallback DIEKSTRAK ke rth.c/h (lihat rth.h untuk alasannya).
 *     File ini cuma pegang mode state machine (kapan masuk/keluar
 *     NAV_MODE_RTH vs NAV_MODE_RTH_NO_GPS_FALLBACK) dan menyalin hasil
 *     RTH_Compute() ke state->setpoint.
 *   - Waypoint: perhitungan tracking (distance/bearing, advance index,
 *     handle LOITER) DIEKSTRAK ke waypoint.c/h, pola sama dengan RTH.
 *     File ini cuma pegang mode state machine untuk NAV_MODE_WAYPOINT.
 */

#include "navigation.h"
#include "rth.h"
#include "waypoint.h"
#include "altitude_hold.h"
#include "command_handler.h"
#include "error.h"
#include <math.h>
#include <string.h>

static NAV_State_t *s_protocol_nav;

static int32_t read_i32_le(const uint8_t *data)
{
    return (int32_t)((uint32_t)data[0] |
                     ((uint32_t)data[1] << 8) |
                     ((uint32_t)data[2] << 16) |
                     ((uint32_t)data[3] << 24));
}

static int16_t read_i16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static void send_nav_ack(const protocol_frame_t *frame, uint16_t command_id,
                         uint8_t status, uint8_t count)
{
    uint8_t payload[2] = { status, count };
    Protocol_SendFrame(command_id, frame->request_id, payload, sizeof(payload));
}

static void handle_mission_upload(const protocol_frame_t *frame)
{
    if (frame->payload_len != 16U) {
        Error_Send(frame->request_id, ERROR_INVALID_PAYLOAD_LENGTH,
                   CMD_MISSION_UPLOAD);
        return;
    }
    if (Armed_IsArmed()) {
        Error_Send(frame->request_id, ERROR_ARMED_REJECTED,
                   CMD_MISSION_UPLOAD);
        return;
    }

    uint8_t seq = frame->payload[0];
    uint8_t expected_seq = s_protocol_nav->mission_count;
    if (seq == 0U) {
        Nav_ClearMission(s_protocol_nav);
    } else if (seq != expected_seq) {
        Error_Send(frame->request_id, ERROR_INVALID_PAYLOAD_LENGTH,
                   CMD_MISSION_UPLOAD);
        return;
    }

    uint8_t action = frame->payload[11];
    if (action > (uint8_t)NAV_ACTION_RTH) {
        Error_Send(frame->request_id, ERROR_INVALID_PAYLOAD_LENGTH,
                   CMD_MISSION_UPLOAD);
        return;
    }

    NAV_MissionItem_t item;
    item.seq = seq;
    item.lat_e7 = read_i32_le(&frame->payload[1]);
    item.lon_e7 = read_i32_le(&frame->payload[5]);
    item.altitude_m = read_i16_le(&frame->payload[9]);
    item.action_type = (NAV_ActionType_t)action;
    item.action_param = read_i32_le(&frame->payload[12]);

    if (!Nav_LoadMissionItem(s_protocol_nav, &item)) {
        Error_Send(frame->request_id, ERROR_BUSY, CMD_MISSION_UPLOAD);
        return;
    }
    send_nav_ack(frame, CMD_MISSION_UPLOAD, 0U,
                 s_protocol_nav->mission_count);
}

static void handle_home_set(const protocol_frame_t *frame)
{
    if (frame->payload_len != 12U) {
        Error_Send(frame->request_id, ERROR_INVALID_PAYLOAD_LENGTH,
                   CMD_HOME_SET);
        return;
    }
    if (Armed_IsArmed()) {
        Error_Send(frame->request_id, ERROR_ARMED_REJECTED, CMD_HOME_SET);
        return;
    }

    Nav_SetHome(s_protocol_nav,
                read_i32_le(&frame->payload[0]),
                read_i32_le(&frame->payload[4]),
                read_i32_le(&frame->payload[8]));
    send_nav_ack(frame, CMD_HOME_SET, 0U, 1U);
}

static void handle_rth_trigger(const protocol_frame_t *frame)
{
    if (frame->payload_len != 0U) {
        Error_Send(frame->request_id, ERROR_INVALID_PAYLOAD_LENGTH,
                   CMD_RTH_TRIGGER);
        return;
    }

    /* RTH dari web adalah perintah darurat, jadi tetap diterima saat
     * armed. Nav_TriggerRTH() sendiri memilih RTH GPS atau fallback. */
    Nav_TriggerRTH(s_protocol_nav);
    send_nav_ack(frame, CMD_RTH_TRIGGER, 0U,
                 (uint8_t)Nav_GetMode(s_protocol_nav));
}

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

void Nav_Init(NAV_State_t *state)
{
    memset(state, 0, sizeof(*state));
    state->mode        = NAV_MODE_IDLE;
    state->initialized = true;
    ALT_HOLD_Init(&state->altitude_hold);
}

void Nav_SetHome(NAV_State_t *state, int32_t lat_e7, int32_t lon_e7, int32_t altitude_m)
{
    state->home.lat_e7      = lat_e7;
    state->home.lon_e7      = lon_e7;
    state->home.altitude_m  = altitude_m;
    state->home.is_set      = true;
}

bool Nav_LoadMissionItem(NAV_State_t *state, const NAV_MissionItem_t *item)
{
    if (state->mission_count >= NAV_MAX_MISSION_ITEMS) {
        return false;
    }
    state->mission[state->mission_count] = *item;
    state->mission_count++;
    return true;
}

void Nav_ClearMission(NAV_State_t *state)
{
    state->mission_count         = 0U;
    state->mission_current_index = 0U;
    memset(state->mission, 0, sizeof(state->mission));
}

bool Nav_StartWaypoint(NAV_State_t *state)
{
    if (state->mission_count == 0U) {
        return false;
    }

    state->mission_current_index = 0U;
    state->mode = NAV_MODE_WAYPOINT;
    state->failsafe_active = false;
    return true;
}

void Nav_TriggerRTH(NAV_State_t *state)
{
    state->failsafe_active = true;
    /* Failsafe RTH menang atas mode apa pun yang sedang berjalan
     * (termasuk waypoint mission) — konsisten dengan prinsip "link-loss
     * -> trigger RTH" di pembagian-tugas-firmware-4-orang.md. */
    if (!state->home.is_set) {
        /* Belum pernah ada home (mis. arming gagal ke-trigger dulu,
         * kasus langka) — tidak ada tempat yang bisa dituju. Fallback
         * ke level+altitude hold sama seperti kasus tanpa GPS fix. */
        state->mode = NAV_MODE_RTH_NO_GPS_FALLBACK;
        return;
    }

    if (GpsHasUsableFix(&state->last_gps)) {
        state->mode = NAV_MODE_RTH;
    } else {
        state->mode = NAV_MODE_RTH_NO_GPS_FALLBACK;
    }
}

void Nav_EnterAltitudeHold(NAV_State_t *state, float current_alt_m)
{
    ALT_HOLD_Enter(&state->altitude_hold, current_alt_m);
    state->mode = NAV_MODE_ALTITUDE_HOLD;
}

void Nav_ExitToIdle(NAV_State_t *state)
{
    /* Cuma mode yang direset -- home/mission/altitude_hold dibiarkan
     * apa adanya (lihat dokumentasi di navigation.h). setpoint lama
     * juga sengaja dibiarkan basi di struct: NAV_MODE_IDLE membuat
     * task_control() berhenti membaca Nav_GetSetpoint() sama sekali
     * (jalur manual/assisted dipakai), jadi tidak ada konsumen yang
     * bisa salah baca nilai basi ini. */
    state->mode = NAV_MODE_IDLE;
    state->failsafe_active = false;
}

void Nav_Update(NAV_State_t *state,
                const NAV_GpsSample_t *gps,
                float current_alt_m,
                bool armed)
{
    if (!state->initialized) {
        Nav_Init(state);
    }

    state->last_gps = *gps;
    if (GpsHasUsableFix(gps)) {
        state->gps_ever_had_fix = true;
    }

    /* Auto-set home saat transisi ke armed, kalau operator belum set
     * manual lewat CMD_HOME_SET — sesuai kebijakan default di
     * protocol.md Bagian 5 (catatan CMD_HOME_SET). Deteksi transisi
     * disarmed->armed ada di Orang 1 (armed-state handler); di sini
     * cukup syarat: armed=true, home belum di-set, dan GPS punya fix. */
    if (armed && !state->home.is_set && GpsHasUsableFix(gps)) {
        Nav_SetHome(state, gps->lat_e7, gps->lon_e7, (int32_t)current_alt_m);
    }

    switch (state->mode) {

    case NAV_MODE_IDLE:
        /* Tidak ada setpoint aktif — Orang 4 harus treat ini sebagai
         * "manual control", bukan auto-level. Modul ini tidak menulis
         * setpoint di mode ini. */
        break;

    case NAV_MODE_ALTITUDE_HOLD:
        state->setpoint.target_altitude_m = ALT_HOLD_GetTarget(&state->altitude_hold);
        state->setpoint.target_roll_deg   = 0.0f;
        state->setpoint.target_pitch_deg  = 0.0f;
        /* target_yaw_deg sengaja tidak disentuh — biarkan yaw manual
         * (rudder) tetap dikontrol pilot selama altitude-hold-only. */
        break;

    case NAV_MODE_WAYPOINT: {
        WAYPOINT_Output_t wp_out;
        WAYPOINT_Compute(state->mission, state->mission_count,
                          &state->mission_current_index,
                          gps, current_alt_m, &wp_out);

        switch (wp_out.result) {
        case WAYPOINT_RESULT_NO_MISSION:
            /* Tidak ada mission ter-load tapi mode terlanjur WAYPOINT
             * (mis. race command_handler.c) — turun ke altitude-hold
             * di tempat daripada diam tanpa setpoint. TODO: konfirmasi
             * ke tim apakah ini kebijakan yang benar atau harus balik
             * ke NAV_MODE_IDLE. */
            state->setpoint.target_altitude_m = current_alt_m;
            state->setpoint.target_roll_deg   = 0.0f;
            state->setpoint.target_pitch_deg  = 0.0f;
            Nav_EnterAltitudeHold(state, current_alt_m);
            break;
        case WAYPOINT_RESULT_NO_GPS_FALLBACK:
            /* Mission tidak boleh terus berjalan tanpa posisi valid.
             * Gunakan jalur RTH yang sama dengan link-loss; jika GPS
             * belum ada, mode fallback akan menahan airframe level. */
            Nav_TriggerRTH(state);
            break;
        case WAYPOINT_RESULT_ACTION_RTH:
            /* Item aktif eksplisit minta RTH (action_type=RTH) — modul
             * waypoint tidak boleh ubah mode sendiri, jadi dilakukan
             * di sini lewat jalur yang sama dengan failsafe link-loss. */
            Nav_TriggerRTH(state);
            break;
        case WAYPOINT_RESULT_MISSION_COMPLETE:
            /* wp_out.setpoint sudah target altitude item terakhir +
             * roll/pitch level (diisi waypoint.c) — pakai langsung
             * untuk frame ini juga, supaya tidak ada celah satu frame
             * dengan setpoint basi sebelum ALTITUDE_HOLD case jalan. */
            state->setpoint = wp_out.setpoint;
            Nav_EnterAltitudeHold(state, wp_out.setpoint.target_altitude_m);
            break;
        case WAYPOINT_RESULT_TRACKING:
        case WAYPOINT_RESULT_LOITERING:
        case WAYPOINT_RESULT_ADVANCED:
        default:
            state->setpoint = wp_out.setpoint;
            /* Tetap di NAV_MODE_WAYPOINT. */
            break;
        }
        break;
    }

    case NAV_MODE_RTH: {
        RTH_Output_t rth_out;
        RTH_Compute(&state->home, gps, current_alt_m, &rth_out);
        state->setpoint = rth_out.setpoint;

        switch (rth_out.result) {
        case RTH_RESULT_NO_GPS_FALLBACK:
            /* Kehilangan fix (atau home ternyata belum/tidak lagi
             * di-set) di tengah RTH -> downgrade ke fallback, jangan
             * terus terbang ke arah bearing basi. */
            state->mode = NAV_MODE_RTH_NO_GPS_FALLBACK;
            break;
        case RTH_RESULT_ARRIVED:
            /* Sudah "sampai" home secara horizontal. TODO: keputusan
             * tim — auto-land, loiter di atas home, atau altitude-hold
             * menunggu input manual? Untuk skeleton ini: turun ke
             * altitude-hold di tempat supaya tidak orbit tanpa akhir.
             * rth_out.setpoint.target_altitude_m sudah home->altitude_m
             * (diisi rth.c) — dipakai juga sebagai target capture,
             * konsisten dengan jalur masuk altitude-hold yang lain. */
            ALT_HOLD_Enter(&state->altitude_hold, rth_out.setpoint.target_altitude_m);
            state->mode = NAV_MODE_ALTITUDE_HOLD;
            state->failsafe_active = false;   /* BARU: RTH sudah selesai, kembalikan
                                                * kendali throttle ke pilot -- lihat
                                                * catatan di task_control() soal
                                                * Nav_IsFailsafeActive(). */
            break;
        case RTH_RESULT_NAVIGATING:
        default:
            /* Tetap di NAV_MODE_RTH, setpoint sudah di-copy di atas. */
            break;
        }
        break;
    }

    case NAV_MODE_RTH_NO_GPS_FALLBACK: {
        /* Tanpa posisi, satu-satunya hal aman yang bisa dilakukan
         * firmware adalah menstabilkan airframe dan menahan altitude
         * saat ini — TIDAK mencoba menebak arah pulang. Ini keputusan
         * safety minimum untuk skeleton; kalau tim mau perilaku lain
         * (mis. descend perlahan, atau cut throttle untuk mode glide),
         * rth.c (RTH_Compute, kasus RTH_RESULT_NO_GPS_FALLBACK) yang
         * tepat untuk diubah — belum final. */
        RTH_Output_t rth_out;
        RTH_Compute(&state->home, gps, current_alt_m, &rth_out);
        state->setpoint = rth_out.setpoint;

        /* Kalau fix (dan home) tersedia lagi, upgrade balik ke RTH
         * sungguhan — biarkan NAV_MODE_RTH yang urus arrived/navigating
         * di iterasi berikutnya, jangan duplikasi keputusan itu di sini. */
        if (rth_out.result != RTH_RESULT_NO_GPS_FALLBACK) {
            state->mode = NAV_MODE_RTH;
        }
        break;
    }
    }
}

const NAV_Setpoint_t *Nav_GetSetpoint(const NAV_State_t *state)
{
    return &state->setpoint;
}

NAV_Mode_t Nav_GetMode(const NAV_State_t *state)
{
    return state->mode;
}

bool Nav_IsFailsafeActive(const NAV_State_t *state)
{
    return state->failsafe_active;
}

void Nav_RegisterProtocolHandlers(NAV_State_t *state)
{
    s_protocol_nav = state;
    CommandHandler_Register(CMD_MISSION_UPLOAD, handle_mission_upload);
    CommandHandler_Register(CMD_HOME_SET, handle_home_set);
    CommandHandler_Register(CMD_RTH_TRIGGER, handle_rth_trigger);
}
