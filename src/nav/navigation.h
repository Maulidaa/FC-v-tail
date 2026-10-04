/**
 * @file    navigation.h
 * @brief   Navigasi: RTH (+ fallback tanpa GPS fix), altitude hold,
 *          waypoint tracking.
 *
 * Tanggung jawab: Orang 3 (Fusion, Navigasi & Failsafe)
 *
 * Input modul ini:
 *   - GPS data terparsing dari GPS UBX parser (USART2) -> Orang 2
 *   - Attitude dari ahrs_fusion.c (modul pertama, sudah dibuat)
 *   - Home position dari CMD_HOME_SET (protocol.md Bagian 10):
 *     "[lat_e7: i32][lon_e7: i32][altitude_m: i32]" — atau default
 *     posisi GPS saat arming kalau operator tidak set manual
 *     (protocol.md Bagian 5, catatan CMD_HOME_SET).
 *   - Mission item dari CMD_MISSION_UPLOAD (protocol.md Bagian 6 & 10):
 *     "[seq: u8][lat_e7: i32][lon_e7: i32][altitude_m: i16]
 *      [action_type: u8][action_param: i32]"
 *
 * Output modul ini:
 *   - Setpoint (target roll/pitch/yaw/altitude) untuk PID+mixer -> Orang 4
 *   - Sinyal trigger RTH untuk failsafe (dari link-loss iBUS, modul
 *     RX di file terpisah, rx_ibus.c)
 */

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <stdint.h>
#include <stdbool.h>
#include "altitude_hold.h"   /* ALT_HOLD_State_t — tidak circular, altitude_hold.h
                               * tidak include navigation.h (lihat catatannya). */

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------- */
/* Konfigurasi (TODO: tuning pasca uji terbang)                         */
/* ------------------------------------------------------------------- */

#define NAV_UPDATE_RATE_HZ             50U
#define NAV_UPDATE_DT_S                (1.0f / NAV_UPDATE_RATE_HZ)

/** Radius dianggap "sampai" waypoint/home (meter). TODO: tuning,
 *  tergantung kecepatan jelajah airframe V-tail ini. */
#define NAV_WAYPOINT_ACCEPT_RADIUS_M   10.0f

/** Maksimum item mission yang ditampung di RAM. Cukup besar untuk
 *  misi sederhana kampus; kalau kurang, koordinasikan ke Orang 1
 *  soal budget RAM (128KB total di F411CEU6). */
#define NAV_MAX_MISSION_ITEMS          32U

/* ------------------------------------------------------------------- */
/* Tipe data                                                            */
/* ------------------------------------------------------------------- */

/** Fix type GPS — cocokkan ke enum yang dipakai GPS UBX parser Orang 2
 *  dan field fix type di CMD_GPS_DATA (protocol.md ID 0x0102). */
typedef enum {
    NAV_GPS_FIX_NONE = 0,
    NAV_GPS_FIX_2D,
    NAV_GPS_FIX_3D,
} NAV_GpsFixType_t;

/** Snapshot data GPS, diisi caller dari hasil parser UBX (Orang 2)
 *  sebelum dipanggil ke Nav_Update(). */
typedef struct {
    int32_t          lat_e7;
    int32_t          lon_e7;
    float            ground_speed_mps;
    NAV_GpsFixType_t fix_type;
    uint8_t          sat_count;
    bool             data_valid;
} NAV_GpsSample_t;

/** Jenis aksi mission item, cocok dengan action_type di payload
 *  CMD_MISSION_UPLOAD (protocol.md Bagian 10). */
typedef enum {
    NAV_ACTION_WAYPOINT = 0,
    NAV_ACTION_LOITER,
    NAV_ACTION_RTH,
} NAV_ActionType_t;

/** Satu item mission — representasi in-memory dari item yang di-upload
 *  chunked lewat CMD_MISSION_UPLOAD. */
typedef struct {
    uint8_t          seq;
    int32_t          lat_e7;
    int32_t          lon_e7;
    int16_t          altitude_m;
    NAV_ActionType_t action_type;
    /** Radius (meter) kalau action_type == LOITER, diabaikan untuk
     *  WAYPOINT/RTH — sesuai protocol.md Bagian 10. */
    int32_t          action_param;
} NAV_MissionItem_t;

/** Mode navigasi aktif saat ini. */
typedef enum {
    NAV_MODE_IDLE = 0,
    NAV_MODE_ALTITUDE_HOLD,
    NAV_MODE_WAYPOINT,          /* tracking-nya di waypoint.c, lihat file itu */
    NAV_MODE_RTH,
    NAV_MODE_RTH_NO_GPS_FALLBACK,
} NAV_Mode_t;

/** Home position, di-set lewat Nav_SetHome() (dipanggil command_handler.c
 *  saat CMD_HOME_SET diterima, atau otomatis saat arming kalau belum
 *  pernah di-set manual — kebijakan default ada di protocol.md Bagian 5). */
typedef struct {
    int32_t lat_e7;
    int32_t lon_e7;
    int32_t altitude_m;
    bool    is_set;
} NAV_HomePosition_t;

/** Setpoint keluaran untuk Orang 4 (PID + mixer). Unit derajat untuk
 *  attitude, meter untuk altitude. */
typedef struct {
    float target_roll_deg;
    float target_pitch_deg;
    float target_yaw_deg;
    float target_altitude_m;
} NAV_Setpoint_t;

/** State internal modul navigasi. Satu instance per sistem. */
typedef struct {
    NAV_Mode_t          mode;
    NAV_HomePosition_t  home;
    NAV_Setpoint_t      setpoint;

    NAV_MissionItem_t   mission[NAV_MAX_MISSION_ITEMS];
    uint8_t             mission_count;
    uint8_t             mission_current_index;

    NAV_GpsSample_t     last_gps;
    bool                gps_ever_had_fix;
    bool                failsafe_active;

    /** State capture/hold altitude — lihat altitude_hold.h. Jangan
     *  tulis field ini langsung, selalu lewat Nav_EnterAltitudeHold()
     *  atau ALT_HOLD_Enter(). */
    ALT_HOLD_State_t    altitude_hold;

    bool                initialized;
} NAV_State_t;

/* ------------------------------------------------------------------- */
/* API                                                                  */
/* ------------------------------------------------------------------- */

/** Inisialisasi state navigasi. Panggil sekali saat boot. */
void Nav_Init(NAV_State_t *state);

/**
 * @brief  Set/replace home position. Dipanggil command_handler.c saat
 *         CMD_HOME_SET diterima, atau otomatis oleh armed-state handler
 *         (Orang 1) saat arming kalau operator belum set manual.
 */
void Nav_SetHome(NAV_State_t *state, int32_t lat_e7, int32_t lon_e7, int32_t altitude_m);

/**
 * @brief  Muat satu item mission ke buffer in-memory. Dipanggil
 *         command_handler.c per item saat proses chunking
 *         CMD_MISSION_UPLOAD selesai di-assemble (lihat protocol.md
 *         Bagian 6 — reassembly chunk itu tanggung jawab Orang 1 di
 *         command_handler.c, modul ini cuma nerima hasil akhirnya).
 *
 * @return true kalau berhasil ditambahkan, false kalau buffer penuh
 *         (caller harus balas status error, bukan tanggung jawab
 *         modul ini untuk kirim frame balasan).
 */
bool Nav_LoadMissionItem(NAV_State_t *state, const NAV_MissionItem_t *item);

/** Hapus seluruh mission yang ter-load (mis. sebelum upload baru). */
void Nav_ClearMission(NAV_State_t *state);

/** Mulai mission dari item pertama. Mengembalikan false jika mission kosong. */
bool Nav_StartWaypoint(NAV_State_t *state);

/** Daftarkan handler upload mission dan set-home ke protokol USB. */
void Nav_RegisterProtocolHandlers(NAV_State_t *state);

/**
 * @brief  Trigger RTH. Dipanggil dari rx_ibus.c saat failsafe link-loss
 *         terdeteksi, atau dari command_handler.c kalau ada command
 *         RTH manual dari web (belum ada ID khusus di protocol.md v1 —
 *         TODO: konfirmasi apakah perlu command terpisah atau cukup
 *         lewat mission item action_type=RTH).
 */
void Nav_TriggerRTH(NAV_State_t *state);

/**
 * @brief  Masuk ke NAV_MODE_ALTITUDE_HOLD, meng-capture current_alt_m
 *         sebagai target. WAJIB dipanggil untuk masuk ke mode ini dari
 *         luar navigation.c (mis. command_handler.c kalau ada command
 *         altitude-hold manual dari web — sama seperti RTH, belum ada
 *         ID khusus di protocol.md v1) — JANGAN set state->mode secara
 *         langsung, itu akan melewatkan capture dan menahan altitude
 *         di nilai basi/salah. Lihat altitude_hold.h untuk alasannya.
 *         Transisi internal (dari RTH arrived / mission complete) di
 *         navigation.c sendiri juga lewat fungsi ini untuk konsistensi.
 */
void Nav_EnterAltitudeHold(NAV_State_t *state, float current_alt_m);

/**
 * @brief  Kembali ke NAV_MODE_IDLE — serahkan kontrol penuh ke pilot
 *         (assisted/manual di sisi task_control(), bukan tanggung jawab
 *         modul ini). Dipanggil dari task_control() saat pilot memindah
 *         switch mode (SWC) keluar dari posisi "auto navigation" —
 *         TERMASUK saat itu berarti membatalkan RTH/waypoint yang
 *         sedang berjalan: pilot switch secara sengaja selalu menang
 *         atas mode navigasi otomatis apa pun, kapan pun ganti posisi.
 *
 *         TIDAK menyentuh home/mission/altitude_hold state (tetap
 *         tersimpan) — cuma mode yang direset, supaya kalau nanti masuk
 *         lagi lewat Nav_EnterAltitudeHold() atau Nav_TriggerRTH(),
 *         datanya masih ada (mis. tidak perlu re-arm home position).
 *
 *         JANGAN dipanggil dari jalur failsafe link-loss (rx_ibus.c) —
 *         di sana switch mode justru tidak boleh dipercaya (nilai bisa
 *         basi saat link putus), lihat catatan gating link_ok di
 *         task_control().
 */
void Nav_ExitToIdle(NAV_State_t *state);

/**
 * @brief  Update loop navigasi. Dipanggil scheduler (Orang 1) di rate
 *         NAV_UPDATE_RATE_HZ.
 *
 * @param  state        State navigasi (in/out).
 * @param  gps          Sample GPS terbaru dari parser UBX (Orang 2).
 * @param  current_alt_m Estimasi altitude saat ini (dari baro BMP280,
 *                        Orang 2 — bukan dari GPS altitude yang kurang
 *                        akurat untuk hold jangka pendek).
 * @param  armed         Status armed saat ini (dari Orang 1) — dipakai
 *                        untuk auto-set home saat transisi disarmed->armed
 *                        kalau home belum pernah di-set manual.
 */
void Nav_Update(NAV_State_t *state,
                const NAV_GpsSample_t *gps,
                float current_alt_m,
                bool armed);

/** Ambil setpoint terbaru untuk dikonsumsi Orang 4 (PID+mixer). */
const NAV_Setpoint_t *Nav_GetSetpoint(const NAV_State_t *state);

/** Mode navigasi aktif saat ini — dipakai juga untuk expose status
 *  ke web kalau nanti diperlukan field tambahan di CMD_GET_STATUS. */
NAV_Mode_t Nav_GetMode(const NAV_State_t *state);

bool Nav_IsFailsafeActive(const NAV_State_t *state);

#ifdef __cplusplus
}
#endif

#endif /* NAVIGATION_H */
