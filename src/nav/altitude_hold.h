/**
 * @file    altitude_hold.h
 * @brief   Capture & hold target altitude, diekstrak dari
 *          navigation.c (kasus NAV_MODE_ALTITUDE_HOLD).
 *
 * Tanggung jawab: Orang 3 (Fusion, Navigasi & Failsafe)
 *
 * Kenapa ini butuh modul sendiri padahal logikanya kelihatan sepele
 * (roll/pitch selalu 0, altitude "tetap"): masalahnya justru di kata
 * "tetap" itu — sebelum modul ini ada, NAV_MODE_ALTITUDE_HOLD di
 * navigation.c cuma melakukan self-assignment
 * (`target_altitude_m = target_altitude_m`), yang KEBETULAN aman kalau
 * masuk ke mode ini lewat transisi dari RTH_RESULT_ARRIVED atau
 * WAYPOINT_RESULT_MISSION_COMPLETE (karena setpoint sudah ke-isi benar
 * oleh rth.c/waypoint.c sebelum switch mode). Tapi kalau nanti
 * command_handler.c (Orang 1) mengizinkan operator masuk altitude-hold
 * langsung dari web (command manual, belum ada ID di protocol.md v1),
 * dan set state->mode = NAV_MODE_ALTITUDE_HOLD tanpa lewat jalur itu,
 * target_altitude_m bisa jadi nilai basi atau 0 dari Nav_Init() —
 * pesawat akan mencoba "hold" di altitude yang salah.
 *
 * Modul ini mewajibkan capture eksplisit lewat ALT_HOLD_Enter() /
 * Nav_EnterAltitudeHold() (lihat navigation.h) supaya target SELALU
 * di-set dari altitude aktual saat mode dimasuki, tidak pernah
 * bergantung pada nilai lama yang kebetulan benar.
 */

#ifndef ALTITUDE_HOLD_H
#define ALTITUDE_HOLD_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float target_altitude_m;
    bool  is_captured;   /* false sebelum ALT_HOLD_Enter() pernah dipanggil —
                           * dipakai buat deteksi kalau ada yang salah pakai
                           * modul ini (assert / log di caller kalau perlu). */
} ALT_HOLD_State_t;

/** Inisialisasi. Panggil sekali saat boot (dipanggil dari Nav_Init()). */
void ALT_HOLD_Init(ALT_HOLD_State_t *s);

/**
 * @brief  Capture current_alt_m sebagai target baru. WAJIB dipanggil
 *         TEPAT SEKALI saat transisi ke NAV_MODE_ALTITUDE_HOLD — bukan
 *         tiap frame. Kalau dipanggil tiap frame, target akan terus
 *         "mengejar" altitude aktual dan hold tidak pernah benar-benar
 *         menahan di satu ketinggian (kontraproduktif dengan tujuan
 *         mode ini).
 *
 * @param  current_alt_m  Altitude yang mau di-hold. Caller boleh kirim
 *                          altitude aktual sekarang (baro), ATAU target
 *                          altitude yang lebih relevan kalau tersedia
 *                          (mis. home->altitude_m saat RTH baru arrived
 *                          — lebih tepat daripada altitude sesaat
 *                          sebelum benar-benar berhenti turun/naik).
 */
void ALT_HOLD_Enter(ALT_HOLD_State_t *s, float current_alt_m);

/** Ambil target altitude yang sedang di-hold. Aman dipanggil tiap
 *  frame selama mode == NAV_MODE_ALTITUDE_HOLD. */
float ALT_HOLD_GetTarget(const ALT_HOLD_State_t *s);

#ifdef __cplusplus
}
#endif

#endif /* ALTITUDE_HOLD_H */
