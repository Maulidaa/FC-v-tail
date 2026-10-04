/**
 * @file    rth.h
 * @brief   Logic Return-To-Home (RTH), diekstrak dari navigation.c.
 *
 * Tanggung jawab: Orang 3 (Fusion, Navigasi & Failsafe)
 *
 * Modul ini SENGAJA dibuat sebagai fungsi murni (tidak menyimpan state
 * internal, tidak ada Init/instance) — navigation.c tetap yang pegang
 * mode state machine (NAV_MODE_RTH / NAV_MODE_RTH_NO_GPS_FALLBACK) dan
 * memutuskan transisi mode; rth.c cuma menjawab satu pertanyaan tiap
 * dipanggil: "dengan home + GPS + altitude saat ini, setpoint apa yang
 * harus dipakai, dan apakah masih navigating / sudah sampai / harus
 * fallback?" Pemisahan ini menghindari dua sumber kebenaran untuk
 * logic RTH yang sama kalau nanti file lain (mis. mission waypoint)
 * juga butuh menge-trigger RTH sebagai salah satu action_type.
 *
 * Dipanggil dari navigation.c (Nav_Update(), kasus NAV_MODE_RTH dan
 * NAV_MODE_RTH_NO_GPS_FALLBACK). Tidak dipanggil langsung oleh
 * command_handler.c atau rx_ibus.c — keduanya tetap lewat
 * Nav_TriggerRTH() di navigation.h supaya mode state machine konsisten.
 */

#ifndef RTH_H
#define RTH_H

#include <stdint.h>
#include <stdbool.h>
#include "navigation.h"   /* reuse NAV_GpsSample_t, NAV_HomePosition_t, NAV_Setpoint_t */

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------- */
/* Konfigurasi (TODO: keputusan tim, belum final)                       */
/* ------------------------------------------------------------------- */

/** Margin ketinggian tambahan di atas altitude saat ini selama
 *  navigating balik ke home, supaya tidak nyangkut obstacle di rute
 *  pulang. TODO: protocol.md Bagian 12 / navigation.c asli sudah catat
 *  ini sebagai keputusan tim yang belum diambil — nilai 0 di sini
 *  BUKAN keputusan final, cuma placeholder aman (tidak nambah risiko
 *  dibanding skeleton sebelumnya) sampai tim putuskan angka N-nya. */
#define RTH_ALTITUDE_SAFETY_MARGIN_M   0.0f

/* ------------------------------------------------------------------- */
/* Tipe data                                                            */
/* ------------------------------------------------------------------- */

typedef enum {
    RTH_RESULT_NAVIGATING = 0,     /* home & fix tersedia, masih menuju home */
    RTH_RESULT_ARRIVED,            /* sudah dalam radius accept dari home */
    RTH_RESULT_NO_GPS_FALLBACK,    /* home belum di-set, atau fix hilang —
                                     * satu-satunya hal aman: level + hold */
} RTH_Result_t;

typedef struct {
    NAV_Setpoint_t setpoint;
    RTH_Result_t   result;
    /** Hanya valid kalau result != RTH_RESULT_NO_GPS_FALLBACK (kalau
     *  fallback, tidak ada posisi terhadap home yang bisa dihitung). */
    float          distance_to_home_m;
    float          bearing_to_home_deg;
} RTH_Output_t;

/* ------------------------------------------------------------------- */
/* API                                                                  */
/* ------------------------------------------------------------------- */

/**
 * @brief  Hitung jarak (meter) dan bearing (derajat, 0=utara, CW) dari
 *         satu titik e7 ke titik e7 lain, pakai flat-earth
 *         approximation. Diekspos publik (bukan static) supaya modul
 *         waypoint (masih stub di navigation.c) bisa reuse — hindari
 *         dua implementasi flat-earth math yang bisa diverge.
 *
 *         TODO: kalau nanti butuh presisi lebih jauh dari beberapa km,
 *         ganti ke haversine — untuk RTH/waypoint jarak pendek khas
 *         airframe kampus ini, flat-earth cukup dan jauh lebih murah
 *         untuk MCU tanpa FPU double.
 */
void RTH_CalcDistanceBearing(int32_t from_lat_e7, int32_t from_lon_e7,
                              int32_t to_lat_e7, int32_t to_lon_e7,
                              float *out_distance_m, float *out_bearing_deg);

/**
 * @brief  Hitung satu langkah RTH. Fungsi murni — tidak menyimpan state,
 *         aman dipanggil tiap loop dari Nav_Update() tanpa side effect
 *         selain menulis ke *out.
 *
 * @param  home           Home position saat ini (NULL tidak diizinkan;
 *                          cek home->is_set di dalam, bukan caller).
 * @param  gps            Sample GPS terbaru dari parser UBX (Orang 2).
 * @param  current_alt_m  Estimasi altitude saat ini (dari baro BMP280,
 *                          Orang 2 — bukan GPS altitude).
 * @param  out            Output setpoint + status (wajib non-NULL).
 */
void RTH_Compute(const NAV_HomePosition_t *home,
                  const NAV_GpsSample_t *gps,
                  float current_alt_m,
                  RTH_Output_t *out);

#ifdef __cplusplus
}
#endif

#endif /* RTH_H */
