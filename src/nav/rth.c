/**
 * @file    rth.c
 * @brief   Implementasi RTH — diekstrak dari navigation.c (lihat rth.h
 *          untuk alasan pemisahan).
 *
 * Status: SKELETON, perilaku sama persis dengan yang sebelumnya inline
 * di navigation.c — ini murni refactor struktur, bukan perubahan logic:
 *   - Distance/bearing: flat-earth approximation (cukup untuk RTH jarak
 *     pendek fixed-wing kampus).
 *   - Tanpa GPS fix / home belum di-set: fallback ke level + altitude
 *     hold di tempat, TIDAK mencoba menebak arah pulang. Keputusan
 *     safety minimum, belum final — lihat catatan di rth.h dan
 *     navigation.c.
 *   - Margin altitude selama navigating: masih placeholder 0 (lihat
 *     RTH_ALTITUDE_SAFETY_MARGIN_M di rth.h), TODO tim.
 */

#include "rth.h"
#include <math.h>
#include <string.h>

/* ------------------------------------------------------------------- */
/* Helper statis                                                        */
/* ------------------------------------------------------------------- */

#define DEG_TO_RAD_F        (0.0174532925199433f)
#define RAD_TO_DEG_F        (57.29577951308232f)

/* 1 derajat latitude ~= 111320 meter. Dipakai buat konversi e7 (1e-7
 * derajat) ke meter dalam approximation flat-earth lokal. */
#define METERS_PER_LAT_DEG  111320.0f

static bool GpsHasUsableFix(const NAV_GpsSample_t *gps)
{
    return gps->data_valid &&
           (gps->fix_type == NAV_GPS_FIX_2D || gps->fix_type == NAV_GPS_FIX_3D);
}

/* ------------------------------------------------------------------- */
/* API publik                                                           */
/* ------------------------------------------------------------------- */

void RTH_CalcDistanceBearing(int32_t from_lat_e7, int32_t from_lon_e7,
                              int32_t to_lat_e7, int32_t to_lon_e7,
                              float *out_distance_m, float *out_bearing_deg)
{
    float lat_avg_rad = ((float)from_lat_e7 * 1e-7f) * DEG_TO_RAD_F;

    float dlat_deg = (float)(to_lat_e7 - from_lat_e7) * 1e-7f;
    float dlon_deg = (float)(to_lon_e7 - from_lon_e7) * 1e-7f;

    float north_m = dlat_deg * METERS_PER_LAT_DEG;
    float east_m  = dlon_deg * METERS_PER_LAT_DEG * cosf(lat_avg_rad);

    *out_distance_m = sqrtf(north_m * north_m + east_m * east_m);

    float bearing_rad = atan2f(east_m, north_m);
    float bearing_deg = bearing_rad * RAD_TO_DEG_F;
    if (bearing_deg < 0.0f) {
        bearing_deg += 360.0f;
    }
    *out_bearing_deg = bearing_deg;
}

void RTH_Compute(const NAV_HomePosition_t *home,
                  const NAV_GpsSample_t *gps,
                  float current_alt_m,
                  RTH_Output_t *out)
{
    memset(out, 0, sizeof(*out));

    if (!home->is_set || !GpsHasUsableFix(gps)) {
        /* Tanpa home ATAU tanpa fix: satu-satunya hal aman adalah
         * menstabilkan airframe dan menahan altitude saat ini — TIDAK
         * mencoba menebak arah pulang. */
        out->result                     = RTH_RESULT_NO_GPS_FALLBACK;
        out->setpoint.target_roll_deg   = 0.0f;
        out->setpoint.target_pitch_deg  = 0.0f;
        out->setpoint.target_altitude_m = current_alt_m;
        return;
    }

    float distance_m, bearing_deg;
    RTH_CalcDistanceBearing(gps->lat_e7, gps->lon_e7,
                             home->lat_e7, home->lon_e7,
                             &distance_m, &bearing_deg);
    out->distance_to_home_m  = distance_m;
    out->bearing_to_home_deg = bearing_deg;

    if (distance_m <= NAV_WAYPOINT_ACCEPT_RADIUS_M) {
        /* Sudah "sampai" home secara horizontal. TODO: keputusan tim —
         * auto-land, loiter di atas home, atau altitude-hold menunggu
         * input manual? Untuk skeleton ini: caller (navigation.c)
         * men-downgrade mode ke ALTITUDE_HOLD di titik home. */
        out->result                     = RTH_RESULT_ARRIVED;
        out->setpoint.target_altitude_m = (float)home->altitude_m;
        out->setpoint.target_roll_deg   = 0.0f;
        out->setpoint.target_pitch_deg  = 0.0f;
        return;
    }

    out->result                     = RTH_RESULT_NAVIGATING;
    out->setpoint.target_yaw_deg    = bearing_deg;
    /* TODO: RTH_ALTITUDE_SAFETY_MARGIN_M masih 0 (placeholder aman),
     * bukan keputusan final — lihat catatan di rth.h. */
    out->setpoint.target_altitude_m = current_alt_m + RTH_ALTITUDE_SAFETY_MARGIN_M;
    out->setpoint.target_roll_deg   = 0.0f; /* koordinasi turn: Orang 4/PID yang urus roll-to-turn */
    out->setpoint.target_pitch_deg  = 0.0f;
}
