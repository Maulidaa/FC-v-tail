/**
 * calib_mag.c
 *
 * Lihat calib_mag.h untuk kontrak publik & penjelasan metode
 * (hard/soft-iron sederhana + progress berbasis cakupan oktan).
 */

#include "calib_mag.h"
#include <stddef.h>

#define OCTANT_COUNT     (8u)
#define MIN_MAGNITUDE_SQ (100 * 100) /* abaikan sample terlalu lemah/noise */

static CalibMagState_t s_state = CALIB_MAG_STATE_IDLE;

static int16_t s_min_x, s_max_x;
static int16_t s_min_y, s_max_y;
static int16_t s_min_z, s_max_z;

static bool s_octant_visited[OCTANT_COUNT];
static uint8_t s_octant_visited_count;

static MagCalibResult_t s_result;

void CalibMag_Start(void)
{
    s_state = CALIB_MAG_STATE_IN_PROGRESS;

    s_min_x = s_min_y = s_min_z = INT16_MAX;
    s_max_x = s_max_y = s_max_z = INT16_MIN;

    for (uint8_t i = 0; i < OCTANT_COUNT; i++) {
        s_octant_visited[i] = false;
    }
    s_octant_visited_count = 0;
}

void CalibMag_Stop(void)
{
    s_state = CALIB_MAG_STATE_IDLE;
}

static uint8_t octant_index(int16_t x, int16_t y, int16_t z)
{
    /* Oktan ditentukan murni dari kombinasi tanda tiap axis — cukup
     * untuk deteksi cakupan orientasi kasar, tidak perlu normalisasi
     * vektor penuh. */
    uint8_t idx = 0;
    if (x >= 0) idx |= 0x01u;
    if (y >= 0) idx |= 0x02u;
    if (z >= 0) idx |= 0x04u;
    return idx;
}

static void finalize_result(void)
{
    int32_t range_x = (int32_t)s_max_x - (int32_t)s_min_x;
    int32_t range_y = (int32_t)s_max_y - (int32_t)s_min_y;
    int32_t range_z = (int32_t)s_max_z - (int32_t)s_min_z;

    if (range_x <= 0 || range_y <= 0 || range_z <= 0) {
        /* Salah satu axis tidak pernah berubah sama sekali — data
         * tidak valid untuk dihitung skalanya, gagalkan daripada
         * membagi dengan nol / menghasilkan scale tak masuk akal */
        s_state = CALIB_MAG_STATE_FAILED;
        return;
    }

    float avg_range = (float)(range_x + range_y + range_z) / 3.0f;

    s_result.offset_x = (int16_t)(((int32_t)s_max_x + (int32_t)s_min_x) / 2);
    s_result.offset_y = (int16_t)(((int32_t)s_max_y + (int32_t)s_min_y) / 2);
    s_result.offset_z = (int16_t)(((int32_t)s_max_z + (int32_t)s_min_z) / 2);

    s_result.scale_x = avg_range / (float)range_x;
    s_result.scale_y = avg_range / (float)range_y;
    s_result.scale_z = avg_range / (float)range_z;

    s_state = CALIB_MAG_STATE_DONE;
}

void CalibMag_FeedSample(int16_t x, int16_t y, int16_t z)
{
    if (s_state != CALIB_MAG_STATE_IN_PROGRESS) {
        return;
    }

    int32_t mag_sq = (int32_t)x * x + (int32_t)y * y + (int32_t)z * z;
    if (mag_sq < MIN_MAGNITUDE_SQ) {
        return; /* sample terlalu lemah, kemungkinan noise/gangguan lokal */
    }

    if (x < s_min_x) s_min_x = x;
    if (x > s_max_x) s_max_x = x;
    if (y < s_min_y) s_min_y = y;
    if (y > s_max_y) s_max_y = y;
    if (z < s_min_z) s_min_z = z;
    if (z > s_max_z) s_max_z = z;

    uint8_t idx = octant_index(x, y, z);
    if (!s_octant_visited[idx]) {
        s_octant_visited[idx] = true;
        s_octant_visited_count++;
    }

    if (s_octant_visited_count >= OCTANT_COUNT) {
        finalize_result();
    }
}

CalibMagState_t CalibMag_GetState(void)
{
    return s_state;
}

uint8_t CalibMag_GetProgressPercent(void)
{
    if (s_state == CALIB_MAG_STATE_DONE) {
        return 100u;
    }
    if (s_state != CALIB_MAG_STATE_IN_PROGRESS) {
        return 0u;
    }
    return (uint8_t)((s_octant_visited_count * 100u) / OCTANT_COUNT);
}

bool CalibMag_GetResult(MagCalibResult_t *out)
{
    if (s_state != CALIB_MAG_STATE_DONE || out == NULL) {
        return false;
    }
    *out = s_result;
    return true;
}
