/**
 * @file    telemetry.c
 * @brief   Implementasi builder payload telemetri. Lihat telemetry.h.
 */
#include "telemetry.h"
#include "command_handler.h"
#include "navigation.h"      /* NAV_GPS_FIX_* untuk pemetaan eksplisit */
#include <string.h>

/* Panjang wire = jumlah field; dikunci compile-time supaya salah hitung
 * tertangkap saat build, bukan saat debug lintas dua repo. */
_Static_assert(TELEM_ATTITUDE_LEN == 4u + 4u + 4u + 1u,        "CMD_ATTITUDE harus 13 byte");
_Static_assert(TELEM_GPS_LEN     == 4u + 4u + 2u + 1u + 1u,    "CMD_GPS_DATA harus 12 byte");
_Static_assert(TELEM_BATTERY_LEN == 2u + 2u,                   "CMD_BATTERY harus 4 byte");
_Static_assert(TELEM_IMU_RAW_LEN == 2u * 6u * 2u,              "CMD_IMU_RAW harus 24 byte");
_Static_assert(TELEM_IMU_RAW_LEN <= PROTOCOL_MAX_PAYLOAD,      "payload melebihi batas frame");

uint8_t Telemetry_GpsFixToWire(int nav_fix_type)
{
    switch (nav_fix_type) {
        case NAV_GPS_FIX_2D:   return TELEM_GPS_FIX_2D;
        case NAV_GPS_FIX_3D:   return TELEM_GPS_FIX_3D;
        case NAV_GPS_FIX_NONE:
        default:               return TELEM_GPS_FIX_NONE;
    }
}

uint16_t Telemetry_SatU16(float value)
{
    if (!(value > 0.0f)) {            /* negatif, 0, atau NaN */
        return 0u;
    }
    if (value >= 65535.0f) {
        return 65535u;
    }
    return (uint16_t)(value + 0.5f);  /* dibulatkan, aman: 0 < value < 65535 */
}

static void put_f32(uint8_t *out, float v)     { memcpy(out, &v, sizeof(float)); }
static void put_i32(uint8_t *out, int32_t v)   { memcpy(out, &v, sizeof(int32_t)); }
static void put_u16(uint8_t *out, uint16_t v)  { memcpy(out, &v, sizeof(uint16_t)); }
static void put_i16(uint8_t *out, int16_t v)   { memcpy(out, &v, sizeof(int16_t)); }
/* memcpy langsung = little-endian di Cortex-M4F dan host x86/ARM LE --
 * pola yang sama dipakai settings.c / handle_get_status(). */

uint8_t Telemetry_BuildAttitude(uint8_t *out, const TelemetrySnapshot_t *s)
{
    put_f32(&out[0], s->roll_deg);
    put_f32(&out[4], s->pitch_deg);
    put_f32(&out[8], s->yaw_deg);
    out[12] = s->active_imu;
    return TELEM_ATTITUDE_LEN;
}

uint8_t Telemetry_BuildGps(uint8_t *out, const TelemetrySnapshot_t *s)
{
    /* mm/s -> cm/s; negatif (tidak seharusnya, tapi int32 dari parser)
     * dijepit ke 0, di atas 65535 cm/s dijenuhkan. */
    int32_t cms = s->ground_speed_mm_s / 10;
    uint16_t speed_cms = (cms < 0) ? 0u : (cms > 65535) ? 65535u : (uint16_t)cms;

    put_i32(&out[0], s->lat_e7);
    put_i32(&out[4], s->lon_e7);
    put_u16(&out[8], speed_cms);
    out[10] = Telemetry_GpsFixToWire(s->nav_fix_type);
    out[11] = s->sat_count;
    return TELEM_GPS_LEN;
}

uint8_t Telemetry_BuildBattery(uint8_t *out, const TelemetrySnapshot_t *s)
{
    put_u16(&out[0], Telemetry_SatU16(s->voltage_volts * 1000.0f));
    put_u16(&out[2], Telemetry_SatU16(s->current_amps  * 1000.0f));
    return TELEM_BATTERY_LEN;
}

uint8_t Telemetry_BuildImuRaw(uint8_t *out, const TelemetryImuRaw_t *raw)
{
    size_t idx = 0;
    for (int i = 0; i < 6; i++) {
        put_i16(&out[idx], (raw != NULL && raw->primary_valid) ? raw->primary[i] : 0);
        idx += sizeof(int16_t);
    }
    for (int i = 0; i < 6; i++) {
        put_i16(&out[idx], (raw != NULL && raw->secondary_valid) ? raw->secondary[i] : 0);
        idx += sizeof(int16_t);
    }
    return (uint8_t)idx;
}

void Telemetry_PushAll(const TelemetrySnapshot_t *s)
{
    uint8_t buf[TELEM_ATTITUDE_LEN];   /* terbesar di antara ketiganya (13) */
    uint8_t len;

    len = Telemetry_BuildAttitude(buf, s);
    (void)Protocol_SendUnsolicited(CMD_ATTITUDE, buf, len);

    len = Telemetry_BuildGps(buf, s);
    (void)Protocol_SendUnsolicited(CMD_GPS_DATA, buf, len);

    len = Telemetry_BuildBattery(buf, s);
    (void)Protocol_SendUnsolicited(CMD_BATTERY, buf, len);
}

/* --- CMD_IMU_RAW on-demand ------------------------------------------------ */
static const TelemetryImuRaw_t *s_imu_raw_src;

static void handle_imu_raw(const protocol_frame_t *frame)
{
    uint8_t payload[TELEM_IMU_RAW_LEN];
    uint8_t len = Telemetry_BuildImuRaw(payload, s_imu_raw_src);
    Protocol_SendFrame(CMD_IMU_RAW, frame->request_id, payload, len);
}

void Telemetry_RegisterCommands(const TelemetryImuRaw_t *raw)
{
    s_imu_raw_src = raw;
    CommandHandler_Register(CMD_IMU_RAW, handle_imu_raw);
}
