/**
 * @file    telemetry.h
 * @brief   Payload telemetri FC -> web (Comms #4): CMD_ATTITUDE,
 *          CMD_GPS_DATA, CMD_BATTERY (push berkala, request_id=0x00) dan
 *          CMD_IMU_RAW (on-demand, dibalas sinkron saat diminta).
 *
 * Wire format (sumber kebenaran: telemetryCommands.ts di faas-configurator),
 * semua multi-byte little-endian:
 *   CMD_ATTITUDE (0x0101) 13 B: roll f32, pitch f32, yaw f32, activeImu u8
 *   CMD_GPS_DATA (0x0102) 12 B: latE7 i32, lonE7 i32, speedCms u16,
 *                               fixType u8, satCount u8
 *   CMD_BATTERY  (0x0103)  4 B: voltageMv u16, currentMa u16
 *   CMD_IMU_RAW  (0x0104) 24 B: primary lalu secondary, tiap IMU
 *                               accelXYZ i16 x3 lalu gyroXYZ i16 x3
 *
 * Builder di sini fungsi murni (tanpa hardware) supaya bisa diuji di host.
 * Semua konversi float->integer memakai SATURASI (bukan cast langsung):
 * cast float di luar rentang tipe tujuan adalah undefined behavior di C,
 * dan arus bisa negatif karena offset bias current sense.
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TELEM_ATTITUDE_LEN   13u   /* 4+4+4+1 */
#define TELEM_GPS_LEN        12u   /* 4+4+2+1+1 */
#define TELEM_BATTERY_LEN     4u   /* 2+2 */
#define TELEM_IMU_RAW_LEN    24u   /* 2 IMU x 6 x i16 */

/** Nilai wire fixType (index GPS_FIX_TYPES web: no-fix, 2d, 3d). */
#define TELEM_GPS_FIX_NONE   0u
#define TELEM_GPS_FIX_2D     1u
#define TELEM_GPS_FIX_3D     2u

/** Snapshot input untuk satu siklus push -- diisi main.c dari s_fc. */
typedef struct {
    float   roll_deg, pitch_deg, yaw_deg;
    uint8_t active_imu;          /* 0=primary (MPU6500), 1=secondary (MPU6050) */

    int32_t lat_e7, lon_e7;
    int32_t ground_speed_mm_s;
    int     nav_fix_type;        /* NAV_GpsFixType_t (NAV_GPS_FIX_*), dipetakan eksplisit */
    uint8_t sat_count;

    float   voltage_volts;
    float   current_amps;
} TelemetrySnapshot_t;

/** Raw IMU untuk CMD_IMU_RAW: urutan accelXYZ, gyroXYZ (LSB mentah). */
typedef struct {
    int16_t primary[6];
    int16_t secondary[6];
    bool    primary_valid;       /* false -> dikirim nol */
    bool    secondary_valid;     /* false -> dikirim nol */
} TelemetryImuRaw_t;

/** Pemetaan eksplisit NAV_GPS_FIX_* -> nilai wire (switch, bukan cast). */
uint8_t Telemetry_GpsFixToWire(int nav_fix_type);

/** Saturating float -> u16 dengan pembulatan; NaN dan negatif -> 0. */
uint16_t Telemetry_SatU16(float value);

/* Builder: tulis payload ke `out` (minimal TELEM_*_LEN byte), return panjang. */
uint8_t Telemetry_BuildAttitude(uint8_t *out, const TelemetrySnapshot_t *s);
uint8_t Telemetry_BuildGps(uint8_t *out, const TelemetrySnapshot_t *s);
uint8_t Telemetry_BuildBattery(uint8_t *out, const TelemetrySnapshot_t *s);
uint8_t Telemetry_BuildImuRaw(uint8_t *out, const TelemetryImuRaw_t *raw);

/**
 * @brief Kirim ketiga frame push (ATTITUDE, GPS_DATA, BATTERY) lewat
 *        Protocol_SendUnsolicited() (request_id=0x00). CMD_IMU_RAW
 *        SENGAJA tidak ikut -- on-demand saja.
 */
void Telemetry_PushAll(const TelemetrySnapshot_t *s);

/**
 * @brief Daftarkan handler CMD_IMU_RAW (on-demand) ke command_handler.c,
 *        terikat ke `raw` (pola sama Nav_RegisterProtocolHandlers()).
 *        WAJIB dipanggil SETELAH CommandHandler_Init(). `raw` harus hidup
 *        selama firmware berjalan (di main.c: field s_fc). NULL -> handler
 *        membalas semua nol tanpa crash.
 */
void Telemetry_RegisterCommands(const TelemetryImuRaw_t *raw);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_H */
