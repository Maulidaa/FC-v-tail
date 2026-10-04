/**
 * @file    test_telemetry.c
 * @brief   Test payload telemetri (Comms #4) -- kode ASLI telemetry.c,
 *          command_handler.c, error.c di-link; hanya transport yang di-stub.
 *
 * ORACLE: decoder di bawah meniru urutan baca telemetryCommands.ts
 * (faas-configurator) sesuai deskripsi prompt -- diimplementasikan
 * independen dari builder (offset dihitung dari nol per field, bukan
 * memakai ulang konstanta offset builder).
 *
 * Dibuktikan: panjang persis 13/12/4/24 byte; nilai ter-decode benar;
 * request_id=0x00 untuk push; konversi satuan (mm/s->cm/s, V/A->mV/mA);
 * saturasi (arus negatif, tegangan >65.535 V, NaN) tanpa UB; pemetaan
 * fix type eksplisit; IMU_RAW hanya membalas saat diminta (nol frame
 * dari PushAll) dengan request_id di-echo; sumber NULL/invalid -> nol.
 */
#include "test_common.h"
#include "telemetry.h"
#include "command_handler.h"
#include "navigation.h"
#include <string.h>

/* ---- stub transport --------------------------------------------------- */
typedef struct { uint16_t cmd; uint8_t req; uint8_t len; uint8_t p[32]; } Tx_t;
static Tx_t s_tx[8];
static int  s_tx_n;
static protocol_frame_handler_t s_frame_handler;

void Protocol_SetFrameHandler(protocol_frame_handler_t h) { s_frame_handler = h; }
int Protocol_SendFrame(uint16_t cmd, uint8_t req, const uint8_t *p, uint8_t len)
{
    if (s_tx_n < 8) {
        s_tx[s_tx_n].cmd = cmd; s_tx[s_tx_n].req = req; s_tx[s_tx_n].len = len;
        memcpy(s_tx[s_tx_n].p, p, len < 32 ? len : 32);
        s_tx_n++;
    }
    return 1;
}
/* Sama seperti protocol.c: thin wrapper request_id = 0x00. */
int Protocol_SendUnsolicited(uint16_t cmd, const uint8_t *p, uint8_t len)
{
    return Protocol_SendFrame(cmd, PROTOCOL_REQUEST_ID_UNSOLICITED, p, len);
}
int Armed_IsArmed(void) { return 0; }

/* ---- oracle decoder --------------------------------------------------- */
static float   rf(const uint8_t *b) { float v; memcpy(&v, b, 4); return v; }
static int32_t ri(const uint8_t *b) { int32_t v; memcpy(&v, b, 4); return v; }
static uint16_t ru(const uint8_t *b) { return (uint16_t)(b[0] | (b[1] << 8)); }   /* LE eksplisit */
static int16_t  rs(const uint8_t *b) { return (int16_t)(b[0] | (b[1] << 8)); }

static TelemetrySnapshot_t base_snap(void)
{
    TelemetrySnapshot_t s;
    memset(&s, 0, sizeof(s));
    s.roll_deg = 12.5f; s.pitch_deg = -3.25f; s.yaw_deg = 271.0f; s.active_imu = 1;
    s.lat_e7 = -60000000; s.lon_e7 = 1068000000;       /* -6.0, 106.8 (Jakarta) */
    s.ground_speed_mm_s = 12345;                        /* 12.345 m/s */
    s.nav_fix_type = NAV_GPS_FIX_3D; s.sat_count = 11;
    s.voltage_volts = 12.6f; s.current_amps = 8.4f;
    return s;
}

int main(void)
{
    /* --- 0. Ukuran payload (dihitung ulang manual di prompt) ---------- */
    CHECK(TELEM_ATTITUDE_LEN == 13);
    CHECK(TELEM_GPS_LEN == 12);
    CHECK(TELEM_BATTERY_LEN == 4);
    CHECK(TELEM_IMU_RAW_LEN == 24);

    /* --- 1. PushAll: tepat 3 frame unsolicited, urutan & ukuran benar --- */
    CommandHandler_Init();
    static TelemetryImuRaw_t raw;
    Telemetry_RegisterCommands(&raw);

    TelemetrySnapshot_t s = base_snap();
    s_tx_n = 0;
    Telemetry_PushAll(&s);
    CHECK(s_tx_n == 3);                                  /* IMU_RAW TIDAK ikut push */
    CHECK(s_tx[0].cmd == CMD_ATTITUDE && s_tx[0].len == 13);
    CHECK(s_tx[1].cmd == CMD_GPS_DATA && s_tx[1].len == 12);
    CHECK(s_tx[2].cmd == CMD_BATTERY  && s_tx[2].len == 4);
    for (int i = 0; i < 3; i++) CHECK(s_tx[i].req == 0x00);   /* unsolicited */

    /* decode ATTITUDE */
    CHECK_NEAR(rf(&s_tx[0].p[0]), 12.5, 1e-6);
    CHECK_NEAR(rf(&s_tx[0].p[4]), -3.25, 1e-6);
    CHECK_NEAR(rf(&s_tx[0].p[8]), 271.0, 1e-6);
    CHECK(s_tx[0].p[12] == 1);
    /* decode GPS */
    CHECK(ri(&s_tx[1].p[0]) == -60000000);
    CHECK(ri(&s_tx[1].p[4]) == 1068000000);
    CHECK(ru(&s_tx[1].p[8]) == 1234);                    /* 12345 mm/s -> 1234 cm/s (integer) */
    CHECK(s_tx[1].p[10] == 2);                           /* 3D */
    CHECK(s_tx[1].p[11] == 11);
    /* decode BATTERY */
    CHECK(ru(&s_tx[2].p[0]) == 12600);
    CHECK(ru(&s_tx[2].p[2]) == 8400);

    /* --- 2. Pemetaan fix type eksplisit (kontrak web: no-fix,2d,3d) ---- */
    CHECK(Telemetry_GpsFixToWire(NAV_GPS_FIX_NONE) == 0);
    CHECK(Telemetry_GpsFixToWire(NAV_GPS_FIX_2D)   == 1);
    CHECK(Telemetry_GpsFixToWire(NAV_GPS_FIX_3D)   == 2);
    CHECK(Telemetry_GpsFixToWire(99) == 0);              /* nilai liar -> no-fix, bukan lolos */
    CHECK(Telemetry_GpsFixToWire(-1) == 0);

    /* --- 3. Saturasi / UB-safety --------------------------------------- */
    CHECK(Telemetry_SatU16(-0.5f) == 0);
    CHECK(Telemetry_SatU16(0.0f) == 0);
    CHECK(Telemetry_SatU16((float)NAN) == 0);
    CHECK(Telemetry_SatU16(1.4f) == 1);
    CHECK(Telemetry_SatU16(1.5f) == 2);                  /* dibulatkan, bukan dipotong */
    CHECK(Telemetry_SatU16(65534.6f) == 65535);
    CHECK(Telemetry_SatU16(1e9f) == 65535);
    CHECK(Telemetry_SatU16(INFINITY) == 65535);
    {
        TelemetrySnapshot_t t = base_snap();
        uint8_t b[4];
        t.current_amps = -0.37f;                          /* offset bias current sense */
        t.voltage_volts = 80.0f;                          /* > 65.535 V */
        Telemetry_BuildBattery(b, &t);
        CHECK(ru(&b[0]) == 65535);
        CHECK(ru(&b[2]) == 0);

        uint8_t g[12];
        t.ground_speed_mm_s = -500;   Telemetry_BuildGps(g, &t);  CHECK(ru(&g[8]) == 0);
        t.ground_speed_mm_s = 2000000000; Telemetry_BuildGps(g, &t); CHECK(ru(&g[8]) == 65535);
        t.ground_speed_mm_s = 0;      Telemetry_BuildGps(g, &t);  CHECK(ru(&g[8]) == 0);
    }

    /* --- 4. CMD_IMU_RAW on-demand: hanya membalas saat diminta ---------- */
    for (int i = 0; i < 6; i++) { raw.primary[i] = (int16_t)(-16384 + i * 1000); raw.secondary[i] = 7; }
    raw.primary[2] = 16384; raw.primary[5] = -32768;
    raw.primary_valid = true; raw.secondary_valid = false;   /* kondisi firmware saat ini */

    s_tx_n = 0;
    Telemetry_PushAll(&s);
    for (int i = 0; i < s_tx_n; i++) CHECK(s_tx[i].cmd != CMD_IMU_RAW);   /* tidak pernah otomatis */

    protocol_frame_t f;
    memset(&f, 0, sizeof(f));
    f.command_id = CMD_IMU_RAW; f.request_id = 0x5A; f.payload_len = 0;
    s_tx_n = 0;
    s_frame_handler(&f);
    CHECK(s_tx_n == 1);
    CHECK(s_tx[0].cmd == CMD_IMU_RAW && s_tx[0].req == 0x5A && s_tx[0].len == 24);
    CHECK(rs(&s_tx[0].p[0]) == -16384);
    CHECK(rs(&s_tx[0].p[4]) == 16384);                    /* accelZ */
    CHECK(rs(&s_tx[0].p[10]) == -32768);                  /* gyroZ primary */
    for (int i = 12; i < 24; i++) CHECK(s_tx[0].p[i] == 0);   /* secondary invalid -> nol */

    raw.secondary_valid = true;
    s_tx_n = 0; s_frame_handler(&f);
    CHECK(rs(&s_tx[0].p[12]) == 7 && rs(&s_tx[0].p[22]) == 7);

    raw.primary_valid = false;                            /* baca primer gagal -> nol, bukan nilai basi */
    s_tx_n = 0; s_frame_handler(&f);
    for (int i = 0; i < 12; i++) CHECK(s_tx[0].p[i] == 0);

    /* --- 5. Sumber NULL tidak crash ------------------------------------- */
    CommandHandler_Init();
    Telemetry_RegisterCommands(NULL);
    s_tx_n = 0; s_frame_handler(&f);
    CHECK(s_tx_n == 1 && s_tx[0].len == 24);
    for (int i = 0; i < 24; i++) CHECK(s_tx[0].p[i] == 0);

    /* --- 6. Sapuan acak: round-trip lewat oracle ------------------------ */
    srand(4242u);
    for (int i = 0; i < 300; i++) {
        TelemetrySnapshot_t t;
        memset(&t, 0, sizeof(t));
        t.roll_deg = (float)(rand() % 36000) / 100.0f - 180.0f;
        t.pitch_deg = (float)(rand() % 18000) / 100.0f - 90.0f;
        t.yaw_deg = (float)(rand() % 72000) / 100.0f - 360.0f;
        t.active_imu = (uint8_t)(rand() & 1);
        t.lat_e7 = (int32_t)(rand() % 1800000000) - 900000000;
        t.lon_e7 = (int32_t)(rand() % 2000000000) - 1000000000;
        t.ground_speed_mm_s = rand() % 60000;
        t.nav_fix_type = rand() % 3; t.sat_count = (uint8_t)(rand() % 30);
        t.voltage_volts = (float)(rand() % 3000) / 100.0f;
        t.current_amps = (float)(rand() % 6000) / 100.0f;   /* 0..60 A, di bawah batas u16 mA (65.535 A) */
        s_tx_n = 0; Telemetry_PushAll(&t);
        CHECK(s_tx_n == 3);
        CHECK_NEAR(rf(&s_tx[0].p[0]), t.roll_deg, 1e-6);
        CHECK(ri(&s_tx[1].p[0]) == t.lat_e7 && ri(&s_tx[1].p[4]) == t.lon_e7);
        CHECK(ru(&s_tx[1].p[8]) == (uint16_t)(t.ground_speed_mm_s / 10));
        CHECK(s_tx[1].p[10] == (uint8_t)t.nav_fix_type);     /* enum sejajar wire utk 0..2 */
        CHECK_NEAR(ru(&s_tx[2].p[0]), t.voltage_volts * 1000.0, 0.51);
        CHECK_NEAR(ru(&s_tx[2].p[2]), t.current_amps * 1000.0, 0.51);
    }

    TEST_SUMMARY("telemetry");
}
