/**
 * @file    test_calib_commands.c
 * @brief   Test end-to-end CMD_CALIB_* (Comms #3), kode ASLI di-link:
 *            src/calibration/calib_dispatcher.c (handler + dispatcher)
 *            src/calibration/calib_accel_gyro.c, calib_mag.c
 *            src/comms/command_handler.c (dispatch), src/system/error.c
 *          Di-stub hanya transport (Protocol_SendFrame/SetFrameHandler),
 *          Armed_IsArmed() dan CalibRecover_IsActive() (aslinya milik
 *          main.c -- di sini dikendalikan test). Pola sama seperti
 *          test_actuator_test.c.
 *
 * Yang dibuktikan: status idle sebelum kalibrasi apa pun (bukan
 * CMD_ERROR); mag idle->in_progress->done lewat sample figure-8 mock;
 * accel/gyro idem; START bentrok -> CMD_ERROR/ERROR_BUSY; START/STOP
 * accel/gyro saat recovery aktif ditolak DAN state singleton
 * calib_accel_gyro.c TIDAK berubah (bukan cuma "balasannya error");
 * command CALIB tidak armed-gated (mendokumentasikan kontrak saat ini).
 */
#include "test_common.h"
#include "calib_dispatcher.h"
#include "calib_accel_gyro.h"
#include "calib_mag.h"
#include "command_handler.h"
#include "error.h"
#include <string.h>

static protocol_frame_handler_t s_frame_handler;
static int      s_tx_calls;
static uint16_t s_tx_cmd;
static uint8_t  s_tx_req, s_tx_len, s_tx_payload[8];

void Protocol_SetFrameHandler(protocol_frame_handler_t h) { s_frame_handler = h; }
int Protocol_SendFrame(uint16_t cmd, uint8_t req, const uint8_t *p, uint8_t len)
{
    s_tx_calls++; s_tx_cmd = cmd; s_tx_req = req; s_tx_len = len;
    if (len && len <= sizeof(s_tx_payload)) memcpy(s_tx_payload, p, len);
    return 1;
}

static int s_armed, s_recover_active;
int  Armed_IsArmed(void)       { return s_armed; }
bool CalibRecover_IsActive(void) { return s_recover_active != 0; }

static void send_cmd(uint16_t cmd, uint8_t req)
{
    protocol_frame_t f;
    memset(&f, 0, sizeof(f));
    f.command_id = cmd; f.request_id = req; f.payload_len = 0;
    s_tx_calls = 0; s_tx_len = 0; s_tx_cmd = 0;
    s_frame_handler(&f);
}

static void expect_ack(uint16_t cmd, uint8_t req)
{
    CHECK(s_tx_calls == 1);
    CHECK(s_tx_cmd == cmd);
    CHECK(s_tx_req == req);
    CHECK(s_tx_len == 0);
}

static void expect_busy(uint16_t cmd, uint8_t req)
{
    CHECK(s_tx_calls == 1);
    CHECK(s_tx_cmd == CMD_ERROR);
    CHECK(s_tx_req == req);
    CHECK(s_tx_len == 3);
    CHECK(s_tx_payload[0] == (uint8_t)ERROR_BUSY);
    CHECK(s_tx_payload[1] == (uint8_t)(cmd & 0xFFu));
    CHECK(s_tx_payload[2] == (uint8_t)(cmd >> 8));
}

static void expect_status(uint8_t req, uint8_t type, uint8_t state, uint8_t prog)
{
    send_cmd(CMD_CALIB_STATUS, req);
    CHECK(s_tx_calls == 1);
    CHECK(s_tx_cmd == CMD_CALIB_STATUS);         /* bukan CMD_ERROR */
    CHECK(s_tx_req == req);
    CHECK(s_tx_len == 3);
    CHECK(s_tx_payload[0] == type);
    CHECK(s_tx_payload[1] == state);
    CHECK(s_tx_payload[2] == prog);
}

/* Mock figure-8: satu sample per oktan (tanda x,y,z), magnitudo di atas
 * ambang MIN_MAGNITUDE_SQ. Kembalikan progress% terakhir yang dilaporkan. */
static void feed_octant(int o)
{
    int16_t x = (o & 1) ? 300 : -300, y = (o & 2) ? 250 : -250, z = (o & 4) ? 200 : -200;
    CalibDispatcher_FeedMagSample(x, y, z);
}

int main(void)
{
    CommandHandler_Init();
    CalibDispatcher_RegisterCommands();
    CHECK(s_frame_handler != 0);

    /* 1. Status sebelum kalibrasi apa pun: IDLE eksplisit, bukan CMD_ERROR */
    expect_status(0x01, 0, 0, 0);

    /* 2. Mag: idle -> in_progress -> done */
    send_cmd(CMD_CALIB_MAG_START, 0x02);  expect_ack(CMD_CALIB_MAG_START, 0x02);
    expect_status(0x03, CALIB_TYPE_MAG, 1, 0);
    uint8_t last = 0;
    for (int o = 0; o < 7; o++) {
        feed_octant(o);
        send_cmd(CMD_CALIB_STATUS, 0x10);
        CHECK(s_tx_payload[1] == 1);                    /* masih in_progress */
        CHECK(s_tx_payload[2] > last);                  /* progress naik monoton */
        last = s_tx_payload[2];
    }
    feed_octant(7);
    expect_status(0x04, CALIB_TYPE_MAG, 2, 100);        /* done */
    send_cmd(CMD_CALIB_MAG_STOP, 0x05);   expect_ack(CMD_CALIB_MAG_STOP, 0x05);
    expect_status(0x06, CALIB_TYPE_MAG, 0, 0);          /* idle setelah stop */

    /* 3. Tidak armed-gated: START mag tetap diterima saat armed=1
     *    (kontrak protocol.md saat ini; lihat catatan temuan di kode). */
    s_armed = 1;
    send_cmd(CMD_CALIB_MAG_START, 0x07);  expect_ack(CMD_CALIB_MAG_START, 0x07);
    send_cmd(CMD_CALIB_MAG_STOP, 0x08);   expect_ack(CMD_CALIB_MAG_STOP, 0x08);
    s_armed = 0;

    /* 4. Accel/gyro normal (recovery tidak aktif): idle->in_progress->done */
    send_cmd(CMD_CALIB_ACCEL_GYRO_START, 0x11);  expect_ack(CMD_CALIB_ACCEL_GYRO_START, 0x11);
    expect_status(0x12, CALIB_TYPE_ACCEL_GYRO, 1, 0);
    for (int i = 0; i < 250; i++) CalibDispatcher_FeedImuSample(10, -20, 16384, 3, -2, 1);
    expect_status(0x13, CALIB_TYPE_ACCEL_GYRO, 1, 50);
    /* 4b. Mag START saat accel/gyro in_progress -> BUSY, sesi accel/gyro utuh */
    send_cmd(CMD_CALIB_MAG_START, 0x14);  expect_busy(CMD_CALIB_MAG_START, 0x14);
    CHECK(CalibAccelGyro_GetState() == CALIB_AG_STATE_IN_PROGRESS);
    for (int i = 0; i < 250; i++) CalibDispatcher_FeedImuSample(10, -20, 16384, 3, -2, 1);
    expect_status(0x15, CALIB_TYPE_ACCEL_GYRO, 2, 100);
    send_cmd(CMD_CALIB_ACCEL_GYRO_STOP, 0x16); expect_ack(CMD_CALIB_ACCEL_GYRO_STOP, 0x16);

    /* 5. Recovery aktif: START/STOP accel/gyro ditolak, singleton TIDAK berubah.
     *    Sesi "recovery" disimulasikan langsung lewat CalibAccelGyro_* persis
     *    seperti task_calib_recover() (tanpa dispatcher). */
    CalibAccelGyro_Start();
    for (int i = 0; i < 100; i++) CalibAccelGyro_FeedSample(1, 2, 16384, 0, 0, 0);
    CHECK(CalibAccelGyro_GetState() == CALIB_AG_STATE_IN_PROGRESS);
    CHECK(CalibAccelGyro_GetProgressPercent() == 20);
    s_recover_active = 1;

    send_cmd(CMD_CALIB_ACCEL_GYRO_START, 0x21);  expect_busy(CMD_CALIB_ACCEL_GYRO_START, 0x21);
    CHECK(CalibAccelGyro_GetState() == CALIB_AG_STATE_IN_PROGRESS);
    CHECK(CalibAccelGyro_GetProgressPercent() == 20);      /* TIDAK di-reset */

    send_cmd(CMD_CALIB_ACCEL_GYRO_STOP, 0x22);   expect_busy(CMD_CALIB_ACCEL_GYRO_STOP, 0x22);
    CHECK(CalibAccelGyro_GetState() == CALIB_AG_STATE_IN_PROGRESS);   /* TIDAK di-stop */

    /* mag start juga ditolak (aturan dispatcher: satu jenis aktif) */
    send_cmd(CMD_CALIB_MAG_START, 0x23);  expect_busy(CMD_CALIB_MAG_START, 0x23);

    /* Recovery selesai -> operator bebas lagi */
    CalibAccelGyro_Stop();
    s_recover_active = 0;
    send_cmd(CMD_CALIB_ACCEL_GYRO_START, 0x24);  expect_ack(CMD_CALIB_ACCEL_GYRO_START, 0x24);
    send_cmd(CMD_CALIB_ACCEL_GYRO_STOP, 0x25);   expect_ack(CMD_CALIB_ACCEL_GYRO_STOP, 0x25);

    TEST_SUMMARY("calib_commands");
}
