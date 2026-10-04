/**
 * @file    test_actuator_test.c
 * @brief   Test end-to-end CMD_MOTOR_TEST / CMD_SERVO_TEST (Comms #2).
 *
 * BEDA dari test_get_status.c / test_rth_bank.c (yang menyalin fungsi
 * static): di sini yang di-link adalah kode ASLI --
 *   src/output/output_map.c   (handler + OutputMap_GetSlotByIndex)
 *   src/comms/command_handler.c (dispatch() + s_armed_gated_commands[])
 *   src/system/error.c        (Error_Send -> CMD_ERROR)
 * Yang di-stub hanya batas hardware/transport: DShot_SetThrottle(),
 * PwmServo_SetPosition(), Protocol_SendFrame(), Protocol_SetFrameHandler()
 * (handler disimpan, lalu test memanggil dispatch() lewat pointer itu,
 * persis seperti Protocol_Poll() sungguhan), plus definisi kuat
 * Armed_IsArmed() yang meng-override weak default di command_handler.c.
 *
 * Bisa dikompilasi native karena Makefile memberi CMSIS include path +
 * -DSTM32F411xE (protocol.h meng-include stm32f4xx.h); tidak ada
 * register/peripheral yang benar-benar dipakai kode yang diuji.
 *
 * Yang dibuktikan:
 *   - disarmed: handler jalan, driver terpanggil dengan driver_index
 *     & nilai (ter-clamp) yang benar, respons status=0, request_id di-echo.
 *   - armed: dispatch() menolak dengan CMD_ERROR/ERROR_ARMED_REJECTED,
 *     handler TIDAK pernah dipanggil (nol panggilan driver, nol respons
 *     ber-command-id test) -- beda jalur dari status=1 milik handler.
 *   - index di luar jangkauan / role tidak cocok / payload pendek /
 *     map kosong / map NULL: status=1, nol panggilan driver, tanpa crash.
 */
#include "test_common.h"
#include "output_map.h"
#include "command_handler.h"
#include "error.h"
#include <string.h>

/* ---- stub driver: catat panggilan ------------------------------------ */
static int   s_dshot_calls, s_servo_calls;
static uint8_t s_dshot_idx, s_servo_idx;
static float s_dshot_val, s_servo_val;

void DShot_SetThrottle(uint8_t motor_index, float throttle_0_1)
{
    s_dshot_calls++; s_dshot_idx = motor_index; s_dshot_val = throttle_0_1;
}
void PwmServo_SetPosition(uint8_t servo_index, float position)
{
    s_servo_calls++; s_servo_idx = servo_index; s_servo_val = position;
}

/* ---- stub transport: tangkap frame keluar & simpan frame handler ------ */
static protocol_frame_handler_t s_frame_handler;
static int      s_tx_calls;
static uint16_t s_tx_cmd;
static uint8_t  s_tx_req, s_tx_len, s_tx_payload[8];

void Protocol_SetFrameHandler(protocol_frame_handler_t handler) { s_frame_handler = handler; }

int Protocol_SendFrame(uint16_t command_id, uint8_t request_id,
                       const uint8_t *payload, uint8_t payload_len)
{
    s_tx_calls++;
    s_tx_cmd = command_id; s_tx_req = request_id; s_tx_len = payload_len;
    if (payload_len <= sizeof(s_tx_payload)) memcpy(s_tx_payload, payload, payload_len);
    return 1;
}

/* ---- override weak hook (definisi kuat menang di linker) -------------- */
static int s_armed;
int Armed_IsArmed(void) { return s_armed; }

/* ---- helper ------------------------------------------------------------ */
static void reset_probes(void)
{
    s_dshot_calls = s_servo_calls = s_tx_calls = 0;
    s_tx_cmd = 0; s_tx_len = 0; s_tx_req = 0;
}

/* Kirim frame request ke dispatch() seperti Protocol_Poll() sungguhan. */
static void send_test(uint16_t cmd, uint8_t req, uint8_t idx, float value, uint8_t payload_len)
{
    protocol_frame_t f;
    memset(&f, 0, sizeof(f));
    f.command_id = cmd;
    f.request_id = req;
    f.payload[0] = idx;
    memcpy(&f.payload[1], &value, sizeof(float));
    f.payload_len = payload_len;
    reset_probes();
    s_frame_handler(&f);
}

static void expect_status(uint16_t cmd, uint8_t req, uint8_t status)
{
    CHECK(s_tx_calls == 1);
    CHECK(s_tx_cmd == cmd);             /* respons bertipe command test, BUKAN CMD_ERROR */
    CHECK(s_tx_req == req);             /* request_id di-echo */
    CHECK(s_tx_len == 1);
    CHECK(s_tx_payload[0] == status);
}

static void expect_no_driver(void) { CHECK(s_dshot_calls == 0 && s_servo_calls == 0); }

static void fresh_boot(OutputMap_t *map)
{
    s_frame_handler = 0;
    CommandHandler_Init();              /* reset tabel + pasang dispatch */
    OutputMap_InitDefault(map);
    OutputMap_RegisterCommands(map);    /* urutan sama seperti main() */
    CHECK(s_frame_handler != 0);
}

int main(void)
{
    OutputMap_t map;
    fresh_boot(&map);
    s_armed = 0;

    /* --- 1. OutputMap_GetSlotByIndex(): urutan pendaftaran --------------- */
    {
        const OutputMapSlot_t *s0 = OutputMap_GetSlotByIndex(&map, 0);
        const OutputMapSlot_t *s4 = OutputMap_GetSlotByIndex(&map, 4);
        CHECK(s0 != NULL && s0->role == OUTPUT_ROLE_MOTOR && s0->driver_index == 0);
        CHECK(s4 != NULL && s4->role == OUTPUT_ROLE_SERVO && s4->driver_index == 3);
        CHECK(OutputMap_GetSlotByIndex(&map, 5) == NULL);
        CHECK(OutputMap_GetSlotByIndex(&map, 255) == NULL);
        CHECK(OutputMap_GetSlotByIndex(NULL, 0) == NULL);
    }

    /* --- 2. Disarmed: motor index 0 bergerak, status=0 ------------------ */
    send_test(CMD_MOTOR_TEST, 0x11, 0, 0.5f, 5);
    expect_status(CMD_MOTOR_TEST, 0x11, 0);
    CHECK(s_dshot_calls == 1 && s_dshot_idx == 0 && s_servo_calls == 0);
    CHECK_NEAR(s_dshot_val, 0.5, 1e-6);

    /* clamp motor ke [0,1] */
    send_test(CMD_MOTOR_TEST, 0x12, 0, 3.0f, 5);   CHECK_NEAR(s_dshot_val, 1.0, 1e-6);
    send_test(CMD_MOTOR_TEST, 0x13, 0, -2.0f, 5);  CHECK_NEAR(s_dshot_val, 0.0, 1e-6);

    /* --- 3. Servo: index logis 1..4 -> driver_index 0..3, clamp [-1,1] --- */
    for (uint8_t i = 1; i <= 4; i++) {
        send_test(CMD_SERVO_TEST, (uint8_t)(0x20 + i), i, -0.25f, 5);
        expect_status(CMD_SERVO_TEST, (uint8_t)(0x20 + i), 0);
        CHECK(s_servo_calls == 1 && s_servo_idx == (uint8_t)(i - 1) && s_dshot_calls == 0);
        CHECK_NEAR(s_servo_val, -0.25, 1e-6);
    }
    send_test(CMD_SERVO_TEST, 0x30, 2, 9.0f, 5);   CHECK_NEAR(s_servo_val, 1.0, 1e-6);
    send_test(CMD_SERVO_TEST, 0x31, 2, -9.0f, 5);  CHECK_NEAR(s_servo_val, -1.0, 1e-6);

    /* --- 4. Armed: dicegat dispatch(), handler TIDAK dipanggil ----------- */
    s_armed = 1;
    for (int k = 0; k < 2; k++) {
        uint16_t cmd = k ? CMD_SERVO_TEST : CMD_MOTOR_TEST;
        send_test(cmd, 0x41, k ? 1 : 0, 0.7f, 5);
        CHECK(s_tx_calls == 1);
        CHECK(s_tx_cmd == CMD_ERROR);                       /* jalur dispatch, bukan status=1 handler */
        CHECK(s_tx_req == 0x41);
        CHECK(s_tx_len == 3);
        CHECK(s_tx_payload[0] == (uint8_t)ERROR_ARMED_REJECTED);
        CHECK(s_tx_payload[1] == (uint8_t)(cmd & 0xFFu));
        CHECK(s_tx_payload[2] == (uint8_t)(cmd >> 8));
        expect_no_driver();
    }
    s_armed = 0;

    /* --- 5. Gagal via handler: status=1, nol panggilan driver ------------ */
    send_test(CMD_MOTOR_TEST, 0x51, 1, 0.5f, 5);   expect_status(CMD_MOTOR_TEST, 0x51, 1); expect_no_driver(); /* idx 1 = servo */
    send_test(CMD_SERVO_TEST, 0x52, 0, 0.5f, 5);   expect_status(CMD_SERVO_TEST, 0x52, 1); expect_no_driver(); /* idx 0 = motor */
    send_test(CMD_MOTOR_TEST, 0x53, 5, 0.5f, 5);   expect_status(CMD_MOTOR_TEST, 0x53, 1); expect_no_driver(); /* di luar jangkauan */
    send_test(CMD_SERVO_TEST, 0x54, 255, 0.5f, 5); expect_status(CMD_SERVO_TEST, 0x54, 1); expect_no_driver();
    send_test(CMD_MOTOR_TEST, 0x55, 0, 0.5f, 4);   expect_status(CMD_MOTOR_TEST, 0x55, 1); expect_no_driver(); /* payload pendek */
    send_test(CMD_MOTOR_TEST, 0x56, 0, 0.5f, 0);   expect_status(CMD_MOTOR_TEST, 0x56, 1); expect_no_driver();

    /* NaN dari wire tidak boleh lolos ke driver (clamp biasa tidak menangkap NaN) */
    send_test(CMD_MOTOR_TEST, 0x57, 0, (float)NAN, 5);   expect_status(CMD_MOTOR_TEST, 0x57, 1); expect_no_driver();
    send_test(CMD_SERVO_TEST, 0x58, 1, (float)NAN, 5);   expect_status(CMD_SERVO_TEST, 0x58, 1); expect_no_driver();

    /* --- 6. Urutan boot salah: map belum di-InitDefault / NULL ----------- */
    {
        OutputMap_t empty;
        memset(&empty, 0, sizeof(empty));           /* slot_count = 0, seperti .bss */
        s_frame_handler = 0;
        CommandHandler_Init();
        OutputMap_RegisterCommands(&empty);
        send_test(CMD_MOTOR_TEST, 0x61, 0, 0.5f, 5);
        expect_status(CMD_MOTOR_TEST, 0x61, 1); expect_no_driver();

        s_frame_handler = 0;
        CommandHandler_Init();
        OutputMap_RegisterCommands(NULL);
        send_test(CMD_SERVO_TEST, 0x62, 1, 0.5f, 5);
        expect_status(CMD_SERVO_TEST, 0x62, 1); expect_no_driver();
    }

    /* --- 7. Boot ulang normal setelah skenario salah tetap berfungsi ------ */
    fresh_boot(&map);
    send_test(CMD_MOTOR_TEST, 0x71, 0, 0.1f, 5);
    expect_status(CMD_MOTOR_TEST, 0x71, 0);

    TEST_SUMMARY("actuator_test");
}
