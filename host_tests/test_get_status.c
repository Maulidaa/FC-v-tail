/**
 * @file    test_get_status.c
 * @brief   Host test untuk handle_get_status() (src/comms/command_handler.c).
 *
 * handle_get_status() adalah fungsi `static`, dan translation unit-nya
 * (command_handler.c -> command_handler.h -> protocol.h) meng-include
 * "protocol.h" lewat quote-include DARI DALAM src/comms/ -- artinya
 * pencarian file akan selalu menemukan src/comms/protocol.h ASLI
 * (yang menyeret stm32f4xx.h/CMSIS) lebih dulu, TERLEPAS dari urutan
 * `-I` di Makefile (searah-direktori-sendiri selalu menang untuk quote-
 * include, beda dari kasus settings.h/compass.c yang bisa dioverride
 * lewat -I. karena src/fusion/ tidak punya settings.h sendiri). Jadi
 * command_handler.c TIDAK bisa dikompilasi native di host tanpa
 * menyediakan stm32f4xx.h sungguhan + define chip target.
 *
 * Pola yang dipakai di sini SAMA seperti test_rth_bank.c untuk
 * rth_bank_from_heading() di main.c: fungsi disalin verbatim, plus
 * "stand-in" lokal untuk setiap dependency eksternalnya (hook
 * Armed_IsArmed()/Battery_GetVoltageMillivolts()/dst, dan
 * Protocol_SendFrame() -- di sini versi capturing, bukan no-op, supaya
 * payload yang dihasilkan bisa diperiksa).
 *
 * SALINAN PERSIS dari handle_get_status() + makro PROTOCOL_VERSION/
 * FW_VERSION_* di src/comms/command_handler.c. WAJIB disinkronkan
 * manual kalau fungsi/nilai aslinya berubah.
 *
 * ORACLE: get_status_decode_like_web() di bawah meng-implementasi ULANG
 * (independen, bukan menyalin logika encode di atas) urutan baca yang
 * dipakai `getStatusCommand.decodeResponse()` di
 * faas-configurator/src/core/commands/statusCommands.ts (sesuai
 * deskripsi di prompt task Comms #1):
 *   armed: r.u8() !== 0
 *   firmwareVersion: r.lstring()   -- u8 length-prefix + string
 *   protocolVersion: r.u8()
 *   batteryVoltage: r.f32()
 *   gpsFixType: r.u8()
 * ASUMSI (tidak eksplisit di prompt, implementasi r.f32() tidak
 * terlihat): little-endian, konsisten dengan protocol.md Bagian 2
 * ("Byte order: little-endian untuk semua field multi-byte") yang
 * jadi acuan seluruh wire format di proyek ini, dan dengan target
 * Cortex-M4F (little-endian) yang memakai memcpy langsung tanpa
 * byte-swap. Kalau asumsi ini salah, test ini perlu direvisi bersamaan
 * dengan konfirmasi ulang ke sisi web.
 */
#include "test_common.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* --- Stand-in struct frame minimal -- hanya field yang benar-benar
 * dipakai handle_get_status() (request_id), bukan salinan penuh
 * protocol_frame_t (yang juga punya command_id/payload/payload_len,
 * tidak relevan di sini). --------------------------------------------- */
typedef struct {
    uint8_t request_id;
} test_frame_t;

/* --- Stand-in hook (pengganti Armed_IsArmed()/dkk. dari
 * command_handler.c) -- dikendalikan test lewat variabel global,
 * bukan weak-link sungguhan (tidak relevan di host test). ------------- */
static int      s_armed        = 0;
static uint16_t s_battery_mv   = 0;
static uint8_t  s_gps_fix_type = 0;

static int      Armed_IsArmed(void)                     { return s_armed; }
static uint16_t Battery_GetVoltageMillivolts(void)       { return s_battery_mv; }
static uint8_t  GPS_GetFixType(void)                     { return s_gps_fix_type; }

/* --- Stand-in Protocol_SendFrame() -- menangkap payload alih-alih
 * mengirim ke USB CDC sungguhan. --------------------------------------- */
#define CMD_GET_STATUS_LOCAL 0x0001u
static uint16_t s_sent_command_id;
static uint8_t  s_sent_request_id;
static uint8_t  s_sent_payload[64];
static uint8_t  s_sent_payload_len;
static int      s_send_call_count;

static int Protocol_SendFrame(uint16_t command_id, uint8_t request_id,
                              const uint8_t *payload, uint8_t payload_len)
{
    s_send_call_count++;
    s_sent_command_id  = command_id;
    s_sent_request_id  = request_id;
    s_sent_payload_len = payload_len;
    if (payload != NULL && payload_len > 0) {
        memcpy(s_sent_payload, payload, payload_len);
    }
    return 1;
}

/* ===================================================================== *
 * SALINAN PERSIS dari command_handler.c (versi SESUDAH fix Comms #1)
 * ===================================================================== */
#define PROTOCOL_VERSION   1u
#define FW_VERSION_MAJOR   0u
#define FW_VERSION_MINOR   1u
#define FW_VERSION_PATCH   0u

static void handle_get_status(const test_frame_t *frame)
{
    uint16_t battery_mv    = Battery_GetVoltageMillivolts();
    float    battery_volts = (float)battery_mv / 1000.0f;

    char fw_str[16];
    int  fw_len = snprintf(fw_str, sizeof(fw_str), "%u.%u.%u",
                            FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH);
    if (fw_len < 0) {
        fw_len = 0;
    }
    if ((size_t)fw_len > sizeof(fw_str) - 1) {
        fw_len = (int)sizeof(fw_str) - 1;
    }

    uint8_t payload[1 + 1 + 16 + 1 + 4 + 1];
    size_t  idx = 0;
    payload[idx++] = (uint8_t)Armed_IsArmed();
    payload[idx++] = (uint8_t)fw_len;
    for (int i = 0; i < fw_len; i++) {
        payload[idx++] = (uint8_t)fw_str[i];
    }
    payload[idx++] = PROTOCOL_VERSION;
    memcpy(&payload[idx], &battery_volts, sizeof(float));
    idx += sizeof(float);
    payload[idx++] = GPS_GetFixType();

    Protocol_SendFrame(CMD_GET_STATUS_LOCAL, frame->request_id, payload, (uint8_t)idx);
}

/* ===================================================================== *
 * ORACLE -- oracle independen meniru statusCommands.ts::decodeResponse()
 * ===================================================================== */
typedef struct {
    bool    armed;
    char    firmware_version[16];
    uint8_t protocol_version;
    float   battery_voltage;
    uint8_t gps_fix_type;
    size_t  bytes_consumed;   /* untuk memastikan tidak ada byte nyasar/kurang */
    bool    ok;               /* false kalau buffer terlalu pendek untuk field wajib */
} DecodedStatus_t;

static DecodedStatus_t get_status_decode_like_web(const uint8_t *buf, size_t len)
{
    DecodedStatus_t out;
    memset(&out, 0, sizeof(out));

    size_t pos = 0;
    if (pos + 1 > len) { out.ok = false; return out; }
    out.armed = (buf[pos] != 0);              /* r.u8() !== 0 */
    pos += 1;

    if (pos + 1 > len) { out.ok = false; return out; }
    uint8_t fw_len = buf[pos];                /* r.lstring(): length prefix */
    pos += 1;
    if (pos + fw_len > len || fw_len >= sizeof(out.firmware_version)) {
        out.ok = false;
        return out;
    }
    memcpy(out.firmware_version, &buf[pos], fw_len);
    out.firmware_version[fw_len] = '\0';
    pos += fw_len;

    if (pos + 1 > len) { out.ok = false; return out; }
    out.protocol_version = buf[pos];          /* r.u8() */
    pos += 1;

    if (pos + 4 > len) { out.ok = false; return out; }
    memcpy(&out.battery_voltage, &buf[pos], sizeof(float));  /* r.f32() LE */
    pos += 4;

    if (pos + 1 > len) { out.ok = false; return out; }
    out.gps_fix_type = buf[pos];              /* r.u8() */
    pos += 1;

    out.bytes_consumed = pos;
    out.ok = true;
    return out;
}

/* Bangun frame contoh, panggil handle_get_status(), decode lewat oracle,
 * lalu bandingkan ke nilai input asli -- ini yang membuktikan payload
 * "bisa di-decode langsung oleh getStatusCommand.decodeResponse() tanpa
 * error dan menghasilkan nilai yang benar" (kriteria selesai task). */
static void run_case(int armed, uint16_t battery_mv, uint8_t gps_fix_type,
                     uint8_t request_id)
{
    s_armed        = armed;
    s_battery_mv   = battery_mv;
    s_gps_fix_type = gps_fix_type;
    s_send_call_count = 0;

    test_frame_t frame = { .request_id = request_id };
    handle_get_status(&frame);

    CHECK(s_send_call_count == 1);
    CHECK(s_sent_command_id == CMD_GET_STATUS_LOCAL);
    CHECK(s_sent_request_id == request_id);   /* request_id wajib di-echo, protocol.md Bag.3 */

    DecodedStatus_t d = get_status_decode_like_web(s_sent_payload, s_sent_payload_len);
    CHECK(d.ok);
    if (!d.ok) return;

    /* Tidak ada byte nyasar/kurang -- oracle habis persis di ujung
     * payload yang dikirim, bukan berhenti lebih awal atau butuh lebih. */
    CHECK(d.bytes_consumed == s_sent_payload_len);

    CHECK(d.armed == (armed != 0));
    CHECK(strcmp(d.firmware_version, "0.1.0") == 0);
    CHECK(d.protocol_version == PROTOCOL_VERSION);
    CHECK_NEAR(d.battery_voltage, (double)battery_mv / 1000.0, 1e-4);
    CHECK(d.gps_fix_type == gps_fix_type);
}

int main(void)
{
    /* --- 1. Kasus dasar: armed=0, baterai contoh, kombinasi request_id - */
    run_case(0, 12600, 3, 0x07);   /* 12.6V, fix type 3, request_id acak */

    /* --- 2. armed=1 --------------------------------------------------- */
    run_case(1, 11100, 0, 0x01);   /* 11.1V (mendekati batas low-voltage), no fix */

    /* --- 3. Batas bawah: baterai 0 (belum terbaca / belum ada sensor) - */
    run_case(0, 0, 0, 0x00);       /* request_id=0x00 (unsolicited-style, tetap harus di-echo apa adanya) */

    /* --- 4. Batas atas u16: battery_mv = 65535 (num maksimum tipe lama,
     * memastikan konversi ke volt tidak overflow/salah bulat) --------- */
    run_case(1, 65535, 255, 0xFF);

    /* --- 5. Verifikasi byte-per-byte manual untuk SATU contoh (bukan
     * cuma lewat oracle) -- menagih layout persis yang didokumentasikan
     * di command_handler.c, termasuk offset float. ---------------------- */
    {
        s_armed = 1; s_battery_mv = 12000; s_gps_fix_type = 2;
        test_frame_t frame = { .request_id = 0x2A };
        handle_get_status(&frame);

        /* "0.1.0" -> fw_len=5 -> total = 1+1+5+1+4+1 = 13 byte */
        CHECK(s_sent_payload_len == 13);
        CHECK(s_sent_payload[0] == 1);            /* armed */
        CHECK(s_sent_payload[1] == 5);             /* fw_len */
        CHECK(memcmp(&s_sent_payload[2], "0.1.0", 5) == 0);
        CHECK(s_sent_payload[7] == PROTOCOL_VERSION);
        float expected_volts = 12.0f;
        CHECK(memcmp(&s_sent_payload[8], &expected_volts, sizeof(float)) == 0);
        CHECK(s_sent_payload[12] == 2);            /* gps_fix_type */
    }

    /* --- 6. Sapuan acak: banyak kombinasi armed/battery/gps_fix, semua
     * harus lolos decode oracle dan bernilai benar (bukti tambahan di
     * atas beberapa kasus manual, bukan pengganti kasus 1-5). --------- */
    {
        srand(777u);
        int checked = 0;
        for (int i = 0; i < 300; i++) {
            int      armed    = rand() % 2;
            uint16_t batt_mv  = (uint16_t)(rand() % 30000);
            uint8_t  gps_fix  = (uint8_t)(rand() % 6);
            uint8_t  req_id   = (uint8_t)(rand() % 256);
            run_case(armed, batt_mv, gps_fix, req_id);
            checked++;
        }
        printf("  sapuan acak: %d kombinasi status diverifikasi lewat oracle decode\n", checked);
    }

    TEST_SUMMARY("get_status");
}
