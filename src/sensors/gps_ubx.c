/**
 * gps_ubx.c
 *
 * Lihat gps_ubx.h. Target: u-blox NEO-6M (tanpa NAV-PVT).
 * Checksum: Fletcher-8 atas class, id, length, payload.
 */

#include "gps_ubx.h"
#include <string.h>

#define UBX_SYNC1          (0xB5u)
#define UBX_SYNC2          (0x62u)

#define UBX_CLASS_NAV      (0x01u)
#define UBX_ID_NAV_POSLLH  (0x02u)
#define UBX_ID_NAV_SOL     (0x06u)
#define UBX_ID_NAV_VELNED  (0x12u)
#define NAV_POSLLH_LEN     (28u)
#define NAV_SOL_LEN        (52u)
#define NAV_VELNED_LEN     (36u)

#define UBX_CLASS_CFG      (0x06u)
#define UBX_ID_CFG_PRT     (0x00u)
#define UBX_ID_CFG_MSG     (0x01u)
#define UBX_ID_CFG_RATE    (0x08u)
#define UBX_ID_CFG_NAV5    (0x24u)

#define PAYLOAD_BUF_MAX    (64u)  /* > message terbesar yang di-parse (52) */

#define PART_POSLLH  (0x01u)
#define PART_VELNED  (0x02u)
#define PART_SOL     (0x04u)
#define PART_ALL     (PART_POSLLH | PART_VELNED | PART_SOL)

typedef enum {
    ST_WAIT_SYNC1 = 0,
    ST_WAIT_SYNC2,
    ST_CLASS,
    ST_ID,
    ST_LEN_LO,
    ST_LEN_HI,
    ST_PAYLOAD,
    ST_CK_A,
    ST_CK_B
} ParserState_t;

static ParserState_t s_state;
static uint8_t  s_msg_class;
static uint8_t  s_msg_id;
static uint16_t s_payload_len;
static uint16_t s_payload_idx;
static uint8_t  s_payload[PAYLOAD_BUF_MAX];
static uint8_t  s_ck_a, s_ck_b;
static uint8_t  s_rx_ck_a, s_rx_ck_b;

/* Epoch yang sedang dirakit dari tiga message */
static GPS_Data_t s_pending;
static uint8_t    s_pending_mask;
static uint32_t   s_pending_itow;

/* Epoch lengkap terakhir */
static GPS_Data_t s_last_fix;
static bool       s_has_data;

static uint8_t s_cfg_step;

static void checksum_update(uint8_t byte)
{
    s_ck_a = (uint8_t)(s_ck_a + byte);
    s_ck_b = (uint8_t)(s_ck_b + s_ck_a);
}

/* Little-endian helpers (byte order native UBX) */
static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static int32_t rd_i32(const uint8_t *p) { return (int32_t)rd_u32(p); }
static uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

void GPS_UBX_Init(void)
{
    s_state = ST_WAIT_SYNC1;
    s_payload_idx = 0;
    s_pending_mask = 0;
    s_has_data = false;
    memset(&s_pending, 0, sizeof(s_pending));
    memset(&s_last_fix, 0, sizeof(s_last_fix));
    s_cfg_step = 0;
}

/* ------------------------------------------------------------------ */
/* Perakitan epoch                                                     */
/* ------------------------------------------------------------------ */

/* Dipanggil SEBELUM menulis field: kalau iTOW beda dari epoch yang
 * sedang dirakit, buang rakitan lama (sebagian message hilang). */
static void begin_part(uint32_t itow)
{
    if (s_pending_mask != 0u && itow != s_pending_itow) {
        s_pending_mask = 0u;
    }
    if (s_pending_mask == 0u) {
        s_pending_itow = itow;
        s_pending.itow_ms = itow;
    }
}

/* Return true kalau ketiga bagian sudah lengkap dan di-commit. */
static bool end_part(uint8_t part)
{
    s_pending_mask |= part;
    if (s_pending_mask == PART_ALL) {
        s_last_fix = s_pending;
        s_has_data = true;
        s_pending_mask = 0u;
        return true;
    }
    return false;
}

static bool parse_posllh(const uint8_t *p)
{
    begin_part(rd_u32(&p[0]));
    s_pending.lon_e7        = rd_i32(&p[4]);
    s_pending.lat_e7        = rd_i32(&p[8]);
    s_pending.height_mm     = rd_i32(&p[12]);
    s_pending.height_msl_mm = rd_i32(&p[16]);
    s_pending.h_acc_mm      = rd_u32(&p[20]);
    return end_part(PART_POSLLH);
}

static bool parse_velned(const uint8_t *p)
{
    begin_part(rd_u32(&p[0]));
    /* gSpeed di UBX dalam cm/s -> mm/s */
    s_pending.ground_speed_mm_s = (int32_t)(rd_u32(&p[20]) * 10u);
    s_pending.heading_e5        = rd_i32(&p[24]);
    return end_part(PART_VELNED);
}

static bool parse_sol(const uint8_t *p)
{
    uint8_t fix = p[10];

    begin_part(rd_u32(&p[0]));
    s_pending.fix_type  = (fix <= (uint8_t)GPS_FIX_TIME_ONLY)
                          ? (GPS_FixType_t)fix : GPS_FIX_NONE;
    s_pending.fix_ok    = ((p[11] & 0x01u) != 0u);   /* gpsFixOK */
    s_pending.pdop_x100 = rd_u16(&p[44]);
    s_pending.num_sv    = p[47];
    return end_part(PART_SOL);
}

static bool finalize_frame(void)
{
    if (s_rx_ck_a != s_ck_a || s_rx_ck_b != s_ck_b) {
        return false;
    }
    if (s_msg_class != UBX_CLASS_NAV) {
        return false;   /* mis. ACK/NAK (class 0x05): valid, diabaikan */
    }

    if (s_msg_id == UBX_ID_NAV_POSLLH && s_payload_len == NAV_POSLLH_LEN) {
        return parse_posllh(s_payload);
    }
    if (s_msg_id == UBX_ID_NAV_VELNED && s_payload_len == NAV_VELNED_LEN) {
        return parse_velned(s_payload);
    }
    if (s_msg_id == UBX_ID_NAV_SOL && s_payload_len == NAV_SOL_LEN) {
        return parse_sol(s_payload);
    }
    return false;
}

bool GPS_UBX_ProcessByte(uint8_t byte)
{
    bool frame_ready = false;

    switch (s_state) {
        case ST_WAIT_SYNC1:
            if (byte == UBX_SYNC1) {
                s_state = ST_WAIT_SYNC2;
            }
            break;

        case ST_WAIT_SYNC2:
            if (byte == UBX_SYNC2) {
                s_ck_a = 0;
                s_ck_b = 0;
                s_state = ST_CLASS;
            } else if (byte != UBX_SYNC1) {
                /* B5 B5 62: tetap di sini, byte ini adalah SYNC1 baru */
                s_state = ST_WAIT_SYNC1;
            }
            break;

        case ST_CLASS:
            s_msg_class = byte;
            checksum_update(byte);
            s_state = ST_ID;
            break;

        case ST_ID:
            s_msg_id = byte;
            checksum_update(byte);
            s_state = ST_LEN_LO;
            break;

        case ST_LEN_LO:
            s_payload_len = byte;
            checksum_update(byte);
            s_state = ST_LEN_HI;
            break;

        case ST_LEN_HI:
            s_payload_len |= (uint16_t)((uint16_t)byte << 8);
            checksum_update(byte);
            s_payload_idx = 0;
            if (s_payload_len > PAYLOAD_BUF_MAX) {
                s_state = ST_WAIT_SYNC1;   /* terlalu besar / korup: skip */
            } else if (s_payload_len == 0u) {
                s_state = ST_CK_A;
            } else {
                s_state = ST_PAYLOAD;
            }
            break;

        case ST_PAYLOAD:
            s_payload[s_payload_idx++] = byte;
            checksum_update(byte);
            if (s_payload_idx >= s_payload_len) {
                s_state = ST_CK_A;
            }
            break;

        case ST_CK_A:
            s_rx_ck_a = byte;
            s_state = ST_CK_B;
            break;

        case ST_CK_B:
            s_rx_ck_b = byte;
            frame_ready = finalize_frame();
            s_state = ST_WAIT_SYNC1;
            break;

        default:
            s_state = ST_WAIT_SYNC1;
            break;
    }

    return frame_ready;
}

bool GPS_UBX_GetData(GPS_Data_t *out)
{
    if (!s_has_data || out == NULL) {
        return false;
    }
    *out = s_last_fix;
    return true;
}

bool GPS_UBX_HasFix(void)
{
    return s_has_data && s_last_fix.fix_ok
        && (s_last_fix.fix_type == GPS_FIX_2D
         || s_last_fix.fix_type == GPS_FIX_3D
         || s_last_fix.fix_type == GPS_FIX_GNSS_DR);
}

/* ------------------------------------------------------------------ */
/* Konfigurasi modul                                                   */
/* ------------------------------------------------------------------ */

/* Bangun frame UBX (sync + header + payload + checksum) lalu kirim.
 * Checksum dihitung di sini supaya tidak ada konstanta hex tulis-tangan. */
static void ubx_send(GPS_UBX_TxFn tx, uint8_t cls, uint8_t id,
                     const uint8_t *payload, uint16_t len)
{
    uint8_t frame[8u + 40u];   /* 8 byte overhead + payload terbesar (36) */
    uint8_t a = 0, b = 0;
    uint16_t i;

    if (tx == NULL || len > 40u) {
        return;
    }

    frame[0] = UBX_SYNC1;
    frame[1] = UBX_SYNC2;
    frame[2] = cls;
    frame[3] = id;
    frame[4] = (uint8_t)(len & 0xFFu);
    frame[5] = (uint8_t)(len >> 8);
    for (i = 0; i < len; i++) {
        frame[6u + i] = payload[i];
    }
    for (i = 2; i < (uint16_t)(6u + len); i++) {
        a = (uint8_t)(a + frame[i]);
        b = (uint8_t)(b + a);
    }
    frame[6u + len] = a;
    frame[7u + len] = b;

    tx(frame, (uint16_t)(8u + len));
}

static void send_cfg_msg(GPS_UBX_TxFn tx, uint8_t cls, uint8_t id)
{
    /* rate per port: [DDC, UART1, UART2, USB, SPI, reserved]; 1 = tiap epoch.
     * Modul terhubung lewat UART1-nya. */
    uint8_t p[8] = { cls, id, 0, 1, 0, 0, 0, 0 };
    ubx_send(tx, UBX_CLASS_CFG, UBX_ID_CFG_MSG, p, 8);
}

bool GPS_UBX_ConfigStep(GPS_UBX_TxFn tx)
{
    switch (s_cfg_step) {
        case 0: {   /* CFG-RATE: measRate ms, navRate 1, timeRef GPS */
            uint8_t p[6] = {
                (uint8_t)(GPS_UBX_MEAS_RATE_MS & 0xFFu),
                (uint8_t)(GPS_UBX_MEAS_RATE_MS >> 8),
                1, 0, 1, 0
            };
            ubx_send(tx, UBX_CLASS_CFG, UBX_ID_CFG_RATE, p, 6);
            break;
        }
        case 1: {   /* CFG-NAV5: hanya set dynModel (mask bit0) */
            uint8_t p[36];
            memset(p, 0, sizeof(p));
            p[0] = 0x01;                        /* mask: dyn */
            p[2] = (uint8_t)GPS_UBX_DYN_MODEL;
            ubx_send(tx, UBX_CLASS_CFG, UBX_ID_CFG_NAV5, p, 36);
            break;
        }
        case 2: send_cfg_msg(tx, UBX_CLASS_NAV, UBX_ID_NAV_POSLLH); break;
        case 3: send_cfg_msg(tx, UBX_CLASS_NAV, UBX_ID_NAV_VELNED); break;
        case 4: send_cfg_msg(tx, UBX_CLASS_NAV, UBX_ID_NAV_SOL);    break;
        case 5: {   /* CFG-PRT UART1: 8N1, 9600, in=UBX+NMEA, out=UBX saja */
            uint8_t p[20];
            memset(p, 0, sizeof(p));
            p[0]  = 1;                          /* portID = UART1 */
            p[4]  = 0xD0; p[5] = 0x08;          /* mode 0x000008D0 = 8N1 */
            p[8]  = 0x80; p[9] = 0x25;          /* baudRate 9600 */
            p[12] = 0x07;                       /* inProtoMask: UBX|NMEA|RTCM */
            p[14] = 0x01;                       /* outProtoMask: UBX saja */
            ubx_send(tx, UBX_CLASS_CFG, UBX_ID_CFG_PRT, p, 20);
            break;
        }
        default:
            return true;
    }

    s_cfg_step++;
    return (s_cfg_step > 5u);
}

void GPS_UBX_ConfigRestart(void)
{
    s_cfg_step = 0;
}

bool GPS_UBX_ConfigDone(void)
{
    return (s_cfg_step > 5u);
}
