#include "protocol.h"
#include "usb_cdc_if.h"
#include "bsp_uart.h"

/* ============================================================================
 * protocol.c
 * Implementasi framing sesuai protocol.md. Parser adalah state machine
 * byte-demi-byte (bukan buffer-then-parse) supaya Protocol_Poll() bisa
 * dipanggil murni non-blocking dari main loop/scheduler tanpa perlu
 * menyimpan buffer mentah terpisah dari struct frame hasil parse.
 *
 * DUA TRANSPORT: parser ini sekarang menerima byte dari USB CDC ATAU
 * USART6 (command link cadangan untuk adapter USB-to-TTL, lihat
 * bsp_uart.h/bsp_pinmap.h) -- device bisa disambung lewat salah satu,
 * tidak perlu tahu di sisi web mana yang dipakai. Supaya byte dari kedua
 * sumber tidak pernah tercampur di tengah satu frame, transport "dikunci"
 * begitu preamble pertama cocok (s_state keluar dari PARSE_WAIT_PREAMBLE1)
 * dan dilepas lagi begitu parser kembali ke PARSE_WAIT_PREAMBLE1 (frame
 * selesai/gagal) -- lihat Protocol_Poll(). Balasan (Protocol_SendFrame
 * dengan request_id != UNSOLICITED) dikirim ke transport yang sama dengan
 * request-nya; frame unsolicited (push status) di-broadcast ke keduanya
 * karena tidak terikat ke satu jalur.
 * ============================================================================ */

typedef enum {
    PARSE_WAIT_PREAMBLE1 = 0,
    PARSE_WAIT_PREAMBLE2,
    PARSE_WAIT_LENGTH,
    PARSE_WAIT_CMD_LO,
    PARSE_WAIT_CMD_HI,
    PARSE_WAIT_REQUEST_ID,
    PARSE_READ_PAYLOAD,
    PARSE_WAIT_CRC
} parser_state_t;

static parser_state_t         s_state;
static protocol_frame_t       s_frame_in_progress;
static uint8_t                s_cmd_lo;
static uint8_t                s_payload_bytes_read;
static uint8_t                s_crc_running;
static protocol_frame_handler_t s_frame_handler = 0;

/* ---------------------------------------------------------------------------
 * Transport ganda: USB CDC + USART6 (lihat catatan header di atas)
 * ------------------------------------------------------------------------- */
typedef enum {
    XPORT_USB = 0,
    XPORT_UART6
} xport_t;

/* Transport pemilik frame yang sedang diparse; XPORT_USB dipakai juga
 * sebagai nilai "belum ada lock" default -- lihat pengecekan
 * s_state == PARSE_WAIT_PREAMBLE1 di Protocol_Poll(), bukan nilai ini,
 * jadi default-nya tidak signifikan selain sebagai nilai awal yang valid. */
static xport_t s_locked_transport = XPORT_USB;
static int     s_is_locked        = 0;

/* Transport asal frame yang barusan selesai diparse -- dipakai
 * Protocol_SendFrame() untuk membalas ke jalur yang sama. Default XPORT_USB
 * supaya frame unsolicited yang dikirim sebelum ada request apa pun (mis.
 * pesan boot) tetap konsisten dengan perilaku lama (transport tunggal). */
static xport_t s_reply_transport  = XPORT_USB;

/* ---------------------------------------------------------------------------
 * CRC8 DVB-S2: poly 0xD5, MSB-first, tanpa reflect input/output.
 * ------------------------------------------------------------------------- */
static uint8_t crc8_update_byte(uint8_t crc, uint8_t data)
{
    crc ^= data;
    for (int bit = 0; bit < 8; bit++) {
        if (crc & 0x80u) {
            crc = (uint8_t)((crc << 1) ^ 0xD5u);
        } else {
            crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

uint8_t Protocol_CRC8_DVBS2(const uint8_t *data, uint16_t len, uint8_t initial)
{
    uint8_t crc = initial;
    for (uint16_t i = 0; i < len; i++) {
        crc = crc8_update_byte(crc, data[i]);
    }
    return crc;
}

/* ---------------------------------------------------------------------------
 * Parser -- satu byte per panggilan, memicu s_frame_handler saat frame
 * valid (CRC cocok) selesai.
 * ------------------------------------------------------------------------- */
static void parser_reset(void)
{
    s_state = PARSE_WAIT_PREAMBLE1;
}

static void parse_byte(uint8_t byte)
{
    switch (s_state) {

    case PARSE_WAIT_PREAMBLE1:
        if (byte == FRAME_PREAMBLE_1) {
            s_state = PARSE_WAIT_PREAMBLE2;
        }
        /* selain itu: buang byte, tetap di WAIT_PREAMBLE1 (resync) */
        break;

    case PARSE_WAIT_PREAMBLE2:
        if (byte == FRAME_PREAMBLE_2) {
            s_crc_running = 0x00u; /* initial value CRC per protocol.md Bag.4 */
            s_state = PARSE_WAIT_LENGTH;
        } else if (byte == FRAME_PREAMBLE_1) {
            /* mis. 0xFA 0xFA 0xFC -- byte ini kandidat preamble1 baru,
             * tetap di state ini menunggu 0xFC berikutnya */
        } else {
            s_state = PARSE_WAIT_PREAMBLE1;
        }
        break;

    case PARSE_WAIT_LENGTH:
        s_frame_in_progress.payload_len = byte;
        s_crc_running = crc8_update_byte(s_crc_running, byte);
        s_state = PARSE_WAIT_CMD_LO;
        break;

    case PARSE_WAIT_CMD_LO:
        s_cmd_lo = byte;
        s_crc_running = crc8_update_byte(s_crc_running, byte);
        s_state = PARSE_WAIT_CMD_HI;
        break;

    case PARSE_WAIT_CMD_HI:
        s_frame_in_progress.command_id = (uint16_t)s_cmd_lo | ((uint16_t)byte << 8);
        s_crc_running = crc8_update_byte(s_crc_running, byte);
        s_state = PARSE_WAIT_REQUEST_ID;
        break;

    case PARSE_WAIT_REQUEST_ID:
        s_frame_in_progress.request_id = byte;
        s_crc_running = crc8_update_byte(s_crc_running, byte);
        s_payload_bytes_read = 0;
        if (s_frame_in_progress.payload_len == 0u) {
            s_state = PARSE_WAIT_CRC;
        } else {
            s_state = PARSE_READ_PAYLOAD;
        }
        break;

    case PARSE_READ_PAYLOAD:
        s_frame_in_progress.payload[s_payload_bytes_read] = byte;
        s_crc_running = crc8_update_byte(s_crc_running, byte);
        s_payload_bytes_read++;
        if (s_payload_bytes_read >= s_frame_in_progress.payload_len) {
            s_state = PARSE_WAIT_CRC;
        }
        break;

    case PARSE_WAIT_CRC:
        if (byte == s_crc_running) {
            /* Frame valid -- panggil handler terdaftar (biasanya
             * command_handler.c). request_id di frame ini sudah pasti
             * valid (sudah lolos CRC), jadi caller yang perlu membalas
             * CMD_ERROR untuk kasus lain (command tidak dikenal, dsb)
             * bisa langsung echo request_id ini. */
            if (s_frame_handler != 0) {
                s_frame_handler(&s_frame_in_progress);
            }
        }
        /* CRC mismatch: sengaja TIDAK mengirim CMD_ERROR dari sini --
         * byte CRC yang salah bisa jadi bukan CRC sungguhan (mis. garis
         * data ikut ter-skip alignment), jadi request_id yang sudah
         * terbaca belum tentu bisa dipercaya sebagai milik frame yang
         * sama. Caller di sisi web akan retry lewat mekanisme timeout
         * normal (protocol.md Bagian 3), bukan menunggu CMD_ERROR
         * eksplisit untuk kasus corrupt-in-transit seperti ini. */
        s_state = PARSE_WAIT_PREAMBLE1;
        break;

    default:
        s_state = PARSE_WAIT_PREAMBLE1;
        break;
    }
}

/* ---------------------------------------------------------------------------
 * API publik
 * ------------------------------------------------------------------------- */

void Protocol_Init(void)
{
    parser_reset();
    s_frame_handler = 0;
    s_is_locked = 0;
    s_reply_transport = XPORT_USB;
}

void Protocol_SetFrameHandler(protocol_frame_handler_t handler)
{
    s_frame_handler = handler;
}

void Protocol_Poll(void)
{
    uint8_t byte;

    for (;;) {
        xport_t use;
        int have_byte = 0;

        if (s_is_locked) {
            /* Mid-frame: HANYA ambil dari transport yang mengunci frame
             * ini, supaya byte dari sumber lain tidak pernah tercampur di
             * tengah satu frame. Kalau ring buffer transport ini kosong
             * untuk saat ini, berhenti dan tunggu tick berikutnya --
             * jangan pindah ke transport lain di tengah frame. */
            use = s_locked_transport;
            if (use == XPORT_USB) {
                have_byte = (BSP_USB_CDC_Available() > 0u) && BSP_USB_CDC_ReadByte(&byte);
            } else {
                have_byte = (BSP_UART6_Available() > 0u) && BSP_UART6_ReadByte(&byte);
            }
            if (!have_byte) {
                break;
            }
        } else if (BSP_USB_CDC_Available() > 0u) {
            use = XPORT_USB;
            have_byte = BSP_USB_CDC_ReadByte(&byte);
        } else if (BSP_UART6_Available() > 0u) {
            use = XPORT_UART6;
            have_byte = BSP_UART6_ReadByte(&byte);
        } else {
            break; /* tidak ada byte baru di kedua transport */
        }

        if (!have_byte) {
            break; /* race jarang: Available()>0 tapi Read gagal -- berhenti aman */
        }

        s_reply_transport = use;
        parse_byte(byte);

        /* Lock/unlock berdasarkan state SETELAH byte ini diproses:
         * kembali ke PARSE_WAIT_PREAMBLE1 berarti frame selesai (valid
         * atau gagal) -- lepas lock, transport berikutnya bebas dipilih
         * lagi. Selain itu berarti masih mid-frame -- kunci ke transport
         * byte ini. */
        if (s_state == PARSE_WAIT_PREAMBLE1) {
            s_is_locked = 0;
        } else {
            s_is_locked = 1;
            s_locked_transport = use;
        }
    }
}

int Protocol_SendFrame(uint16_t command_id, uint8_t request_id,
                        const uint8_t *payload, uint8_t payload_len)
{
    if (payload_len > 0u && payload == 0) {
        return 0; /* payload_len>0 tapi buffer NULL -- caller salah pakai API */
    }

    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    uint16_t idx = 0;

    buf[idx++] = FRAME_PREAMBLE_1;
    buf[idx++] = FRAME_PREAMBLE_2;
    buf[idx++] = payload_len;
    buf[idx++] = (uint8_t)(command_id & 0xFFu);
    buf[idx++] = (uint8_t)((command_id >> 8) & 0xFFu);
    buf[idx++] = request_id;

    for (uint8_t i = 0; i < payload_len; i++) {
        buf[idx++] = payload[i];
    }

    /* Domain CRC: [length, cmd_lo, cmd_hi, request_id, payload...] --
     * yaitu buf[2..idx-1], TIDAK termasuk 2 byte preamble di buf[0..1]. */
    uint8_t crc = Protocol_CRC8_DVBS2(&buf[2], (uint16_t)(idx - 2u), 0x00u);
    buf[idx++] = crc;

    if (request_id == PROTOCOL_REQUEST_ID_UNSOLICITED) {
        /* Push status tidak terikat ke satu jalur -- broadcast ke
         * keduanya. USART6 selalu "berhasil" (TX blocking ke pin fisik,
         * tidak ada status enumerasi seperti USB), jadi keberhasilan
         * keseluruhan tetap ditentukan status USB CDC seperti sebelumnya. */
        BSP_UART6_WriteBuf(buf, idx);
        return BSP_USB_CDC_WriteBuf(buf, idx);
    }

    /* Balasan atas request tertentu -- kirim ke transport asal request
     * itu saja (di-set Protocol_Poll() tepat sebelum byte CRC frame ini
     * diproses, jadi masih valid selama kita di call stack yang sama). */
    if (s_reply_transport == XPORT_UART6) {
        BSP_UART6_WriteBuf(buf, idx);
        return 1;
    }
    return BSP_USB_CDC_WriteBuf(buf, idx);
}

int Protocol_SendUnsolicited(uint16_t command_id, const uint8_t *payload, uint8_t payload_len)
{
    return Protocol_SendFrame(command_id, PROTOCOL_REQUEST_ID_UNSOLICITED, payload, payload_len);
}
