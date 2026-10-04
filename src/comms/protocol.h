#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "stm32f4xx.h"
#include <stdint.h>

/* ============================================================================
 * protocol.h
 * Implementasi kontrak wire-format FC<->web sesuai protocol.md (draft v1).
 * Satu-satunya modul firmware yang boleh tahu detail byte-level framing --
 * command_handler.c/h dan modul lain HANYA bicara lewat protocol_frame_t
 * dan Protocol_SendFrame()/Protocol_SendUnsolicited(), tidak pernah
 * menyentuh preamble/length/crc secara langsung (protocol.md Bagian 1).
 *
 * Transport: USB CDC (usb_cdc_if.h) ATAU USART6 command link cadangan
 * (bsp_uart.h, untuk adapter USB-to-TTL -- lihat catatan dual-transport
 * di protocol.c). Protocol_Poll() dipanggil dari main loop / scheduler
 * tiap tick, menarik byte yang tersedia dari ring buffer transport yang
 * aktif dan memberi makan parser; balasan otomatis lewat jalur yang sama
 * dengan request-nya.
 * ============================================================================ */

/* ---------------------------------------------------------------------------
 * Format frame (protocol.md Bagian 2)
 * [0] 0xFA  [1] 0xFC  [2] length  [3] cmd_lo [4] cmd_hi [5] request_id
 * [6..] payload (length byte)  [6+length] crc8
 * ------------------------------------------------------------------------- */
#define FRAME_PREAMBLE_1        0xFAu
#define FRAME_PREAMBLE_2        0xFCu
#define PROTOCOL_MAX_PAYLOAD    255u
/* Total frame maksimum: 2 preamble + 1 length + 2 cmd + 1 request_id +
 * payload + 1 crc = 7 + PROTOCOL_MAX_PAYLOAD */
#define PROTOCOL_MAX_FRAME_SIZE (7u + PROTOCOL_MAX_PAYLOAD)

/* request_id = 0x00 berarti frame unsolicited (push status dari FC),
 * bukan balasan atas request tertentu (protocol.md Bagian 3). */
#define PROTOCOL_REQUEST_ID_UNSOLICITED   0x00u

/* ---------------------------------------------------------------------------
 * Registry Command ID (protocol.md Bagian 5)
 * ------------------------------------------------------------------------- */
#define CMD_GET_STATUS                 0x0001u

#define CMD_ATTITUDE                   0x0101u
#define CMD_GPS_DATA                   0x0102u
#define CMD_BATTERY                    0x0103u
#define CMD_IMU_RAW                    0x0104u

#define CMD_SETTING_SCHEMA_LIST        0x0201u
#define CMD_SETTING_GET                0x0202u
#define CMD_SETTING_SET                0x0203u
#define CMD_SETTING_COMMIT             0x0204u

#define CMD_MOTOR_TEST                 0x0301u  /* armed-gated */
#define CMD_SERVO_TEST                 0x0302u  /* armed-gated (keputusan tim, lihat pembagian-tugas Bag.2 #2) */

#define CMD_CALIB_ACCEL_GYRO_START     0x0401u
#define CMD_CALIB_ACCEL_GYRO_STOP      0x0402u
#define CMD_CALIB_MAG_START            0x0403u
#define CMD_CALIB_MAG_STOP             0x0404u
#define CMD_CALIB_STATUS               0x0405u

#define CMD_MISSION_UPLOAD             0x0501u
#define CMD_HOME_SET                   0x0502u
#define CMD_RTH_TRIGGER                0x0503u

#define CMD_REBOOT_DFU                 0x0601u  /* armed-gated */
#define CMD_FLASH_HASH                 0x0602u

#define CMD_ERROR                      0xFFFFu

/* ---------------------------------------------------------------------------
 * Tipe frame terparsing -- dipakai command_handler.c untuk dispatch,
 * TIDAK pernah dipakai untuk menyusun ulang byte mentah (itu tugas
 * Protocol_SendFrame()).
 * ------------------------------------------------------------------------- */
typedef struct {
    uint16_t command_id;
    uint8_t  request_id;
    uint8_t  payload[PROTOCOL_MAX_PAYLOAD];
    uint8_t  payload_len;
} protocol_frame_t;

/**
 * @brief Callback dipanggil Protocol_Poll() setiap kali satu frame valid
 *        (preamble cocok, panjang konsisten, CRC8 cocok) selesai diterima.
 *        `frame` hanya valid selama durasi callback -- command_handler.c
 *        yang perlu menyimpan data lebih lama harus menyalin sendiri.
 */
typedef void (*protocol_frame_handler_t)(const protocol_frame_t *frame);

/**
 * @brief Init parser (reset state machine ke kondisi awal). Panggil
 *        sekali saat boot, setelah BSP_USB_CDC_Init().
 */
void Protocol_Init(void);

/**
 * @brief Daftarkan fungsi yang dipanggil tiap frame valid selesai
 *        diterima. Biasanya diisi command_handler.c dengan fungsi
 *        dispatch-nya (mis. `CommandHandler_Dispatch`). Memanggil ini
 *        lagi akan menimpa handler sebelumnya (hanya satu handler aktif).
 */
void Protocol_SetFrameHandler(protocol_frame_handler_t handler);

/**
 * @brief Tarik semua byte yang tersedia dari BSP_USB_CDC_Available()/
 *        ReadByte(), beri makan parser byte demi byte. Setiap frame valid
 *        yang selesai diparse langsung memicu handler terdaftar (lihat
 *        Protocol_SetFrameHandler()) sebelum lanjut ke byte berikutnya.
 *        Non-blocking -- return begitu ring buffer USB CDC kosong.
 *        Dipanggil dari main loop / scheduler tiap tick.
 */
void Protocol_Poll(void);

/**
 * @brief Hitung CRC8 DVB-S2 (poly 0xD5, no reflect, initial value
 *        eksplisit lewat parameter) atas buffer `data` sepanjang `len`
 *        byte. Fungsi murni tanpa state, sesuai protocol.md Bagian 4.
 *        Dipakai internal oleh parser & sender, tapi diexpose juga
 *        supaya blackbox/flash_w25q64.c (Orang 4) bisa reuse untuk
 *        format record blackbox (lihat pembagian-tugas Bag.2 #4).
 * @param data buffer input
 * @param len jumlah byte
 * @param initial nilai awal CRC (protocol.md pakai 0x00 untuk frame)
 * @return nilai CRC8 hasil
 */
uint8_t Protocol_CRC8_DVBS2(const uint8_t *data, uint16_t len, uint8_t initial);

/**
 * @brief Susun & kirim satu frame lengkap (preamble+length+cmd+request_id
 *        +payload+crc8) lewat BSP_USB_CDC_WriteBuf(). Dipakai untuk
 *        balasan atas request (request_id di-echo persis dari frame
 *        yang diterima) MAUPUN untuk mengirim frame baru.
 * @param command_id ID command (lihat daftar CMD_* di atas)
 * @param request_id di-echo dari request (atau PROTOCOL_REQUEST_ID_UNSOLICITED)
 * @param payload buffer payload, boleh NULL kalau payload_len==0
 * @param payload_len panjang payload (0-255)
 * @return 1 kalau berhasil dikirim, 0 kalau gagal (mis. USB belum
 *         configured, atau payload_len melebihi PROTOCOL_MAX_PAYLOAD --
 *         caller bertanggung jawab tidak melebihi batas ini, fungsi
 *         akan menolak & return 0 kalau tetap terjadi)
 */
int Protocol_SendFrame(uint16_t command_id, uint8_t request_id,
                        const uint8_t *payload, uint8_t payload_len);

/**
 * @brief Helper tipis di atas Protocol_SendFrame() dengan
 *        request_id = PROTOCOL_REQUEST_ID_UNSOLICITED (0x00) -- dipakai
 *        untuk push status yang bukan balasan atas request tertentu
 *        (protocol.md Bagian 3).
 */
int Protocol_SendUnsolicited(uint16_t command_id, const uint8_t *payload, uint8_t payload_len);

#endif /* PROTOCOL_H */
