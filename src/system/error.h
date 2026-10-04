#ifndef ERROR_H
#define ERROR_H

#include "stm32f4xx.h"
#include <stdint.h>

/* ============================================================================
 * error.h
 * Pembungkus tipis di atas Protocol_SendFrame() untuk mengirim frame
 * CMD_ERROR (0xFFFF, protocol.md Bagian 5 & 9 -- lihat CATATAN di bawah).
 *
 * CATATAN TERBUKA -- perlu dikonfirmasi ke tim / disinkronkan ke sisi web:
 *   Tabel command registry protocol.md Bagian 5 menulis "CMD_ERROR ...
 *   lihat Bagian 9", tapi isi Bagian 9 di draft protocol.md yang ada
 *   sekarang adalah skema setting `mixer` (bukan definisi payload
 *   CMD_ERROR) -- kemungkinan referensi bagian ini basi dari revisi
 *   dokumen sebelumnya. Payload CMD_ERROR BELUM didefinisikan eksplisit
 *   di draft manapun yang saya lihat.
 *
 *   Format di bawah ini (error_code + command ID yang gagal) adalah
 *   PROPOSAL modul ini, bukan kontrak yang sudah disepakati tim --
 *   sebelum `core/protocol/frame.ts` sisi web mengasumsikan bentuk ini,
 *   perlu dikonfirmasi dulu (sama urgency-nya dengan item terbuka lain
 *   di protocol.md Bagian 12). Kalau tim sudah sepakat bentuk lain,
 *   HANYA payload_len & isi payload di error.c yang perlu berubah --
 *   protocol.c dan command_handler.c tidak perlu tahu detail ini.
 *
 * Payload (proposal): [error_code: u8][orig_command_id_lo: u8][orig_command_id_hi: u8]
 * request_id di frame CMD_ERROR WAJIB di-echo dari request yang gagal
 * (protocol.md Bagian 3) -- ditangani oleh caller Error_Send(), bukan
 * modul ini (modul ini cuma builder payload + pemanggil Protocol_SendFrame).
 * ============================================================================ */

typedef enum {
    ERROR_UNKNOWN_COMMAND         = 1, /* command_id tidak ada di registry */
    ERROR_INVALID_PAYLOAD_LENGTH  = 2, /* payload_len tidak sesuai command_id ybs */
    ERROR_ARMED_REJECTED          = 3, /* command armed-gated ditolak karena armed=true */
    ERROR_SETTING_KEY_NOT_FOUND   = 4, /* SETTING_GET/SET dengan key yang tidak ada di schema */
    ERROR_SETTING_VALUE_OUT_OF_RANGE = 5, /* SETTING_SET melanggar min/max/step schema */
    ERROR_BUSY                    = 6, /* mis. kalibrasi lain sedang berjalan, flash sedang ditulis dsb */
    ERROR_INTERNAL                = 7, /* kegagalan internal tak terduga (HW fault, dsb) */
} error_code_t;

/**
 * @brief Kirim frame CMD_ERROR (0xFFFF) dengan request_id di-echo dari
 *        request yang gagal, error_code sesuai kejadian, dan
 *        original_command_id supaya web tahu request mana yang ditolak
 *        (berguna terutama kalau beberapa request sempat in-flight).
 * @param request_id di-echo PERSIS dari frame request yang menyebabkan
 *        error ini (protocol.md Bagian 3) -- JANGAN PROTOCOL_REQUEST_ID_UNSOLICITED
 *        kecuali error memang tidak terkait request manapun.
 * @param code kode error (lihat error_code_t)
 * @param original_command_id command_id dari request yang gagal (0x0000
 *        kalau tidak relevan/tidak diketahui, mis. error terjadi sebelum
 *        command_id sempat terbaca -- kasus ini seharusnya jarang karena
 *        protocol.c hanya memanggil handler setelah frame lolos CRC)
 * @return 1 kalau berhasil dikirim, 0 kalau gagal (lihat Protocol_SendFrame)
 */
int Error_Send(uint8_t request_id, error_code_t code, uint16_t original_command_id);

#endif /* ERROR_H */
