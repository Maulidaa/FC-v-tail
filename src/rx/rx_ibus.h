/**
 * @file    rx_ibus.h
 * @brief   Parser protokol FlySky iBUS (servo/channel frame) untuk channel
 *          input dari radio receiver + deteksi failsafe link-loss.
 *
 * Tanggung jawab: Orang 3 (Fusion, Navigasi & Failsafe)
 *
 * Hardware: USART1_RX (PA10), RX-only -- sama seperti keputusan CRSF
 * sebelumnya (lihat pinout-fc-stm32f411.md & pembagian tugas): iBUS
 * "servo" frame memang satu-arah (receiver -> FC), tidak butuh jalur
 * balik, jadi PA9 (USART1_TX) tetap dibebaskan jadi spare.
 *
 * PENTING (wiring): receiver FlySky yang punya 2 pin iBUS terpisah
 * (mis. FS-iA6B) -- sambungkan HANYA ke pin "iBUS servo" (channel data,
 * satu arah). JANGAN sambung ke pin "iBUS sensor/telemetry" (half-duplex,
 * protokol beda, bukan cakupan modul ini).
 *
 * Modul ini TIDAK menyentuh UART secara langsung -- byte mentah didorong
 * masuk oleh driver bsp_uart.c (Orang 1, lewat interrupt/ring buffer) ke
 * RxIbus_FeedByte()/RxIbus_FeedBuffer(). Pemisahan sama seperti rx_crsf.c
 * sebelumnya.
 *
 * KENAPA TIDAK CUKUP PAKAI DETEKSI TIMEOUT SAJA (beda penting dari CRSF):
 * Banyak receiver FlySky (termasuk FS-iA6B, dikonfirmasi berulang kali di
 * laporan komunitas Betaflight/iNav/IBusBM) TETAP mengirim frame iBUS
 * ber-checksum valid terus-menerus walau link RF ke transmitter sudah
 * putus -- byte di UART tidak pernah berhenti mengalir. Isi channel-nya
 * saja yang berubah (atau malah tidak berubah / "hold last value" kalau
 * failsafe per-channel tidak dikonfigurasi di transmitter). Karena itu
 * modul ini menggabungkan TIGA sinyal failsafe, bukan cuma satu:
 *   1) Timeout murni (elapsed sejak frame valid terakhir) -- menangkap
 *      kasus wire putus / receiver mati total / benar2 tidak ada byte.
 *   2) Value-match -- channel tertentu (biasanya throttle) dibandingkan
 *      ke nilai failsafe yang SUDAH dikonfigurasi manual di menu
 *      transmitter lewat RxIbus_SetFailsafeChannel(). Wajib dikonfigurasi
 *      di radio dulu, modul ini tidak bisa menebak nilainya sendiri.
 *   3) Bit-flag per-channel -- ada laporan komunitas (belum kami
 *      verifikasi langsung lewat capture serial sungguhan) FS-iA6B
 *      menyalakan bit 5 di byte-tinggi nilai channel 16-bit saat channel
 *      itu sedang failsafe. Dicoba dipakai sebagai sinyal tambahan, TAPI
 *      WAJIB dikonfirmasi dulu ke hardware asli sebelum dipercaya penuh
 *      untuk keputusan RTH -- lihat TODO di rx_ibus.c.
 * Ketiganya di-OR: link dianggap LOST kalau salah satu saja aktif.
 */

#ifndef RX_IBUS_H
#define RX_IBUS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------- */
/* Konstanta protokol iBUS (servo/channel frame)                        */
/* ------------------------------------------------------------------- */

#define IBUS_HEADER_BYTE1           0x20U   /* = panjang total frame (32) */
#define IBUS_HEADER_BYTE2           0x40U   /* command: channel data */
#define IBUS_FRAME_LEN              32U     /* fixed, BUKAN variable-length spt CRSF */
#define IBUS_NUM_CHANNELS           14U     /* standar iBUS servo frame */

#define IBUS_CH_VALUE_MIN           1000U   /* konvensi umum iBUS/PWM, sama seperti 1000-2000us */
#define IBUS_CH_VALUE_MID           1500U
#define IBUS_CH_VALUE_MAX           2000U

/* Bit ke-13 (bit 5 dari byte TINGGI nilai channel 16-bit) -- lihat
 * disclaimer "KENAPA TIDAK CUKUP..." di atas. Nilai channel asli (1000-
 * 2000) muat di bit 0-10, jadi bit ini tidak pernah bentrok dengan data
 * channel normal -- aman dipakai sebagai flag terpisah SETELAH mask. */
#define IBUS_CH_FAILSAFE_FLAG_BIT   (1U << 13)
#define IBUS_CH_VALUE_MASK          0x1FFFU  /* bit 0-12, buang flag bit 13 ke atas */

/** Timeout tanpa frame valid sebelum dianggap link-loss (ms). Ini HANYA
 *  lapisan pertama (lihat disclaimer di atas) -- jangan dianggap satu-
 *  satunya mekanisme. */
#define IBUS_FAILSAFE_TIMEOUT_MS    500U

/* ------------------------------------------------------------------- */
/* Tipe data                                                            */
/* ------------------------------------------------------------------- */

typedef enum {
    IBUS_LINK_OK = 0,
    IBUS_LINK_LOST,
} IBUS_LinkStatus_t;

/** Hasil parsing channel, dipakai Orang 4 (mixer input). */
typedef struct {
    uint16_t channel[IBUS_NUM_CHANNELS];              /* nilai sudah di-mask, bit flag dibuang */
    bool     channel_failsafe_flag[IBUS_NUM_CHANNELS]; /* lihat IBUS_CH_FAILSAFE_FLAG_BIT, TODO verifikasi hardware */
    bool     data_valid;   /* true kalau minimal 1 frame channel pernah diterima */
} IBUS_ChannelData_t;

/** Konfigurasi deteksi failsafe lewat nilai channel tertentu (mekanisme
 *  #2). HARUS diisi manual sesuai apa yang sudah dikonfigurasi di menu
 *  transmitter -- lihat RxIbus_SetFailsafeChannel(). Dibiarkan disabled
 *  by default (enabled=false) supaya tidak salah asumsi nilai sebelum
 *  dikonfirmasi ke radio sungguhan. */
typedef struct {
    bool     enabled;
    uint8_t  channel_index;   /* 0-based index ke channel[], mis. 2 = throttle */
    uint16_t expected_value;  /* nilai failsafe yang di-set di menu TX */
    uint16_t tolerance;       /* +- toleransi pembacaan, mis. 20 */
} IBUS_FailsafeChannelCfg_t;

/** State parser + failsafe. Satu instance per sistem. */
typedef struct {
    uint8_t  rx_buffer[IBUS_FRAME_LEN];
    uint8_t  rx_index;

    IBUS_ChannelData_t channels;
    IBUS_LinkStatus_t  link_status;
    uint32_t           last_valid_frame_tick_ms;
    bool               failsafe_triggered;      /* latch, spy Nav_TriggerRTH cuma dipanggil sekali per event */
    /* True begitu SATU frame ber-checksum valid pernah diterima sejak
     * RxIbus_Init(). Dipakai RxIbus_Update() untuk membedakan "link
     * terputus" dari "belum pernah ada link" -- lihat alasannya di sana.
     * Sengaja TIDAK pernah kembali false: sekali transmitter terbukti
     * pernah tersambung, link-loss berikutnya memang link-loss. */
    bool               has_received_frame;

    IBUS_FailsafeChannelCfg_t failsafe_cfg;      /* mekanisme #2, lihat RxIbus_SetFailsafeChannel() */
    bool  frame_indicates_failsafe;              /* cache hasil evaluasi mekanisme #2 & #3 dari frame terakhir */

    bool initialized;
} IBUS_State_t;

/* ------------------------------------------------------------------- */
/* API                                                                  */
/* ------------------------------------------------------------------- */

/** Inisialisasi state parser. Panggil sekali saat boot. */
void RxIbus_Init(IBUS_State_t *state);

/**
 * @brief  Konfigurasi mekanisme failsafe #2 (value-match). Panggil
 *         SETELAH failsafe per-channel dikonfigurasi manual di menu
 *         transmitter (mis. FS-i6/i6X: RX Setup -> Failsafe). Contoh:
 *         throttle (channel index 2) di-set failsafe ke -100% di TX ->
 *         RxIbus_SetFailsafeChannel(&s_fc.rx, 2U, IBUS_CH_VALUE_MIN, 20U);
 * @param  channel_index  0-based index channel yang dipantau
 * @param  expected_value nilai failsafe yang sudah di-set di TX
 * @param  tolerance      +- toleransi pembacaan
 */
void RxIbus_SetFailsafeChannel(IBUS_State_t *state,
                                uint8_t channel_index,
                                uint16_t expected_value,
                                uint16_t tolerance);

/**
 * @brief  Suapi satu byte mentah dari UART RX (dipanggil dari task drain
 *         ring buffer bsp_uart.c milik Orang 1). Resync otomatis: cari
 *         IBUS_HEADER_BYTE1 lalu IBUS_HEADER_BYTE2 berurutan sebagai
 *         anchor, karena frame iBUS selalu fixed 32 byte (beda dari CRSF
 *         yang length-nya variable dari byte kedua).
 */
void RxIbus_FeedByte(IBUS_State_t *state, uint8_t byte, uint32_t tick_ms);

/** Varian batch -- dorong beberapa byte sekaligus. */
void RxIbus_FeedBuffer(IBUS_State_t *state, const uint8_t *data, uint16_t len, uint32_t tick_ms);

/**
 * @brief  Evaluasi gabungan TIGA mekanisme failsafe (lihat disclaimer di
 *         atas file ini), dipanggil scheduler tiap loop terlepas dari
 *         kapan byte UART datang.
 * @return true kalau baru saja transisi ke link-loss pada panggilan ini
 *         (edge, bukan level).
 */
bool RxIbus_CheckFailsafe(IBUS_State_t *state, uint32_t tick_ms);

/**
 * @brief  Update gabungan: cek failsafe DAN trigger RTH otomatis lewat
 *         navigation.c kalau baru transisi ke link-loss. Bentuk & alasan
 *         function-pointer sama seperti RxCrsf_Update() sebelumnya
 *         (supaya modul ini tetap testable terpisah tanpa link ke
 *         seluruh state navigasi).
 */
typedef void (*RxIbus_RthTriggerFn)(void *nav_context);

void RxIbus_Update(IBUS_State_t *state,
                    uint32_t tick_ms,
                    RxIbus_RthTriggerFn rth_trigger_fn,
                    void *nav_context);

/** Ambil data channel terakhir yang valid untuk dikonsumsi Orang 4. */
const IBUS_ChannelData_t *RxIbus_GetChannels(const IBUS_State_t *state);

/** Status link saat ini (level, bukan edge). */
IBUS_LinkStatus_t RxIbus_GetLinkStatus(const IBUS_State_t *state);

#ifdef __cplusplus
}
#endif

#endif /* RX_IBUS_H */
