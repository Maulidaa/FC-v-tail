/**
 * @file    rx_crsf.h
 * @brief   Parser protokol CRSF (Crossfire) untuk channel input dari
 *          radio receiver + deteksi failsafe link-loss.
 *
 * Tanggung jawab: Orang 3 (Fusion, Navigasi & Failsafe)
 *
 * Hardware: USART1_RX (PA10), RX-only untuk v1 — keputusan yang
 * sudah diambil (lihat pinout-fc-stm32f411.md & pembagian tugas):
 * tidak ada telemetry balik, PA9 (USART1_TX) dibebaskan jadi spare.
 * Kabari Orang 1 kalau keputusan ini berubah, karena PA9 perlu
 * direalokasi lagi di bsp_pinmap.h.
 *
 * Modul ini TIDAK menyentuh UART secara langsung — byte mentah
 * didorong masuk oleh driver bsp_uart.c (Orang 1, lewat interrupt/DMA
 * RX) ke fungsi RxCrsf_FeedByte()/RxCrsf_FeedBuffer(). Pemisahan ini
 * konsisten dengan prinsip modul lain: BSP di Orang 1, logic protokol
 * spesifik di modul pemilik fitur.
 */

#ifndef RX_CRSF_H
#define RX_CRSF_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------- */
/* Konstanta protokol CRSF                                              */
/* ------------------------------------------------------------------- */

#define CRSF_SYNC_BYTE              0xC8U
#define CRSF_MAX_FRAME_SIZE         64U     /* spek CRSF: frame <= 64 byte total */
#define CRSF_NUM_CHANNELS           16U     /* RC_CHANNELS_PACKED: 16 channel, 11-bit tiap channel */
#define CRSF_CHANNEL_VALUE_MIN      172U    /* ~988us, konvensi CRSF/Betaflight */
#define CRSF_CHANNEL_VALUE_MID      992U    /* ~1500us */
#define CRSF_CHANNEL_VALUE_MAX      1811U   /* ~2012us */

/** Frame type yang relevan untuk RX-only channel input.
 *  TODO: kalau nanti ada kebutuhan telemetry balik, tambahkan type
 *  lain di sini (battery, GPS, attitude, dst — lihat spek CRSF resmi). */
typedef enum {
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
} CRSF_FrameType_t;

/** Timeout tanpa frame valid sebelum dianggap link-loss (ms).
 *  TODO: tuning — nilai umum di FC lain 500ms-1000ms, tapi sesuaikan
 *  dengan rate paket radio yang dipakai tim (mis. ELRS packet rate). */
#define CRSF_FAILSAFE_TIMEOUT_MS    500U

/* ------------------------------------------------------------------- */
/* Tipe data                                                            */
/* ------------------------------------------------------------------- */

/** Status link RX dari sudut pandang failsafe. */
typedef enum {
    CRSF_LINK_OK = 0,
    CRSF_LINK_LOST,
} CRSF_LinkStatus_t;

/** Hasil parsing channel, sudah dalam bentuk yang gampang dipakai
 *  Orang 4 (mixer input) — nilai mentah 11-bit CRSF (172-1811). */
typedef struct {
    uint16_t channel[CRSF_NUM_CHANNELS];
    bool     data_valid;   /* true kalau minimal 1 frame channel pernah diterima */
} CRSF_ChannelData_t;

/** State parser + failsafe. Satu instance per sistem. */
typedef struct {
    /* Buffer parsing byte-per-byte */
    uint8_t  rx_buffer[CRSF_MAX_FRAME_SIZE];
    uint8_t  rx_index;
    uint8_t  expected_length;   /* dari byte length setelah sync, 0 = belum tahu */

    CRSF_ChannelData_t channels;
    CRSF_LinkStatus_t  link_status;
    uint32_t           last_valid_frame_tick_ms;
    bool               failsafe_triggered;   /* latch, supaya Nav_TriggerRTH cuma dipanggil sekali per event */

    bool initialized;
} CRSF_State_t;

/* ------------------------------------------------------------------- */
/* API                                                                  */
/* ------------------------------------------------------------------- */

/** Inisialisasi state parser. Panggil sekali saat boot. */
void RxCrsf_Init(CRSF_State_t *state);

/**
 * @brief  Suapi satu byte mentah dari UART RX (dipanggil dari ISR/DMA
 *         callback bsp_uart.c milik Orang 1, atau dari task drain
 *         buffer kalau bsp_uart pakai ring buffer). Melakukan resync
 *         otomatis kalau sync byte tidak ditemukan di posisi yang
 *         diharapkan (mirip filosofi resync di protocol.md Bagian 2,
 *         tapi ini protokol CRSF yang terpisah/independen dari
 *         protocol.md — protocol.md itu kontrak FC<->web, bukan
 *         FC<->radio).
 */
void RxCrsf_FeedByte(CRSF_State_t *state, uint8_t byte, uint32_t tick_ms);

/**
 * @brief  Varian batch — dorong beberapa byte sekaligus (mis. hasil
 *         satu DMA transfer complete). Lebih efisien daripada panggil
 *         RxCrsf_FeedByte() berkali-kali dari loop caller.
 */
void RxCrsf_FeedBuffer(CRSF_State_t *state, const uint8_t *data, uint16_t len, uint32_t tick_ms);

/**
 * @brief  Cek status failsafe berdasarkan waktu sejak frame valid
 *         terakhir. Dipanggil scheduler (Orang 1) tiap loop, terlepas
 *         dari kapan byte UART datang — supaya link-loss tetap
 *         terdeteksi walau tidak ada byte masuk sama sekali.
 *
 * @param  tick_ms   Timestamp saat ini (ms).
 * @return true kalau baru saja transisi ke link-loss pada panggilan
 *         ini (edge, bukan level) — caller (biasanya modul ini sendiri
 *         di RxCrsf_Update()) pakai ini untuk trigger Nav_TriggerRTH()
 *         tepat sekali saat kejadian, bukan tiap loop selama link mati.
 */
bool RxCrsf_CheckFailsafe(CRSF_State_t *state, uint32_t tick_ms);

/**
 * @brief  Update gabungan: cek failsafe DAN trigger RTH otomatis lewat
 *         navigation.c kalau baru transisi ke link-loss. Ini fungsi
 *         yang sebaiknya dipanggil scheduler (bukan CheckFailsafe
 *         mentah), supaya logic "link-loss -> RTH" konsisten satu
 *         tempat sesuai pembagian tugas Orang 3.
 *
 *         Diberi function pointer untuk trigger RTH (bukan include
 *         langsung navigation.h) supaya modul ini tetap testable
 *         terpisah tanpa perlu link ke seluruh state navigasi.
 */
typedef void (*RxCrsf_RthTriggerFn)(void *nav_context);

void RxCrsf_Update(CRSF_State_t *state,
                    uint32_t tick_ms,
                    RxCrsf_RthTriggerFn rth_trigger_fn,
                    void *nav_context);

/** Ambil data channel terakhir yang valid untuk dikonsumsi Orang 4. */
const CRSF_ChannelData_t *RxCrsf_GetChannels(const CRSF_State_t *state);

/** Status link saat ini (level, bukan edge) — dipakai juga kalau perlu
 *  expose ke CMD_GET_STATUS (belum ada field khusus di protocol.md v1,
 *  TODO diskusikan ke Orang 1 kalau operator perlu lihat status link
 *  radio dari web, bukan cuma armed/battery/gps). */
CRSF_LinkStatus_t RxCrsf_GetLinkStatus(const CRSF_State_t *state);

#ifdef __cplusplus
}
#endif

#endif /* RX_CRSF_H */
