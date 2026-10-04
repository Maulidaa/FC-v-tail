/**
 * @file    rx_ibus.c
 * @brief   Implementasi parser iBUS (servo/channel frame) + failsafe
 *          link-loss 3-lapis.
 *
 * Status: SKELETON.
 *   - State machine parsing (header1 -> header2 -> 30 byte sisa -> cek
 *     checksum) sudah lengkap alurnya, dan format frame (32 byte fixed,
 *     14 channel x u16 LE, checksum 0xFFFF-sum) sudah dicek silang ke
 *     beberapa referensi implementasi independen (Betaflight/iNav/
 *     IBusBM) -- TAPI tetap WAJIB diverifikasi ulang terhadap capture
 *     serial sungguhan (logic analyzer/osiloskop ke pin iBUS receiver
 *     asli) sebelum dipakai ke radio sungguhan, jangan asumsikan dari
 *     skeleton ini saja.
 *   - Mekanisme failsafe #3 (bit-flag per-channel, IBUS_CH_FAILSAFE_FLAG_BIT)
 *     berdasarkan LAPORAN KOMUNITAS untuk FS-iA6B (belum kami verifikasi
 *     langsung). Kalau pas dites ternyata bit itu TIDAK pernah aktif di
 *     hardware kalian, itu bukan bug di sini -- cukup andalkan mekanisme
 *     #1 (timeout) + #2 (value-match, WAJIB dikonfigurasi manual di TX)
 *     saja, jangan didiamkan tanpa salah satu dari keduanya aktif.
 */

#include "rx_ibus.h"
#include <string.h>

/* ------------------------------------------------------------------- */
/* Checksum iBUS: 0xFFFF - sum(byte[0..IBUS_FRAME_LEN-3]), little-endian */
/* ------------------------------------------------------------------- */

static uint16_t Ibus_ComputeChecksum(const uint8_t *frame)
{
    uint16_t sum = 0U;
    for (uint8_t i = 0; i < (IBUS_FRAME_LEN - 2U); i++) {
        sum = (uint16_t)(sum + frame[i]);
    }
    return (uint16_t)(0xFFFFU - sum);
}

/* ------------------------------------------------------------------- */
/* Unpacking channel                                                    */
/* ------------------------------------------------------------------- */

static void UnpackChannels(const uint8_t *payload, IBUS_ChannelData_t *out)
{
    for (uint8_t ch = 0; ch < IBUS_NUM_CHANNELS; ch++) {
        uint16_t raw = (uint16_t)((uint16_t)payload[ch * 2U] |
                                   ((uint16_t)payload[(ch * 2U) + 1U] << 8));
        out->channel[ch]               = (uint16_t)(raw & IBUS_CH_VALUE_MASK);
        out->channel_failsafe_flag[ch] = (raw & IBUS_CH_FAILSAFE_FLAG_BIT) != 0U;
    }
    out->data_valid = true;
}

/* ------------------------------------------------------------------- */
/* Evaluasi mekanisme failsafe #2 (value-match) & #3 (bit-flag)         */
/* ------------------------------------------------------------------- */

static bool EvaluateFrameFailsafe(const IBUS_State_t *state)
{
    /* #2: value-match, hanya aktif kalau sudah dikonfigurasi eksplisit */
    if (state->failsafe_cfg.enabled) {
        uint8_t  idx = state->failsafe_cfg.channel_index;
        if (idx < IBUS_NUM_CHANNELS) {
            int32_t diff = (int32_t)state->channels.channel[idx] -
                            (int32_t)state->failsafe_cfg.expected_value;
            if (diff < 0) { diff = -diff; }
            if ((uint32_t)diff <= state->failsafe_cfg.tolerance) {
                return true;
            }
        }
    }

    /* #3: bit-flag per-channel -- cek semua channel yang dipetakan main.c
     * (index 0-5: roll/pitch/throttle/yaw/arm/passthrough), bukan
     * seluruh 14 channel, supaya channel kosong/tidak terpakai di
     * receiver 6ch tidak memicu false-positive. */
    for (uint8_t ch = 0; ch < 6U && ch < IBUS_NUM_CHANNELS; ch++) {
        if (state->channels.channel_failsafe_flag[ch]) {
            return true;
        }
    }

    return false;
}

/* ------------------------------------------------------------------- */
/* State machine parsing                                                */
/* ------------------------------------------------------------------- */

static void ResetParser(IBUS_State_t *state)
{
    state->rx_index = 0U;
}

static void ProcessCompleteFrame(IBUS_State_t *state, uint32_t tick_ms)
{
    uint16_t checksum_calc = Ibus_ComputeChecksum(state->rx_buffer);
    uint16_t checksum_recv = (uint16_t)((uint16_t)state->rx_buffer[IBUS_FRAME_LEN - 2U] |
                                         ((uint16_t)state->rx_buffer[IBUS_FRAME_LEN - 1U] << 8));

    if (checksum_calc != checksum_recv) {
        /* checksum gagal -> buang frame, jangan sentuh channel/timer
         * manapun. Resync otomatis di FeedByte berikutnya. */
        return;
    }

    UnpackChannels(&state->rx_buffer[2], &state->channels);

    /* Frame ber-checksum valid = byte memang mengalir di UART (dipakai
     * mekanisme #1/timeout), TAPI ini TIDAK otomatis berarti link RF
     * sehat -- lihat disclaimer di rx_ibus.h. Evaluasi #2/#3 dulu
     * sebelum memutuskan link_status. */
    state->last_valid_frame_tick_ms = tick_ms;
    state->has_received_frame       = true;
    state->frame_indicates_failsafe = EvaluateFrameFailsafe(state);
}

void RxIbus_Init(IBUS_State_t *state)
{
    memset(state, 0, sizeof(*state));
    state->link_status = IBUS_LINK_OK; /* optimis di boot, failsafe akan
                                         * trigger sendiri lewat timeout
                                         * kalau memang tidak ada sinyal
                                         * masuk sama sekali */
    state->last_valid_frame_tick_ms = 0U;
    state->initialized = true;
}

void RxIbus_SetFailsafeChannel(IBUS_State_t *state,
                                uint8_t channel_index,
                                uint16_t expected_value,
                                uint16_t tolerance)
{
    state->failsafe_cfg.enabled       = true;
    state->failsafe_cfg.channel_index = channel_index;
    state->failsafe_cfg.expected_value = expected_value;
    state->failsafe_cfg.tolerance     = tolerance;
}

void RxIbus_FeedByte(IBUS_State_t *state, uint8_t byte, uint32_t tick_ms)
{
    if (!state->initialized) {
        RxIbus_Init(state);
    }

    if (state->rx_index == 0U) {
        if (byte != IBUS_HEADER_BYTE1) {
            return; /* buang byte sampai ketemu header1 -- resync otomatis */
        }
        state->rx_buffer[0] = byte;
        state->rx_index     = 1U;
        return;
    }

    if (state->rx_index == 1U) {
        if (byte != IBUS_HEADER_BYTE2) {
            /* Bukan header2 yang diharapkan. Kalau byte ini kebetulan
             * header1 lagi, anggap itu awal frame baru (tetap di index
             * 1); kalau bukan, buang total dan mulai cari header1 lagi. */
            if (byte == IBUS_HEADER_BYTE1) {
                return;
            }
            ResetParser(state);
            return;
        }
        state->rx_buffer[1] = byte;
        state->rx_index     = 2U;
        return;
    }

    /* Sedang mengumpulkan sisa 30 byte (28 channel + 2 checksum) */
    state->rx_buffer[state->rx_index] = byte;
    state->rx_index++;

    if (state->rx_index >= IBUS_FRAME_LEN) {
        ProcessCompleteFrame(state, tick_ms);
        ResetParser(state);
    }
}

void RxIbus_FeedBuffer(IBUS_State_t *state, const uint8_t *data, uint16_t len, uint32_t tick_ms)
{
    for (uint16_t i = 0; i < len; i++) {
        RxIbus_FeedByte(state, data[i], tick_ms);
    }
}

bool RxIbus_CheckFailsafe(IBUS_State_t *state, uint32_t tick_ms)
{
    uint32_t elapsed_ms = tick_ms - state->last_valid_frame_tick_ms;

    /* Gabungan tiga mekanisme, di-OR (lihat disclaimer di rx_ibus.h):
     *   #1 timeout murni, #2/#3 dievaluasi tiap kali frame baru masuk
     *   dan hasilnya di-cache di frame_indicates_failsafe. */
    bool timeout_expired = (elapsed_ms >= IBUS_FAILSAFE_TIMEOUT_MS);
    bool should_be_lost   = timeout_expired || state->frame_indicates_failsafe;

    bool is_new_event = should_be_lost && (state->link_status != IBUS_LINK_LOST);
    state->link_status = should_be_lost ? IBUS_LINK_LOST : IBUS_LINK_OK;
    return is_new_event;
}

void RxIbus_Update(IBUS_State_t *state,
                    uint32_t tick_ms,
                    RxIbus_RthTriggerFn rth_trigger_fn,
                    void *nav_context)
{
    bool link_just_lost = RxIbus_CheckFailsafe(state, tick_ms);

    /* BUG yang diperbaiki: RTH ter-trigger palsu di setiap boot.
     *
     * RxIbus_Init() menyetel link_status = IBUS_LINK_OK (optimis) dan
     * last_valid_frame_tick_ms = 0. Saat scheduler mulai jalan, tick_ms
     * sudah beberapa detik (kalibrasi boot yang blocking), jadi panggilan
     * RxIbus_CheckFailsafe() PERTAMA langsung melihat elapsed jauh di atas
     * IBUS_FAILSAFE_TIMEOUT_MS. Transisi OK -> LOST itu dianggap "link baru
     * saja putus" dan Nav_TriggerRTH() dipanggil -- padahal belum pernah
     * ada satu frame pun masuk dan transmitter mungkin memang belum
     * dinyalakan. Akibatnya nav mode masuk RTH di setiap boot tanpa TX.
     *
     * Syarat has_received_frame membuat RTH hanya dipicu oleh link-loss
     * yang SUNGGUHAN: pernah ada frame valid, lalu hilang. Status
     * link_status sendiri tetap jadi LOST seperti sebelumnya (itu memang
     * benar dan menahan arming lewat rx_link_ok di main.c) -- yang
     * dicegah di sini HANYA aksi RTH-nya. */
    if (link_just_lost && state->has_received_frame && !state->failsafe_triggered) {
        state->failsafe_triggered = true;
        if (rth_trigger_fn != NULL) {
            rth_trigger_fn(nav_context);
        }
    } else if (state->link_status == IBUS_LINK_OK) {
        /* reset latch begitu link pulih (semua mekanisme kembali sehat),
         * supaya event failsafe berikutnya bisa trigger RTH lagi */
        state->failsafe_triggered = false;
    }
}

const IBUS_ChannelData_t *RxIbus_GetChannels(const IBUS_State_t *state)
{
    return &state->channels;
}

IBUS_LinkStatus_t RxIbus_GetLinkStatus(const IBUS_State_t *state)
{
    return state->link_status;
}
