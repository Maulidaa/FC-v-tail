/**
 * @file    rx_crsf.c
 * @brief   Implementasi parser CRSF (RC_CHANNELS_PACKED) + failsafe
 *          link-loss.
 *
 * Status: SKELETON.
 *   - State machine parsing frame (sync -> length -> payload+crc)
 *     sudah lengkap alurnya, tapi field offset/urutan byte WAJIB
 *     diverifikasi ulang terhadap spek resmi CRSF sebelum dipakai ke
 *     radio sungguhan — jangan asumsikan dari memori/skeleton ini.
 *   - CRC8 di sini pakai poly 0xD5 (kebetulan sama dengan CRC8 DVB-S2
 *     yang dipakai protocol.md untuk FC<->web), TAPI diimplementasi
 *     terpisah/lokal di file ini (bukan reuse crc8Dvbs2() dari
 *     comms/protocol.c milik Orang 1) supaya modul RX radio ini tidak
 *     punya dependency ke modul protokol FC<->web — dua hal yang
 *     kebetulan sama angkanya tapi konseptual berbeda domain.
 *   - Unpacking 16 channel x 11-bit dari payload packed: logic bit
 *     shifting sudah standard, tapi tetap TODO verifikasi terhadap
 *     capture logic analyzer / referensi betaflight sebelum terbang.
 */

#include "rx_crsf.h"
#include <string.h>

/* ------------------------------------------------------------------- */
/* CRC8 lokal (poly 0xD5, tanpa reflect, initial 0x00)                  */
/* ------------------------------------------------------------------- */

static uint8_t Crsf_Crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00U;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8U; bit++) {
            if (crc & 0x80U) {
                crc = (uint8_t)((crc << 1) ^ 0xD5U);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* ------------------------------------------------------------------- */
/* Unpacking channel                                                    */
/* ------------------------------------------------------------------- */

/**
 * @brief  Unpack 16 channel 11-bit dari payload RC_CHANNELS_PACKED
 *         (22 byte, little-bit-endian packed). TODO: verifikasi urutan
 *         bit terhadap spek CRSF resmi / referensi implementasi lain
 *         sebelum dipercaya penuh.
 */
static void UnpackChannels(const uint8_t *payload, CRSF_ChannelData_t *out)
{
    uint32_t bit_buffer = 0;
    uint8_t  bits_in_buffer = 0;
    uint8_t  byte_index = 0;

    for (uint8_t ch = 0; ch < CRSF_NUM_CHANNELS; ch++) {
        while (bits_in_buffer < 11U) {
            bit_buffer |= ((uint32_t)payload[byte_index] << bits_in_buffer);
            bits_in_buffer += 8U;
            byte_index++;
        }
        out->channel[ch] = (uint16_t)(bit_buffer & 0x7FFU);
        bit_buffer >>= 11U;
        bits_in_buffer -= 11U;
    }
    out->data_valid = true;
}

/* ------------------------------------------------------------------- */
/* State machine parsing                                                */
/* ------------------------------------------------------------------- */

static void ResetParser(CRSF_State_t *state)
{
    state->rx_index        = 0U;
    state->expected_length = 0U;
}

/** Proses satu frame lengkap yang sudah terkumpul di rx_buffer.
 *  Layout diasumsikan: [0]=sync [1]=len [2]=type [3..]=payload [last]=crc
 *  len = jumlah byte dari [type] sampai sebelum [crc] + 1 (byte crc itu
 *  sendiri) — TODO: cocokkan persis definisi "len" di spek resmi CRSF,
 *  skeleton ini pakai konvensi paling umum dipakai implementasi lain. */
static bool ProcessCompleteFrame(CRSF_State_t *state, uint32_t tick_ms)
{
    uint8_t type       = state->rx_buffer[2];
    uint8_t frame_total = (uint8_t)(state->expected_length + 2U); /* sync+len prefix */
    uint8_t crc_received = state->rx_buffer[frame_total - 1U];

    /* CRC dihitung atas [type][payload...], tidak termasuk sync & len,
     * tidak termasuk byte crc itu sendiri. */
    uint8_t crc_calc = Crsf_Crc8(&state->rx_buffer[2], (uint8_t)(frame_total - 3U));

    if (crc_calc != crc_received) {
        /* CRC gagal -> buang frame ini, jangan update failsafe timer.
         * Resync akan terjadi otomatis di panggilan FeedByte berikutnya
         * begitu sync byte baru ditemukan. */
        return false;
    }

    if (type == (uint8_t)CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
        UnpackChannels(&state->rx_buffer[3], &state->channels);
        state->last_valid_frame_tick_ms = tick_ms;
        state->link_status              = CRSF_LINK_OK;
        state->failsafe_triggered       = false; /* reset latch begitu link pulih */
        return true;
    }

    /* Frame type lain (belum relevan untuk RX-only v1) — abaikan tapi
     * tetap hitung sebagai "link hidup" karena CRC valid berarti radio
     * memang mengirim sesuatu. TODO: kalau nanti butuh telemetry balik
     * atau frame type lain, tangani di sini. */
    state->last_valid_frame_tick_ms = tick_ms;
    state->link_status              = CRSF_LINK_OK;
    return true;
}

void RxCrsf_Init(CRSF_State_t *state)
{
    memset(state, 0, sizeof(*state));
    state->link_status = CRSF_LINK_OK; /* optimis di boot, failsafe akan
                                         * trigger sendiri kalau memang
                                         * tidak ada sinyal masuk sampai
                                         * timeout pertama */
    state->last_valid_frame_tick_ms = 0U;
    state->initialized = true;
}

void RxCrsf_FeedByte(CRSF_State_t *state, uint8_t byte, uint32_t tick_ms)
{
    if (!state->initialized) {
        RxCrsf_Init(state);
    }

    if (state->rx_index == 0U) {
        if (byte != CRSF_SYNC_BYTE) {
            return; /* buang byte sampai ketemu sync — resync otomatis */
        }
        state->rx_buffer[0] = byte;
        state->rx_index     = 1U;
        return;
    }

    if (state->rx_index == 1U) {
        /* byte kedua = length */
        if (byte == 0U || byte > (CRSF_MAX_FRAME_SIZE - 2U)) {
            /* length tidak masuk akal -> buang, mulai cari sync lagi */
            ResetParser(state);
            return;
        }
        state->expected_length = byte;
        state->rx_buffer[1]    = byte;
        state->rx_index        = 2U;
        return;
    }

    /* Sedang mengumpulkan payload+type+crc */
    state->rx_buffer[state->rx_index] = byte;
    state->rx_index++;

    uint8_t frame_total = (uint8_t)(state->expected_length + 2U);
    if (state->rx_index >= frame_total) {
        ProcessCompleteFrame(state, tick_ms);
        ResetParser(state);
    }
}

void RxCrsf_FeedBuffer(CRSF_State_t *state, const uint8_t *data, uint16_t len, uint32_t tick_ms)
{
    for (uint16_t i = 0; i < len; i++) {
        RxCrsf_FeedByte(state, data[i], tick_ms);
    }
}

bool RxCrsf_CheckFailsafe(CRSF_State_t *state, uint32_t tick_ms)
{
    uint32_t elapsed_ms = tick_ms - state->last_valid_frame_tick_ms;

    if (elapsed_ms >= CRSF_FAILSAFE_TIMEOUT_MS) {
        bool is_new_event = (state->link_status != CRSF_LINK_LOST);
        state->link_status = CRSF_LINK_LOST;
        return is_new_event;
    }

    return false;
}

void RxCrsf_Update(CRSF_State_t *state,
                    uint32_t tick_ms,
                    RxCrsf_RthTriggerFn rth_trigger_fn,
                    void *nav_context)
{
    bool link_just_lost = RxCrsf_CheckFailsafe(state, tick_ms);

    if (link_just_lost && !state->failsafe_triggered) {
        state->failsafe_triggered = true;
        if (rth_trigger_fn != NULL) {
            rth_trigger_fn(nav_context);
        }
        /* TODO: selain trigger RTH, koordinasikan ke Orang 1 apakah
         * failsafe link-loss juga perlu masuk daftar armed-state guard
         * (mis. tolak command *_TEST tambahan selama failsafe aktif) —
         * belum ada keputusan eksplisit soal ini di dokumen manapun. */
    }
}

const CRSF_ChannelData_t *RxCrsf_GetChannels(const CRSF_State_t *state)
{
    return &state->channels;
}

CRSF_LinkStatus_t RxCrsf_GetLinkStatus(const CRSF_State_t *state)
{
    return state->link_status;
}
