/* Test host-side untuk rx/rx_ibus.c, memakai implementasi ASLI (bukan
 * tiruan). Tidak butuh CMSIS/register palsu -- modul ini murni logic
 * parsing byte + state machine, tidak menyentuh hardware sama sekali. */
#include "test_common.h"
#include <string.h>
#include "rx_ibus.h"

/* -------------------------------------------------------------------
 * Helper bikin frame iBUS valid dari 14 nilai channel, isi checksum-nya
 * sendiri -- supaya test tidak duplikasi logic checksum dari rx_ibus.c
 * (kalau nanti formula checksum berubah, cukup ubah di satu tempat: ini).
 * ------------------------------------------------------------------- */
static void build_frame(uint8_t out[IBUS_FRAME_LEN], const uint16_t ch[IBUS_NUM_CHANNELS])
{
    out[0] = IBUS_HEADER_BYTE1;
    out[1] = IBUS_HEADER_BYTE2;
    for (uint8_t i = 0; i < IBUS_NUM_CHANNELS; i++) {
        out[2 + i * 2]     = (uint8_t)(ch[i] & 0xFFU);
        out[2 + i * 2 + 1] = (uint8_t)((ch[i] >> 8) & 0xFFU);
    }
    uint16_t sum = 0U;
    for (uint8_t i = 0; i < IBUS_FRAME_LEN - 2U; i++) {
        sum = (uint16_t)(sum + out[i]);
    }
    uint16_t crc = (uint16_t)(0xFFFFU - sum);
    out[IBUS_FRAME_LEN - 2U] = (uint8_t)(crc & 0xFFU);
    out[IBUS_FRAME_LEN - 1U] = (uint8_t)((crc >> 8) & 0xFFU);
}

static void default_channels(uint16_t ch[IBUS_NUM_CHANNELS])
{
    for (uint8_t i = 0; i < IBUS_NUM_CHANNELS; i++) {
        ch[i] = IBUS_CH_VALUE_MID;
    }
}

static int g_rth_calls = 0;
static void rth_trigger(void *ctx) { (void)ctx; g_rth_calls++; }

int main(void)
{
    /* ---------------- 1) Frame valid --> channel & link OK ---------------- */
    {
        IBUS_State_t st;
        RxIbus_Init(&st);

        uint16_t ch[IBUS_NUM_CHANNELS];
        default_channels(ch);
        ch[0] = 1000U; ch[1] = 1500U; ch[2] = 1750U; ch[5] = 2000U;

        uint8_t frame[IBUS_FRAME_LEN];
        build_frame(frame, ch);
        RxIbus_FeedBuffer(&st, frame, IBUS_FRAME_LEN, 1000U);

        const IBUS_ChannelData_t *out = RxIbus_GetChannels(&st);
        CHECK(out->data_valid);
        CHECK(out->channel[0] == 1000U);
        CHECK(out->channel[1] == 1500U);
        CHECK(out->channel[2] == 1750U);
        CHECK(out->channel[5] == 2000U);
        CHECK(RxIbus_GetLinkStatus(&st) == IBUS_LINK_OK);
        CHECK(!RxIbus_CheckFailsafe(&st, 1100U)); /* belum timeout */
    }

    /* ---------------- 2) Checksum salah --> frame dibuang ---------------- */
    {
        IBUS_State_t st;
        RxIbus_Init(&st);

        uint16_t ch[IBUS_NUM_CHANNELS];
        default_channels(ch);
        uint8_t frame[IBUS_FRAME_LEN];
        build_frame(frame, ch);
        frame[IBUS_FRAME_LEN - 1U] ^= 0xFFU; /* rusak byte checksum tinggi */

        RxIbus_FeedBuffer(&st, frame, IBUS_FRAME_LEN, 1000U);
        const IBUS_ChannelData_t *out = RxIbus_GetChannels(&st);
        CHECK(!out->data_valid); /* belum pernah ada frame VALID yang diterima */
    }

    /* ---------------- 3) Resync setelah sampah di depan header ------------ */
    {
        IBUS_State_t st;
        RxIbus_Init(&st);

        uint16_t ch[IBUS_NUM_CHANNELS];
        default_channels(ch);
        ch[3] = 1234U;
        uint8_t frame[IBUS_FRAME_LEN];
        build_frame(frame, ch);

        /* sampah acak + satu byte 0x20 nyasar (bukan header sungguhan)
         * sebelum frame asli -- parser harus tetap nemu frame yang benar */
        uint8_t garbage[] = { 0x00, 0xAB, IBUS_HEADER_BYTE1, 0x99, 0x11 };
        RxIbus_FeedBuffer(&st, garbage, sizeof(garbage), 500U);
        RxIbus_FeedBuffer(&st, frame, IBUS_FRAME_LEN, 1000U);

        const IBUS_ChannelData_t *out = RxIbus_GetChannels(&st);
        CHECK(out->data_valid);
        CHECK(out->channel[3] == 1234U);
    }

    /* ------- 3b) Resync pola "0x20 0x20 0x40 ..." (dua header1 berturutan,
     *              dipindah dari test_ibus_parser.c versi sebelumnya) ------- */
    {
        IBUS_State_t st;
        RxIbus_Init(&st);

        uint16_t ch[IBUS_NUM_CHANNELS];
        default_channels(ch);
        ch[4] = 1777U;
        uint8_t frame[IBUS_FRAME_LEN];
        build_frame(frame, ch);

        /* byte header1 "palsu" pertama, lalu header1 KEDUA -- harus
         * dipakai gantikan yang pertama, bukan bikin parser reset total
         * (lihat cabang khusus di RxIbus_FeedByte()). */
        RxIbus_FeedByte(&st, IBUS_HEADER_BYTE1, 4000U);
        RxIbus_FeedByte(&st, IBUS_HEADER_BYTE1, 4000U);
        RxIbus_FeedBuffer(&st, &frame[1], (uint16_t)(IBUS_FRAME_LEN - 1U), 4001U);

        const IBUS_ChannelData_t *out = RxIbus_GetChannels(&st);
        CHECK(out->data_valid);
        CHECK(out->channel[4] == 1777U);
    }

    /* ---------------- 4) Mekanisme #1: timeout murni ---------------- */
    {
        IBUS_State_t st;
        RxIbus_Init(&st);
        uint16_t ch[IBUS_NUM_CHANNELS];
        default_channels(ch);
        uint8_t frame[IBUS_FRAME_LEN];
        build_frame(frame, ch);
        RxIbus_FeedBuffer(&st, frame, IBUS_FRAME_LEN, 1000U);

        CHECK(!RxIbus_CheckFailsafe(&st, 1000U + IBUS_FAILSAFE_TIMEOUT_MS - 1U));
        bool edge = RxIbus_CheckFailsafe(&st, 1000U + IBUS_FAILSAFE_TIMEOUT_MS);
        CHECK(edge); /* transisi baru terjadi persis di batas timeout */
        CHECK(RxIbus_GetLinkStatus(&st) == IBUS_LINK_LOST);
        /* edge cuma sekali -- panggilan berikutnya level, bukan edge lagi */
        CHECK(!RxIbus_CheckFailsafe(&st, 1000U + IBUS_FAILSAFE_TIMEOUT_MS + 10U));
    }

    /* -------- 5) Mekanisme #2: value-match, WALAU frame terus masuk ------- */
    {
        IBUS_State_t st;
        RxIbus_Init(&st);
        RxIbus_SetFailsafeChannel(&st, 2U, IBUS_CH_VALUE_MIN, 20U); /* throttle failsafe -100% */

        uint16_t ch[IBUS_NUM_CHANNELS];
        default_channels(ch);
        ch[2] = 1500U; /* throttle normal dulu */
        uint8_t frame[IBUS_FRAME_LEN];
        build_frame(frame, ch);
        RxIbus_FeedBuffer(&st, frame, IBUS_FRAME_LEN, 1000U);
        CHECK(!RxIbus_CheckFailsafe(&st, 1010U));
        CHECK(RxIbus_GetLinkStatus(&st) == IBUS_LINK_OK);

        /* "link putus" versi FlySky: throttle jatuh ke nilai failsafe TX,
         * tapi frame TETAP mengalir terus (checksum tetap valid) -- ini
         * skenario yang TIDAK akan tertangkap timeout murni. */
        ch[2] = 1005U; /* dalam toleransi 20 dari IBUS_CH_VALUE_MIN=1000 */
        build_frame(frame, ch);
        RxIbus_FeedBuffer(&st, frame, IBUS_FRAME_LEN, 1020U); /* jauh di bawah timeout 500ms */

        bool edge = RxIbus_CheckFailsafe(&st, 1030U);
        CHECK(edge);
        CHECK(RxIbus_GetLinkStatus(&st) == IBUS_LINK_LOST);

        /* pulih: throttle kembali normal di frame berikutnya */
        ch[2] = 1500U;
        build_frame(frame, ch);
        RxIbus_FeedBuffer(&st, frame, IBUS_FRAME_LEN, 1040U);
        CHECK(!RxIbus_CheckFailsafe(&st, 1050U));
        CHECK(RxIbus_GetLinkStatus(&st) == IBUS_LINK_OK);
    }

    /* -------- 6) Mekanisme #3: bit-flag per-channel (channel 0-5) ------- */
    {
        IBUS_State_t st;
        RxIbus_Init(&st);

        uint16_t ch[IBUS_NUM_CHANNELS];
        default_channels(ch);
        ch[4] |= IBUS_CH_FAILSAFE_FLAG_BIT; /* arm-switch channel ditandai failsafe oleh RX */
        uint8_t frame[IBUS_FRAME_LEN];
        build_frame(frame, ch);
        RxIbus_FeedBuffer(&st, frame, IBUS_FRAME_LEN, 2000U);

        const IBUS_ChannelData_t *out = RxIbus_GetChannels(&st);
        CHECK(out->channel_failsafe_flag[4]);
        CHECK(out->channel[4] == IBUS_CH_VALUE_MID); /* nilai data tetap bersih, flag terpisah */

        bool edge = RxIbus_CheckFailsafe(&st, 2010U);
        CHECK(edge);
        CHECK(RxIbus_GetLinkStatus(&st) == IBUS_LINK_LOST);

        /* channel 6-13 (di luar rentang yang dipakai main.c) SENGAJA tidak
         * dicek mekanisme #3 -- flag di channel index 8 misalnya, tidak
         * boleh memicu apa pun. */
        IBUS_State_t st2;
        RxIbus_Init(&st2);
        uint16_t ch2[IBUS_NUM_CHANNELS];
        default_channels(ch2);
        ch2[8] |= IBUS_CH_FAILSAFE_FLAG_BIT;
        uint8_t frame2[IBUS_FRAME_LEN];
        build_frame(frame2, ch2);
        RxIbus_FeedBuffer(&st2, frame2, IBUS_FRAME_LEN, 2000U);
        CHECK(!RxIbus_CheckFailsafe(&st2, 2010U));
        CHECK(RxIbus_GetLinkStatus(&st2) == IBUS_LINK_OK);
    }

    /* ---------------- 7) RxIbus_Update() trigger RTH tepat sekali ---------------- */
    {
        IBUS_State_t st;
        RxIbus_Init(&st);
        uint16_t ch[IBUS_NUM_CHANNELS];
        default_channels(ch);
        uint8_t frame[IBUS_FRAME_LEN];
        build_frame(frame, ch);
        RxIbus_FeedBuffer(&st, frame, IBUS_FRAME_LEN, 0U);

        g_rth_calls = 0;
        RxIbus_Update(&st, IBUS_FAILSAFE_TIMEOUT_MS, rth_trigger, NULL);
        CHECK(g_rth_calls == 1);
        RxIbus_Update(&st, IBUS_FAILSAFE_TIMEOUT_MS + 100U, rth_trigger, NULL);
        CHECK(g_rth_calls == 1); /* tidak dipanggil ulang selama masih LOST */

        /* pulih -> frame baru normal, lalu putus lagi -> RTH boleh trigger lagi */
        RxIbus_FeedBuffer(&st, frame, IBUS_FRAME_LEN, IBUS_FAILSAFE_TIMEOUT_MS + 200U);
        RxIbus_Update(&st, IBUS_FAILSAFE_TIMEOUT_MS + 210U, rth_trigger, NULL);
        CHECK(g_rth_calls == 1);
        RxIbus_Update(&st, IBUS_FAILSAFE_TIMEOUT_MS + 210U + IBUS_FAILSAFE_TIMEOUT_MS, rth_trigger, NULL);
        CHECK(g_rth_calls == 2);
    }

    /* ------- 8) REGRESI: RTH tidak boleh ter-trigger sebelum ada link ------- */
    {
        /* Ini persis kondisi boot: RxIbus_Init() lalu scheduler mulai jalan
         * beberapa detik kemudian (kalibrasi boot yang blocking), TANPA satu
         * frame pun pernah masuk karena transmitter belum dinyalakan.
         * Sebelum perbaikan, panggilan Update() pertama melihat transisi
         * OK -> LOST dan memanggil Nav_TriggerRTH(). */
        IBUS_State_t st;
        RxIbus_Init(&st);

        g_rth_calls = 0;
        RxIbus_Update(&st, 5000U, rth_trigger, NULL);
        CHECK(g_rth_calls == 0);
        /* Status link TETAP harus jadi LOST -- itu yang menahan arming
         * lewat rx_link_ok di main.c; yang dicegah hanya aksi RTH-nya. */
        CHECK(RxIbus_GetLinkStatus(&st) == IBUS_LINK_LOST);

        /* Dipanggil terus pun tetap tidak memicu apa pun. */
        for (int i = 0; i < 50; i++) {
            RxIbus_Update(&st, 5000U + (uint32_t)i * 20U, rth_trigger, NULL);
        }
        CHECK(g_rth_calls == 0);

        /* Begitu transmitter akhirnya dinyalakan dan frame masuk, lalu link
         * benar-benar putus, RTH HARUS tetap jalan seperti biasa. */
        uint16_t ch8[IBUS_NUM_CHANNELS];
        default_channels(ch8);
        uint8_t frame8[IBUS_FRAME_LEN];
        build_frame(frame8, ch8);
        RxIbus_FeedBuffer(&st, frame8, IBUS_FRAME_LEN, 6000U);
        RxIbus_Update(&st, 6010U, rth_trigger, NULL);
        CHECK(g_rth_calls == 0);                 /* link sehat, belum apa-apa */
        RxIbus_Update(&st, 6010U + IBUS_FAILSAFE_TIMEOUT_MS, rth_trigger, NULL);
        CHECK(g_rth_calls == 1);                 /* link-loss sungguhan */
    }

    TEST_SUMMARY("test_rx_ibus");
}
