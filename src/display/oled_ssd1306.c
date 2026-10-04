/**
 * oled_ssd1306.c
 *
 * Lihat oled_ssd1306.h untuk kontrak publik & alasan desain (framebuffer
 * + flush 1 page per tick).
 *
 * Struktur file:
 *   [1] Transport I2C           -- satu-satunya bagian yang menyentuh bus
 *   [2] Framebuffer & primitif  -- pixel, kotak, garis (tanpa I2C)
 *   [3] Font & teks             -- 5x7, skala integer, opsi miring (shear)
 *   [4] Layar                   -- splash / disarmed / armed
 *   [5] API publik
 *
 * Bagian [2]-[4] TIDAK memanggil I2C sama sekali, jadi bisa dikompilasi
 * dan diuji di PC (dengan OLED_HOST_TEST didefinisikan, transport diganti
 * stub) -- itulah cara tampilan diverifikasi tanpa hardware.
 */

#include "oled_ssd1306.h"
#include <stdio.h>
#include <string.h>

#ifndef OLED_HOST_TEST
#include "bsp_i2c.h"
#endif

#define OLED_CTRL_CMD   (0x00u)
#define OLED_CTRL_DATA  (0x40u)

/* ===========================================================================
 * [1] Transport I2C
 * =========================================================================*/

#ifdef OLED_HOST_TEST
/* Di PC: tidak ada bus. Hook ini diisi test harness. */
extern int host_i2c_write(const uint8_t *buf, uint16_t len);
extern void host_bus_recovery(void);
#define I2C_WRITE(buf, len)   host_i2c_write((buf), (len))
#define I2C_OK                (0)
#define I2C_RECOVER()         host_bus_recovery()
#else
#define I2C_WRITE(buf, len)   BSP_I2C1_Write(OLED_I2C_ADDR, (buf), (len))
#define I2C_OK                (BSP_I2C_OK)
#define I2C_RECOVER()         ((void)BSP_I2C1_BusRecovery())
#endif

static OLED_Status_t i2c_write(uint8_t ctrl_byte, const uint8_t *payload, uint16_t len)
{
    /* Frame I2C: [ctrl_byte][payload...] -- bsp_i2c.c menambahkan
     * start/addr/stop. 130 cukup untuk 1 page (128) + ctrl byte. */
    uint8_t buf[OLED_WIDTH + 2u];
    if (len > (sizeof(buf) - 1u)) {
        return OLED_ERR_I2C;
    }
    buf[0] = ctrl_byte;
    memcpy(&buf[1], payload, len);

    if (I2C_WRITE(buf, (uint16_t)(len + 1u)) != I2C_OK) {
        I2C_RECOVER();
        return OLED_ERR_I2C;
    }
    return OLED_OK;
}

static OLED_Status_t write_cmd(uint8_t cmd)
{
    return i2c_write(OLED_CTRL_CMD, &cmd, 1);
}

static OLED_Status_t set_position(uint8_t page, uint8_t col)
{
    OLED_Status_t st;

    st = write_cmd((uint8_t)(0xB0 | (page & 0x0Fu)));         /* page start   */
    if (st != OLED_OK) return st;
    st = write_cmd((uint8_t)(0x00 | (col & 0x0Fu)));          /* col low nib  */
    if (st != OLED_OK) return st;
    return write_cmd((uint8_t)(0x10 | ((col >> 4) & 0x0Fu))); /* col high nib */
}

/* ===========================================================================
 * [2] Framebuffer & primitif gambar
 * =========================================================================*/

/* Layout SSD1306: s_fb[page][col], bit0 = baris pixel paling atas page. */
static uint8_t s_fb[OLED_PAGES][OLED_WIDTH];

static void fb_clear(void)
{
    memset(s_fb, 0, sizeof(s_fb));
}

static void fb_pixel(int x, int y, bool on)
{
    if (x < 0 || y < 0 || x >= (int)OLED_WIDTH || y >= (int)OLED_HEIGHT) {
        return; /* clip diam-diam: teks/bentuk boleh melewati tepi layar */
    }
    uint8_t mask = (uint8_t)(1u << (y & 7));
    if (on) {
        s_fb[y >> 3][x] |= mask;
    } else {
        s_fb[y >> 3][x] &= (uint8_t)~mask;
    }
}

static void fb_fill_rect(int x, int y, int w, int h, bool on)
{
    for (int j = 0; j < h; j++) {
        for (int i = 0; i < w; i++) {
            fb_pixel(x + i, y + j, on);
        }
    }
}

static void fb_hline(int x, int y, int w)
{
    fb_fill_rect(x, y, w, 1, true);
}

/* ===========================================================================
 * [3] Font 5x7 & teks
 * =========================================================================*/

/* ASCII 32..90 berurutan (indeks = c - 32), 5 byte/glyph, satu byte per
 * KOLOM, bit0 = baris atas. Digenerate dari bitmap baris supaya bisa
 * dicek mata -- karakter di luar rentang / tidak didefinisikan = kosong. */
#define FONT_FIRST 32
#define FONT_LAST  90
static const uint8_t s_font[FONT_LAST - FONT_FIRST + 1][5] = {
#include "font_table.inc"
};

static char to_upper(char c)
{
    return (c >= 'a' && c <= 'z') ? (char)(c - 'a' + 'A') : c;
}

static const uint8_t *glyph_of(char c)
{
    c = to_upper(c);
    if (c < FONT_FIRST || c > FONT_LAST) {
        c = ' ';
    }
    return s_font[c - FONT_FIRST];
}

/**
 * Gambar satu glyph 5x7 dengan skala integer `scale`.
 * `slant_num/slant_den` = kemiringan (shear): tiap baris pixel glyph
 * digeser ke KANAN sebanyak (jumlah_baris_dari_bawah * num / den) pixel,
 * sehingga bagian atas condong ke kanan -- efek huruf miring (italic)
 * tanpa perlu font italic terpisah. slant_num=0 -> tegak.
 */
static void draw_glyph(int x, int y, char c, int scale, int slant_num, int slant_den)
{
    const uint8_t *g = glyph_of(c);
    const int glyph_h = 7 * scale;

    for (int col = 0; col < 5; col++) {
        for (int row = 0; row < 7; row++) {
            if (!(g[col] & (1u << row))) {
                continue;
            }
            for (int sy = 0; sy < scale; sy++) {
                int py = y + row * scale + sy;
                /* jarak dari dasar glyph (0 = paling bawah) */
                int from_bottom = (glyph_h - 1) - (row * scale + sy);
                int shear = (slant_den > 0) ? (from_bottom * slant_num) / slant_den : 0;
                for (int sx = 0; sx < scale; sx++) {
                    fb_pixel(x + col * scale + sx + shear, py, true);
                }
            }
        }
    }
}

/** Lebar teks (pixel) untuk skala tertentu: 5 pixel glyph + 1 spasi. */
static int text_width(const char *s, int scale)
{
    int n = (int)strlen(s);
    return (n <= 0) ? 0 : (n * 6 * scale) - scale;
}

static void draw_text(int x, int y, const char *s, int scale, int slant_num, int slant_den)
{
    for (; *s != '\0'; s++) {
        draw_glyph(x, y, *s, scale, slant_num, slant_den);
        x += 6 * scale;
    }
}

/** Teks kecil (skala 1, tegak) -- yang paling sering dipakai. */
static void draw_small(int x, int y, const char *s)
{
    draw_text(x, y, s, 1, 0, 1);
}

static void draw_small_centered(int y, const char *s)
{
    draw_small(((int)OLED_WIDTH - text_width(s, 1)) / 2, y, s);
}

/* ===========================================================================
 * [4] Layar
 * =========================================================================*/

const char *OLED_ModeName(OLED_FlightMode_t mode)
{
    switch (mode) {
        case OLED_MODE_PASSTHROUGH: return "MANUAL";
        case OLED_MODE_ASSISTED:    return "ASSIST";
        case OLED_MODE_ALT_HOLD:    return "ALT HOLD";
        case OLED_MODE_WAYPOINT:    return "WAYPOINT";
        case OLED_MODE_RTH:         return "RTH";
        case OLED_MODE_RTH_NO_GPS:  return "RTH NOGPS";
        default:                    return "?";
    }
}

/* --- Layar 1: splash "FAAS" -------------------------------------------- */

static void screen_splash(void)
{
    /* Font besar (skala 5 -> glyph 25x35 px) + miring 1:4 (lebih halus
     * dari 1:3). Lebar teks = 4*30-5 = 115 px; geser miring maksimum
     * = 34/4 = 8 px, jadi total ~123 px. Bagian atas huruf paling KANAN
     * yang bergeser, sehingga ruang kosong di kiri dan kanan harus
     * dihitung dari lebar TOTAL termasuk geser -- kalau hanya dari lebar
     * teks, huruf terakhir terpotong di tepi layar (terlihat di render
     * uji versi pertama). */
    const int scale = 5;
    const int slant_num = 1, slant_den = 4;
    const char *brand = "FAAS";

    int shear_max = ((7 * scale - 1) * slant_num) / slant_den;
    int total_w = text_width(brand, scale) + shear_max;
    int x = ((int)OLED_WIDTH - total_w) / 2;
    if (x < 0) x = 0;

    draw_text(x, 3, brand, scale, slant_num, slant_den);

    /* garis bawah tipis + tagline kecil, agar splash tidak polos */
    fb_hline(12, 44, (int)OLED_WIDTH - 24);
    draw_small_centered(50, "FLIGHT CONTROLLER");
}

/* --- Layar 2: DISARMED + checklist syarat ------------------------------- */

typedef struct {
    uint8_t     bit;
    const char *label;
} PrearmItem_t;

static const PrearmItem_t s_prearm_items[] = {
    { OLED_PRE_RX,    "RX LINK"   },
    { OLED_PRE_IMU,   "IMU / AHRS" },
    { OLED_PRE_CALIB, "KALIBRASI" },
    { OLED_PRE_THR,   "THROTTLE LOW" },
};
#define PREARM_COUNT (sizeof(s_prearm_items) / sizeof(s_prearm_items[0]))

/** Kotak centang 7x7: terisi + tanda centang = OK, kotak kosong = belum. */
static void draw_checkbox(int x, int y, bool ok)
{
    /* bingkai */
    fb_hline(x, y, 7);
    fb_hline(x, y + 6, 7);
    fb_fill_rect(x, y, 1, 7, true);
    fb_fill_rect(x + 6, y, 1, 7, true);
    if (ok) {
        /* tanda centang di dalam: (1,3)-(2,4)-(5,1) */
        fb_pixel(x + 1, y + 3, true);
        fb_pixel(x + 2, y + 4, true);
        fb_pixel(x + 3, y + 3, true);
        fb_pixel(x + 4, y + 2, true);
        fb_pixel(x + 5, y + 1, true);
    }
}

static void screen_disarmed(uint32_t now_ms, const OLED_View_t *v)
{
    /* Header: "UNARMED" -- kiri, dengan titik heartbeat kedip di kanan
     * supaya kelihatan layar tidak hang. */
    draw_small(0, 0, "UNARMED");
    if ((now_ms / 500u) & 1u) {
        fb_fill_rect((int)OLED_WIDTH - 4, 2, 3, 3, true);
    }
    fb_hline(0, 9, (int)OLED_WIDTH);

    /* Ringkasan: jumlah syarat terpenuhi, di sisi kanan header. */
    int ok_count = 0;
    for (unsigned i = 0; i < PREARM_COUNT; i++) {
        if (v->prearm_ok_mask & s_prearm_items[i].bit) {
            ok_count++;
        }
    }
    char summary[4];
    summary[0] = (char)('0' + ok_count);
    summary[1] = '/';
    summary[2] = (char)('0' + (int)PREARM_COUNT);
    summary[3] = '\0';
    draw_small((int)OLED_WIDTH - text_width(summary, 1) - 9, 0, summary);

    /* Checklist: 4 baris, pitch 10 px, mulai y=13 -> muat sampai y=52. */
    for (unsigned i = 0; i < PREARM_COUNT; i++) {
        bool ok = (v->prearm_ok_mask & s_prearm_items[i].bit) != 0u;
        int y = 13 + (int)i * 10;
        draw_checkbox(2, y, ok);
        draw_small(14, y, s_prearm_items[i].label);

        /* Status teks di kanan supaya terbaca tanpa harus melihat kotak.
         *
         * Baris KALIBRASI punya satu status TAMBAHAN di luar OK/BELUM:
         * selama task pemulihan kalibrasi berjalan, "BELUM" benar tapi
         * tidak berguna -- pengguna tidak tahu bahwa yang perlu dia
         * lakukan hanyalah MENAHAN PAPAN DIAM, dan bahwa prosesnya
         * sedang berjalan. "DIAM NN%" mengatakan keduanya sekaligus.
         *
         * Anggaran lebar (font 5x7 + 1 px spasi = 6 px/karakter):
         * label "KALIBRASI" mulai x=14, 9 karakter -> berakhir di x=67.
         * "DIAM 100%" 9 karakter = 53 px, rata kanan -> mulai x=74.
         * Masih ada 7 px jarak, jadi tidak bertabrakan. */
        char buf[10];
        const char *st;
        if (!ok && s_prearm_items[i].bit == OLED_PRE_CALIB && v->calib_running) {
            unsigned pct = (v->calib_progress_pct > 100u) ? 100u
                                                          : v->calib_progress_pct;
            snprintf(buf, sizeof(buf), "DIAM %u%%", pct);
            st = buf;
        } else {
            st = ok ? "OK" : "BELUM";
        }
        draw_small((int)OLED_WIDTH - text_width(st, 1) - 1, y, st);
    }
}

/* --- Layar 3: ARMED ------------------------------------------------------ */

/** Format float -> teks tanpa printf-float (newlib-nano biasanya tanpa
 *  dukungan %f). Nilai di-CLAMP ke +-999.9 supaya lebar teks selalu
 *  terbatas (maks 6 karakter, mis. "-999.9") dan layout kolom tidak
 *  pecah walau sensor mengeluarkan nilai ngawur / NaN. NaN gagal semua
 *  perbandingan, jadi ditangani eksplisit -> "--". */
static void fmt_f1(char *out, size_t n, float v)
{
    if (!(v == v)) {            /* NaN */
        snprintf(out, n, "--");
        return;
    }
    if (v >  999.9f) v =  999.9f;
    if (v < -999.9f) v = -999.9f;
    bool neg = (v < 0.0f);
    if (neg) v = -v;
    int tenths = (int)(v * 10.0f + 0.5f);
    snprintf(out, n, "%s%d.%d", neg ? "-" : "", tenths / 10, tenths % 10);
}

static void screen_armed(uint32_t now_ms, const OLED_View_t *v)
{
    char a[16], b[16], line[32];

    /* --- Baris paling atas (kecil): "ARMED" + heartbeat --- */
    draw_small(0, 0, "ARMED");
    if ((now_ms / 500u) & 1u) {
        fb_fill_rect((int)OLED_WIDTH - 4, 2, 3, 3, true);
    }
    fb_hline(0, 9, (int)OLED_WIDTH);

    /* --- Bagian tengah: pembacaan sensor, 4 baris pitch 10 px (y=13..43).
     * Dua kolom: kiri = attitude/navigasi, kanan = daya. Kolom kanan
     * rata-kanan. Tiap baris sengaja pendek supaya kolom kiri (maks ~13
     * karakter = 78 px) dan kanan (maks ~7 karakter = 42 px) tidak
     * bertabrakan dalam 128 px. */

    /* baris 1: roll | tegangan */
    fmt_f1(a, sizeof(a), v->roll_deg);
    snprintf(line, sizeof(line), "ROLL %s", a);
    draw_small(0, 13, line);
    fmt_f1(a, sizeof(a), v->battery_v);
    snprintf(line, sizeof(line), "%sV", a);
    draw_small((int)OLED_WIDTH - text_width(line, 1), 13, line);

    /* baris 2: pitch | arus */
    fmt_f1(a, sizeof(a), v->pitch_deg);
    snprintf(line, sizeof(line), "PITCH %s", a);
    draw_small(0, 23, line);
    fmt_f1(a, sizeof(a), v->current_a);
    snprintf(line, sizeof(line), "%sA", a);
    draw_small((int)OLED_WIDTH - text_width(line, 1), 23, line);

    /* baris 3: heading | altitude */
    int hdg = 0;
    if (v->yaw_deg == v->yaw_deg) {                    /* bukan NaN */
        float yy = v->yaw_deg;
        if (yy >  3600.0f) yy =  3600.0f;              /* batasi loop wrap */
        if (yy < -3600.0f) yy = -3600.0f;
        hdg = (int)(yy + (yy < 0.0f ? -0.5f : 0.5f));
        while (hdg < 0)    hdg += 360;
        while (hdg >= 360) hdg -= 360;
    }
    snprintf(line, sizeof(line), "HDG %03d", hdg);
    draw_small(0, 33, line);
    fmt_f1(a, sizeof(a), v->altitude_m);
    snprintf(line, sizeof(line), "%sM", a);
    draw_small((int)OLED_WIDTH - text_width(line, 1), 33, line);

    /* baris 4: kecepatan | GPS */
    fmt_f1(b, sizeof(b), v->ground_speed_ms);
    snprintf(line, sizeof(line), "GS %s", b);   /* maks "GS 999.9" = 8 char = 47 px */
    draw_small(0, 43, line);
    const char *fix;
    switch (v->gps_fix) {
        case OLED_GPS_3D: fix = "3D"; break;
        case OLED_GPS_2D: fix = "2D"; break;
        case OLED_GPS_DR: fix = "DR"; break;
        default:          fix = "NO"; break;
    }
    snprintf(line, sizeof(line), "GPS %s/%u", fix, (unsigned)v->gps_sat_count);
    draw_small((int)OLED_WIDTH - text_width(line, 1), 43, line);

    /* --- Baris paling bawah (kecil): mode terbang --- */
    fb_hline(0, 54, (int)OLED_WIDTH);
    snprintf(line, sizeof(line), "MODE %s", OLED_ModeName(v->mode));
    draw_small(0, 56, line);
}

/* ===========================================================================
 * [5] API publik
 * =========================================================================*/

static uint32_t s_boot_ms;
static uint8_t  s_next_chunk;      /* chunk (setengah page) berikutnya    */
static bool     s_splash_done;     /* sudah berpindah keluar dari splash? */
static bool     s_splash_sent;     /* framebuffer splash sudah terkirim penuh? */

/* Satu "chunk" = setengah page (64 kolom). Dipakai OLED_Update() supaya
 * satu tick I2C ~1.6 ms, bukan ~3.2 ms -- task "ctrl" 200 Hz (periode
 * 5 ms) hanya bisa ditunda separuhnya oleh OLED. Total 16 chunk per
 * layar penuh. */
#define OLED_CHUNK_COLS   (OLED_WIDTH / 2u)
#define OLED_CHUNKS       (OLED_PAGES * 2u)

static OLED_Status_t flush_chunk(uint8_t chunk)
{
    uint8_t page = (uint8_t)(chunk >> 1);
    uint8_t col  = (uint8_t)((chunk & 1u) ? OLED_CHUNK_COLS : 0u);

    OLED_Status_t st = set_position(page, col);
    if (st != OLED_OK) return st;
    return i2c_write(OLED_CTRL_DATA, &s_fb[page][col], OLED_CHUNK_COLS);
}

static OLED_Status_t flush_page(uint8_t page)
{
    OLED_Status_t st = flush_chunk((uint8_t)(page * 2u));
    if (st != OLED_OK) return st;
    return flush_chunk((uint8_t)(page * 2u + 1u));
}

static OLED_Status_t flush_all(void)
{
    for (uint8_t p = 0; p < OLED_PAGES; p++) {
        OLED_Status_t st = flush_page(p);
        if (st != OLED_OK) return st;
    }
    return OLED_OK;
}

/**
 * Satu kali percobaan penuh init sequence. Dipanggil dari retry loop
 * OLED_Init() -- kalau gagal di tengah harus mengulang dari 0xAE, bukan
 * lanjut dari command yang gagal (state register SSD1306 konsisten).
 */
static OLED_Status_t oled_init_sequence(void)
{
    static const uint8_t init_cmds[] = {
        0xAE,       /* display off */
        0xD5, 0x80, /* clock divide ratio / osc freq */
        0xA8, 0x3F, /* multiplex ratio = 63 (128x64) */
        0xD3, 0x00, /* display offset = 0 */
        0x40,       /* display start line = 0 */
        0x8D, 0x14, /* charge pump enable */
        0x20, 0x00, /* memory addressing mode = horizontal */
        0xA1,       /* segment remap (mirror X) */
        0xC8,       /* COM output scan direction (mirror Y) */
        0xDA, 0x12, /* COM pins hardware config, 128x64 */
        0x81, 0x7F, /* contrast, mid-level */
        0xD9, 0xF1, /* pre-charge period */
        0xDB, 0x40, /* VCOMH deselect level */
        0xA4,       /* resume RAM content display */
        0xA6,       /* normal display (not inverted) */
        0xAF        /* display on */
    };

    for (uint16_t i = 0; i < sizeof(init_cmds); i++) {
        OLED_Status_t st = write_cmd(init_cmds[i]);
        if (st != OLED_OK) {
            return st;
        }
    }
    return OLED_OK;
}

OLED_Status_t OLED_Init(uint32_t now_ms)
{
    OLED_Status_t st = OLED_ERR_I2C;

    /* Kenapa retry-from-scratch (root cause, dipertahankan dari versi
     * lama): I2C1 dipakai bersama 4 modul breakout, masing-masing membawa
     * pull-up sendiri. Paralel di satu bus, resistansi gabungan jauh di
     * bawah asumsi CCR/TRISE bsp_i2c.c untuk 400kHz -> rise-time melar.
     * OLED device terakhir di urutan init, jadi paling sering kena. Satu
     * command gagal tidak boleh membuat OLED gagal menyala -- ulang dari
     * 0xAE. i2c_write() sudah memanggil bus recovery tiap kegagalan. */
    for (uint8_t attempt = 0; attempt < OLED_INIT_MAX_RETRIES; attempt++) {
        st = oled_init_sequence();
        if (st == OLED_OK) {
            break;
        }
    }
    if (st != OLED_OK) {
        return st; /* OLED_ERR_I2C setelah OLED_INIT_MAX_RETRIES gagal */
    }

    /* Splash digambar ke framebuffer lalu dikirim PENUH (blocking ~25
     * ms, sekali saja di boot -- scheduler belum jalan, jadi aman).
     *
     * Kegagalan kirim di sini SENGAJA tidak membatalkan init: command
     * sequence di atas sudah sukses (panel menyala), jadi yang gagal
     * hanya isi gambarnya. Membatalkan seluruh init karena satu chunk
     * gagal akan membuat has_oled=false dan OLED mati selamanya sampai
     * reboot -- padahal bus I2C1 yang kena loading (lihat komentar di
     * atas) memang sesekali gagal. Untuk itu splash ditandai belum
     * terkirim (s_splash_sent=false) dan OLED_Update() akan mengirim
     * ulang lewat jalur chunk normal yang sudah punya retry. */
    fb_clear();
    screen_splash();
    s_splash_sent = (flush_all() == OLED_OK);

    s_boot_ms     = now_ms;
    s_next_chunk  = 0;
    s_splash_done = false;
    return OLED_OK;
}

OLED_Status_t OLED_Clear(void)
{
    fb_clear();
    return flush_all();
}

OLED_Status_t OLED_Update(uint32_t now_ms, const OLED_View_t *view)
{
    if (view == NULL) {
        return OLED_ERR_I2C;
    }

    /* Splash: tidak digambar ulang -- framebuffer sudah berisi logo dari
     * OLED_Init() dan sudah terkirim penuh. Tinggal menunggu waktunya. */
    if (!s_splash_done) {
        if ((now_ms - s_boot_ms) < OLED_SPLASH_MS) {
            /* Splash gagal terkirim saat Init: kirim ulang chunk demi
             * chunk sampai lengkap (framebuffer masih berisi logo
             * karena belum ada yang menimpanya). */
            if (!s_splash_sent) {
                OLED_Status_t rst = flush_chunk(s_next_chunk);
                if (rst != OLED_OK) {
                    return rst;
                }
                s_next_chunk = (uint8_t)((s_next_chunk + 1u) % OLED_CHUNKS);
                if (s_next_chunk == 0) {
                    s_splash_sent = true;
                }
            }
            return OLED_OK;
        }
        s_splash_done = true;
        s_next_chunk  = 0;
    }

    /* Render ke framebuffer hanya saat mulai siklus baru (chunk 0), supaya
     * seluruh 16 chunk dalam satu siklus berasal dari SATU snapshot data --
     * tanpa ini, layar bisa menampilkan separuh atas data lama dan
     * separuh bawah data baru (tearing). */
    static OLED_View_t s_frame_view;
    static uint32_t    s_frame_ms;
    if (s_next_chunk == 0) {
        s_frame_view = *view;
        s_frame_ms   = now_ms;

        fb_clear();
        if (s_frame_view.armed) {
            screen_armed(s_frame_ms, &s_frame_view);
        } else {
            screen_disarmed(s_frame_ms, &s_frame_view);
        }
    }

    OLED_Status_t st = flush_chunk(s_next_chunk);
    if (st != OLED_OK) {
        return st; /* chunk sama dicoba lagi di panggilan berikutnya */
    }
    s_next_chunk = (uint8_t)((s_next_chunk + 1u) % OLED_CHUNKS);
    return OLED_OK;
}
