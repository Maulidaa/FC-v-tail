/**
 * oled_ssd1306.h
 *
 * Driver + UI OLED SSD1306 (status display on-board) untuk I2C1:
 *   - Bus   : I2C1 (PB6=SCL, PB7=SDA, AF4)
 *   - Addr  : 0x3C (7-bit)
 *   - Panel : 128x64 (ubah OLED_HEIGHT + init sequence kalau 128x32)
 *
 * === UI interaktif: 3 layar ===
 *   1. BOOT     : logo "FAAS" font besar miring (splash).
 *   2. DISARMED : judul "UNARMED" + checklist syarat arming, tiap syarat
 *                 bertanda OK / belum terpenuhi.
 *   3. ARMED    : "ARMED" kecil di baris paling atas, MODE terbang kecil
 *                 di baris paling bawah, bagian tengah = pembacaan sensor.
 *
 * === Kenapa framebuffer + flush 1 page per tick ===
 * BSP_I2C1_Write() itu busy-wait, dan I2C1 dibagi dengan MPU6050/BMP280/
 * mag. Mengirim 8 page x 129 byte @400kHz ~ 25 ms dalam satu panggilan
 * task akan membuat task "ctrl" (200 Hz, periode 5 ms) overrun dan
 * jitter di seluruh scheduler. Maka:
 *   - Semua gambar dirender ke framebuffer RAM (1 KB), TANPA I2C.
 *   - OLED_Update() mengirim HANYA SETENGAH PAGE (64 byte, ~1.6 ms) per
 *     panggilan, berputar 16 chunk. Satu layar penuh = 16 panggilan.
 *     Setengah page (bukan satu page utuh ~3.2 ms) dipilih supaya task
 *     "ctrl" 200 Hz (periode 5 ms) paling parah hanya tertunda ~1.6 ms.
 * Karena itu OLED_Update() harus dipanggil cukup sering (RATE_OLED_HZ
 * di main.c): 16 chunk x 3 Hz ~ 48 Hz -> refresh layar penuh ~3x/detik.
 *
 * Font: 5x7 ASCII (spasi, digit, huruf KAPITAL, simbol dasar). Huruf
 * kecil otomatis dirender sebagai kapital.
 */

#ifndef OLED_SSD1306_H
#define OLED_SSD1306_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OLED_I2C_ADDR   (0x3Cu)
#define OLED_WIDTH      (128u)
#define OLED_HEIGHT     (64u)
#define OLED_PAGES      (OLED_HEIGHT / 8u)

/* Jumlah maksimal percobaan OLED_Init() (percobaan pertama + retry).
 * Lihat catatan root-cause "4 modul breakout -> pull-up paralel -> bus
 * loading" di OLED_Init() (oled_ssd1306.c). */
#define OLED_INIT_MAX_RETRIES (3u)

/** Lama splash "FAAS" tampil (ms), dihitung sejak OLED_Init() sukses.
 *  Kalibrasi boot yang blocking terjadi SESUDAH init OLED, jadi logo
 *  tetap terlihat selama kalibrasi (framebuffer sudah dikirim penuh di
 *  OLED_Init()), dan langsung berganti begitu task OLED pertama jalan
 *  kalau waktunya sudah lewat. */
#define OLED_SPLASH_MS (2500u)

typedef enum {
    OLED_OK = 0,
    OLED_ERR_I2C
} OLED_Status_t;

/** GPS fix type. Urutan sengaja disamakan dengan GPS_FixType_t. */
typedef enum {
    OLED_GPS_NONE = 0,
    OLED_GPS_DR   = 1,
    OLED_GPS_2D   = 2,
    OLED_GPS_3D   = 3
} OLED_GpsFix_t;

/** Mode terbang yang ditampilkan di baris bawah layar ARMED. */
typedef enum {
    OLED_MODE_PASSTHROUGH = 0,   /* full manual                      */
    OLED_MODE_ASSISTED,          /* stabilize PID, stick = setpoint  */
    OLED_MODE_ALT_HOLD,          /* auto: altitude hold              */
    OLED_MODE_WAYPOINT,          /* auto: waypoint                   */
    OLED_MODE_RTH,               /* auto: return to home             */
    OLED_MODE_RTH_NO_GPS         /* auto: RTH fallback tanpa GPS fix */
} OLED_FlightMode_t;

/**
 * Bitmask syarat arming yang SUDAH terpenuhi (bit=1 -> OK). Diisi
 * caller (main.c) dari sumber yang sama dengan ArmedStateInputs_t,
 * supaya layar menampilkan SEMUA syarat sekaligus -- run_preflight()
 * di armed_state.c hanya melaporkan kegagalan PERTAMA, tidak cukup
 * untuk checklist lengkap.
 */
#define OLED_PRE_RX      (1u << 0)   /* link RX iBus hidup            */
#define OLED_PRE_IMU     (1u << 1)   /* attitude valid                */
#define OLED_PRE_CALIB   (1u << 2)   /* kalibrasi accel/gyro boot OK  */           
#define OLED_PRE_THR     (1u << 4)   /* throttle stick rendah         */
#define OLED_PRE_ALL     (OLED_PRE_RX | OLED_PRE_IMU | OLED_PRE_CALIB | \
                          OLED_PRE_THR)

/** Snapshot semua data yang dibutuhkan layar, satuan siap tampil. */
typedef struct {
    bool              armed;
    uint8_t           prearm_ok_mask;   /* kombinasi OLED_PRE_*          */
    OLED_FlightMode_t mode;

    float             roll_deg;
    float             pitch_deg;
    float             yaw_deg;          /* heading, 0..360               */
    float             altitude_m;
    float             battery_v;
    float             current_a;
    float             ground_speed_ms;
    OLED_GpsFix_t     gps_fix;
    uint8_t           gps_sat_count;

    /* Status kalibrasi accel/gyro yang sedang berjalan (task_calib_recover()
     * di main.c). Dipakai HANYA oleh baris KALIBRASI di checklist layar
     * DISARMED: saat calib_running true, baris itu menampilkan "DIAM NN%"
     * alih-alih "BELUM" -- pesannya ke pengguna adalah "tahan papan tetap
     * diam", karena itu satu-satunya hal yang perlu dia lakukan supaya
     * kalibrasi selesai. calib_progress_pct diabaikan kalau calib_running
     * false. Sumbernya CalibAccelGyro_GetProgressPercent(), 0..100. */
    bool              calib_running;
    uint8_t           calib_progress_pct;
} OLED_View_t;

/**
 * Inisialisasi panel (sequence command SSD1306), gambar splash "FAAS"
 * ke framebuffer lalu kirim penuh ke panel (blocking ~25 ms, SEKALI di
 * boot). Panggil setelah BSP_I2C1_Init(). `now_ms` = get_tick_ms(),
 * dipakai sebagai titik nol durasi splash.
 */
OLED_Status_t OLED_Init(uint32_t now_ms);

/** Hapus framebuffer + panel (langsung, blocking). */
OLED_Status_t OLED_Clear(void);

/**
 * Render satu frame ke FRAMEBUFFER (tanpa I2C, cepat) lalu kirim SATU
 * page ke panel. Panggil berkala dari task scheduler.
 *
 * Layar dipilih otomatis: splash selama OLED_SPLASH_MS sejak Init,
 * setelahnya DISARMED atau ARMED sesuai view->armed.
 */
OLED_Status_t OLED_Update(uint32_t now_ms, const OLED_View_t *view);

/** Nama pendek mode terbang (kapital, maks 8 karakter). */
const char *OLED_ModeName(OLED_FlightMode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* OLED_SSD1306_H */
