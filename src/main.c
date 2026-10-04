/**
 * @file    main.c
 * @brief   Boot sequence + scheduler cooperative untuk flightController.
 *
 * Perbedaan utama dari main.c sebelumnya:
 *   1. SystemClock_Config() dipanggil PALING AWAL. Tanpa ini MCU jalan di
 *      HSI 16MHz dan semua baud rate / CCR I2C / timing DShot salah ~6x.
 *   2. SysTick di-init dengan SYSTEM_CORE_CLOCK_HZ, bukan angka literal
 *      100000000 yang tidak cocok dengan clock tree manapun di proyek ini.
 *   3. Rantai kontrol benar-benar dijalankan: RX -> AHRS -> nav -> stabilize
 *      -> mixer -> output_map -> dshot/servo. Sebelumnya modul-modul itu
 *      ada tapi tidak pernah dipanggil dari mana pun.
 *   4. Independent watchdog aktif — kalau loop macet (mis. di busy-wait I2C
 *      tanpa timeout), MCU reset alih-alih terbang tanpa kontrol.
 *   5. Semua dt yang masuk ke PID dan AHRS adalah dt TERUKUR dari scheduler,
 *      bukan konstanta. Guard dt di pid.c dan ahrs_fusion.c baru ada gunanya
 *      kalau yang diberi memang dt terukur.
 *
 * === STATUS ITEM SEBELUMNYA (diperbarui) ===
 *   a. RESOLVED — bsp_timer_pwm.c sekarang menyediakan wrapper
 *      BSP_TimerPWM_ServoInit/SetPulseUs dan BSP_DShotTimer_Init/
 *      IsTransferBusy/SendFrameDMA di atas implementasi asli
 *      BSP_TIM_Servo_BSP_TIM_DShot_*, jadi dshot.c dan pwm_servo.c
 *      tidak perlu diubah.
 *   b. SEBAGIAN RESOLVED — Protocol_CRC8_DVBS2() SUDAH ada di
 *      comms/protocol.c (termasuk framing+CRC8), jadi blackbox.c
 *      tidak lagi meng-extern simbol yang tidak ada. Yang MASIH
 *      terbuka hanya: blackbox belum didaftarkan sebagai task di
 *      scheduler ini, dan armed_inputs.blackbox_ready masih
 *      hardcode true di task_control().
 *   c. RESOLVED — objects.list sekarang mencakup seluruh modul
 *      (control/output/rx/nav/fusion/power termasuk).
 *   d. RESOLVED — main.c sebelumnya meng-include "mag_compass.h" (modul
 *      dispatcher yang tidak pernah ditulis). Struktur folder tim
 *      (firmware-architecture-stm32f411.md Bagian 2) HANYA menyebut
 *      mag_hmc5883.c/h dan mag_qmc5883.c/h sebagai driver mag yang ada
 *      — jadi main.c di sini memanggil KEDUA driver itu langsung
 *      (coba MAG_HMC5883_Detect() dulu, fallback ke MAG_QMC5883_Detect()),
 *      tanpa menambah file dispatcher baru. Lihat s_active_mag_chip di
 *      bawah untuk state pemilihannya.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "system_clock.h"
#include "systick.h"
#include "scheduler.h"

#include "bsp_pinmap.h"
#include "bsp_spi.h"
#include "bsp_i2c.h"
#include "bsp_uart.h"
#include "bsp_adc.h"

#include "imu_mpu6500.h"
#include "baro_bmp280.h"
#include "mag_hmc5883.h"
#include "mag_qmc5883.h"
#include "gps_ubx.h"
#include "oled_ssd1306.h"
#include "calib_accel_gyro.h"
#include "calib_mag.h"
#include "calib_dispatcher.h"

#include "ahrs_fusion.h"
#include "compass.h"
#include "navigation.h"
#include "stabilize.h"
#include "telemetry.h"
#include "usb_cdc_if.h"
#include "rx_ibus.h"
#include "battery.h"
#include "dshot.h"
#include "pwm_servo.h"
#include "armed_state.h"
#include "protocol.h"
#include "command_handler.h"
#include "usb_cdc_if.h"

#include "calib_gate.h"

/* ===========================================================================
 * Konfigurasi laju task
 * ===========================================================================
 * Angka-angka ini adalah anggaran CPU, bukan keinginan. Naikkan hanya kalau
 * Scheduler_GetTotalOverruns() tetap 0 setelah diuji di bangku.
 *
 * Catatan kenapa IMU sekunder jauh lebih pelan dari primer: MPU6500 di SPI1
 * (cepat, non-blocking-ish), MPU6050 di I2C1 yang dibagi bersama BMP280,
 * magnetometer, dan OLED. Membaca 14 byte I2C di 400kHz memakan ~400us —
 * kalau dipaksa 1kHz, bus I2C saja sudah menghabiskan 40% waktu CPU.
 */
#define RATE_RX_DRAIN_HZ      1000U  /* kosongkan ring buffer UART1 (iBus 115200) */
#define RATE_PROTOCOL_HZ       1000U  /* proses command waypoint dari USB CDC */
#define RATE_IMU_PRIMARY_HZ   1000U  /* MPU6500 via SPI1                        */
#define RATE_AHRS_HZ           200U  /* sama dengan AHRS_FUSION_RATE_HZ         */
#define RATE_CONTROL_HZ        200U  /* PID + mixer + output                    */
#define RATE_DSHOT_HZ          500U  /* kirim frame ESC; ESC butuh sinyal kontinu */
#define RATE_FAILSAFE_HZ        50U  /* cek link-loss iBus                      */
#define RATE_BARO_HZ            25U  /* BMP280 oversampling butuh waktu konversi */
#define RATE_MAG_HZ             50U
#define RATE_GPS_HZ             10U  /* NAV-PVT u-blox biasanya 5-10Hz          */
#define RATE_NAV_HZ             10U
#define RATE_BATTERY_HZ         10U
#define RATE_TELEMETRY_HZ       10U  /* push ATTITUDE+GPS+BATTERY (Comms #4); jangan lebih cepat -- lihat catatan blocking TX di task_telemetry_push() */
#define RATE_OLED_HZ            48U  /* 1 chunk = 1/2 page (~1.6ms I2C) per tick; 16 tick = 1 layar penuh ~3Hz */
#define RATE_CALIB_RECOVER_HZ  200U  /* task pemulihan kalibrasi (C4): 200 Hz x
                                       * TARGET_SAMPLES=500 -> satu percobaan
                                       * kalibrasi ~2,5 detik. Lebih cepat dari
                                       * ini tidak berguna karena satu tick hanya
                                       * memberi SATU sample, dan lebih pelan
                                       * membuat pengguna menahan papan diam
                                       * lebih lama. */
#define ARM_THROTTLE_LOW_THRESHOLD 0.10f
/* KEPUTUSAN TIM: deadstick (Opsi 1), dikonfirmasi 2026-09-19 -- lihat
 * rancangan-mag-fusion-heading-rth.md §3.3. Throttle dipotong penuh ke 0
 * begitu masuk NAV_MODE_RTH maupun NAV_MODE_RTH_NO_GPS_FALLBACK: motor
 * mati total, pesawat glide tanpa tenaga menuju home/wings-level.
 * Alasan tim memilih ini atas "powered glide" (nilai tidak nol): tanpa
 * sensor airspeed, throttle cruise tetap butuh konstanta yang sulit
 * diverifikasi aman di segala kondisi angin/berat pada sesi uji terbang
 * pertama, sedangkan deadstick tidak butuh nilai yang perlu ditebak sama
 * sekali dan failure mode-nya (kehilangan sedikit altitude/jangkauan
 * saat glide) lebih dapat diprediksi. Efek samping yang perlu diingat:
 * dengan throttle=0 di jalur ini, kalau altitude-hold PID nanti dipasang
 * di RTH, PID JANGAN mencoba pertahankan altitude di sini (nose-up tanpa
 * tenaga = jalur menuju stall) -- lihat catatan amandemen pitch-netral
 * saat unpowered di rancangan-mag-fusion-heading-rth.md §3.5-3.6. */
#define RTH_FAILSAFE_THROTTLE       0.0f
#define RTH_NO_GPS_THROTTLE         0.0f
#define RTH_MAX_BANK_DEG            25.0f
#define RTH_HEADING_TO_BANK_GAIN    0.80f


#define GPS_STALE_MS      1000U  /* > 5x periode epoch (200 ms) */
#define GPS_CFG_RETRY_MS  5000U  /* tanpa frame selama ini -> kirim ulang config */

static uint32_t s_gps_last_frame_ms;
static bool     s_gps_seen;

/* ===========================================================================
 * Konversi satuan IMU
 * ===========================================================================
 * Driver MPU6500 mengembalikan int16 LSB mentah, sementara
 * AHRS_ImuSample_t mengharapkan float dalam rad/s (gyro) dan g (accel).
 * Layer konversi ini SEBELUMNYA TIDAK ADA di mana pun — akibatnya attitude
 * akan salah ribuan kali lipat begitu kedua modul disambungkan.
 *
 * Kedua chip dikonfigurasi identik di driver-nya:
 *   GYRO_CONFIG  = 0x08 -> +-500 dps  -> 65.5 LSB/dps
 *   ACCEL_CONFIG = 0x08 -> +-4 g      -> 8192 LSB/g
 * Kalau salah satu driver mengubah range-nya, KONSTANTA DI BAWAH WAJIB IKUT
 * BERUBAH. Idealnya driver mengekspos sensitivitasnya sendiri lewat header,
 * supaya tidak ada dua sumber kebenaran seperti sekarang.
 */
#define GYRO_LSB_PER_DPS     65.5f
#define ACCEL_LSB_PER_G      8192.0f
#define DEG_TO_RAD           0.01745329252f

/* Bias gyro & accel hasil kalibrasi (LSB), diisi dari calib_accel_gyro.c
 * saat boot (lihat run_boot_calibration() dekat main()) dengan pesawat
 * diam. Tanpa pengurangan bias, complementary filter akan melawan drift
 * konstan terus-menerus dan pitch/roll ikut meleset.
 *
 * KONTRAK accel_bias_lsb[2] (sumbu Z) BEDA dari [0]/[1] (X/Y): imu_raw_to_sample()
 * di bawah mengurangi ketiga sumbu dengan formula SAMA PERSIS, padahal
 * AccelGyroBias_t.accel_bias_z dari calib_accel_gyro.c adalah rata-rata
 * mentah yang MASIH mengandung +-1g gravitasi (lihat komentar
 * "bias di sekitar +1g" di calib_accel_gyro.h) -- BEDA dengan accel_bias_x/y
 * yang gravitasinya memang ~0 saat pesawat rata (jadi rata-rata mentahnya
 * sudah bias murni). Kalau accel_bias_lsb[2] diisi rata-rata mentah itu
 * apa adanya, imu_raw_to_sample() akan MEMBUANG SELURUH referensi
 * gravitasi di Z, merusak AccelToRollPitch() di ahrs_fusion.c (butuh accel_z
 * ~1g saat level untuk atan2f(accel_y, accel_z)). Field ini WAJIB diisi
 * SUDAH dikurangi ACCEL_LSB_PER_G oleh pengisi-nya (lihat pemakaian di
 * main(), bukan raw AccelGyroBias_t.accel_bias_z langsung. */
typedef struct {
    float gyro_bias_lsb[3];
    float accel_bias_lsb[3];
    bool  calibrated;
} ImuBias_t;

static ImuBias_t s_bias_primary;

/* ---------------------------------------------------------------------------
 * State task pemulihan kalibrasi (task_calib_recover(), lihat definisinya di
 * bawah). Dideklarasikan SE-AWAL ini -- bukan tepat di atas task-nya -- karena
 * task_oled() yang letaknya lebih dulu di file ini ikut membacanya untuk
 * menampilkan "DIAM NN%" di baris KALIBRASI checklist. Menaruhnya di bawah
 * berarti harus menambah forward-declaration terpisah untuk hal yang sama.
 * ------------------------------------------------------------------------- */
typedef enum {
    CALIB_RECOVER_IDLE = 0,   /* tidak ada kalibrasi ulang yang berjalan       */
    CALIB_RECOVER_RUNNING,    /* sedang mengumpulkan sample untuk SATU IMU     */
    CALIB_RECOVER_BACKOFF     /* baru saja FAILED/timeout, tunggu lalu ulangi  */
} CalibRecoverState_t;

static CalibRecoverState_t s_calib_recover_state = CALIB_RECOVER_IDLE;
static bool     s_calib_recover_on_primary;      /* IMU mana yang sedang dikerjakan */
static uint32_t s_calib_recover_deadline_ms;     /* dipakai untuk backoff & timeout  */

/* ---------------------------------------------------------------------------
 * Mag: dua driver pin-compatible tapi register-map beda total
 * (mag_hmc5883.c/h dan mag_qmc5883.c/h, lihat firmware-architecture doc
 * Bagian 2/3.20 dan pinout doc Bagian 2) — TIDAK ada modul dispatcher
 * terpisah, jadi main.c sendiri yang mengingat chip mana yang terdeteksi
 * saat boot dan memanggil driver yang sesuai di task_mag().
 * ------------------------------------------------------------------------- */
typedef enum {
    MAG_CHIP_NONE = 0,
    MAG_CHIP_HMC5883,
    MAG_CHIP_QMC5883
} MagChip_t;

static MagChip_t s_active_mag_chip = MAG_CHIP_NONE;

/* Dipakai task_mag() untuk mendeteksi transisi kalibrasi mag ke DONE
 * (sekali per transisi, bukan tiap cycle) -- lihat task_mag() di bawah. */
static CalibMagState_t s_mag_calib_state_prev = CALIB_MAG_STATE_IDLE;

static void imu_raw_to_sample(int16_t ax, int16_t ay, int16_t az,
                              int16_t gx, int16_t gy, int16_t gz,
                              const ImuBias_t *bias,
                              bool valid,
                              AHRS_ImuSample_t *out)
{
    out->accel_x = ((float)ax - bias->accel_bias_lsb[0]) / ACCEL_LSB_PER_G;
    out->accel_y = ((float)ay - bias->accel_bias_lsb[1]) / ACCEL_LSB_PER_G;
    out->accel_z = ((float)az - bias->accel_bias_lsb[2]) / ACCEL_LSB_PER_G;

    out->gyro_x = (((float)gx - bias->gyro_bias_lsb[0]) / GYRO_LSB_PER_DPS) * DEG_TO_RAD;
    out->gyro_y = (((float)gy - bias->gyro_bias_lsb[1]) / GYRO_LSB_PER_DPS) * DEG_TO_RAD;
    out->gyro_z = (((float)gz - bias->gyro_bias_lsb[2]) / GYRO_LSB_PER_DPS) * DEG_TO_RAD;

    out->data_valid = valid;
}

/* ===========================================================================
 * State global firmware
 * ===========================================================================
 * Semua di satu struct, bukan variabel global berserakan — supaya jelas apa
 * saja yang dibagi antar task, dan supaya gampang di-dump utuh ke blackbox
 * atau telemetry nanti. Karena scheduler-nya cooperative, tidak ada satu pun
 * field di sini yang butuh proteksi mutex/critical section.
 */
typedef struct {
    AHRS_FusionState_t ahrs;
    AHRS_ImuSample_t   imu_primary;

    NAV_State_t        nav;
    NAV_GpsSample_t    gps_sample;

    StabilizeContext_t stabilize;
    IBUS_State_t        rx;

    BatteryConfig_t    battery_config;
    BatteryReading_t   battery;

    bmp280_data_t      baro;
    /* x/y/z raw LSB, bentuknya sama persis di HMC5883_RawData_t dan
     * QMC5883_RawData_t (lihat kedua header) — disalin ke sini lewat
     * field generik supaya task_mag() tidak perlu tahu tipe struct
     * driver mana yang sedang aktif. */
    struct { int16_t x; int16_t y; int16_t z; } mag;
    CompassCalib_t     compass_calib;   /* hard/soft-iron, diisi lewat
                                          * Compass_SetCalibration() begitu
                                          * kalibrasi mag DONE -- lihat
                                          * task_mag() & fusion/compass.h */
    GPS_Data_t         gps_raw;

    /* Raw LSB IMU untuk CMD_IMU_RAW (Comms #4). primary diisi
     * task_imu_primary(). secondary TIDAK PERNAH diisi di snapshot kode ini
     * (tidak ada jalur baca MPU6050 di main.c) -> secondary_valid=false ->
     * dikirim nol. */
    TelemetryImuRaw_t  imu_raw;

    float              altitude_m;

    bool armed;
    bool armed_prev;

    /* Full manual / passthrough: saat aktif, task_control() melewati
     * Stabilize_Update() (jadi PID roll/pitch sama sekali tidak jalan)
     * dan membangun MixerInput_t langsung dari stick mentah — lihat
     * task_control(). Mixer TETAP dipanggil (itu wiring airframe V-tail,
     * bukan bagian stabilisasi). *_prev dipakai untuk mendeteksi transisi
     * kembali ke stabilize supaya integrator PID bisa direset. */
    bool passthrough_active;
    bool passthrough_prev;

    bool rth_switch_active; 
    bool rth_switch_prev;

    /* Status hasil init — dipakai untuk memutuskan task mana yang
     * diaktifkan, dan ditampilkan ke OLED saat boot. */

    /* KETERPASANGAN IMU: diisi SEKALI dari hasil MPU6500_Init()/
     * MPU6050_Init() di Fase 4, dan setelah itu TIDAK PERNAH diubah lagi
     * oleh siapa pun -- khususnya TIDAK oleh kegagalan kalibrasi.
     *
     * Dulu ada DUA flag dengan arti sama (has_imu_* dan imu_*_present),
     * dan itu justru bentuk bug aslinya: cabang gagal kalibrasi mengeset
     * has_imu_* = false, sehingga "sensor tidak ada" dan "sensor ada tapi
     * kalibrasi gagal" tidak bisa dibedakan lagi. Disatukan jadi satu field
     * supaya tidak ada tempat kedua yang bisa diam-diam menghapus informasi
     * ini. Yang menahan arming saat kalibrasi gagal adalah flag .calibrated
     * di s_bias_*, BUKAN field ini. */
    bool imu_primary_present;
    bool has_baro;
    bool has_mag;
    bool has_oled;
} FcState_t;

static FcState_t   s_fc;
static Scheduler_t s_sched;

static bool calib_gate_ok(void)
{
    return CalibGate_ArmAllowed(s_fc.imu_primary_present, s_bias_primary.calibrated,
                                false, false);
}

/* ===========================================================================
 * Independent watchdog
 * ===========================================================================
 * LSI ~32 kHz, prescaler /32 -> ~1 kHz. Reload 500 -> timeout ~500ms.
 * Dipilih jauh lebih longgar dari periode task paling pelan (OLED 250ms)
 * supaya tidak false-reset, tapi tetap cukup ketat: kalau loop macet di
 * busy-wait I2C, pesawat mereset dalam setengah detik alih-alih terbang
 * tanpa kontrol sampai baterai habis.
 *
 * CATATAN: IWDG TIDAK BISA DIMATIKAN setelah dinyalakan, kecuali reset.
 * Saat debugging step-by-step, aktifkan DBGMCU_APB1_FZ_DBG_IWDG_STOP atau
 * MCU akan reset tiap kali kamu berhenti di breakpoint.
 */
static void watchdog_init(void)
{
    IWDG->KR  = 0x5555U;  /* buka akses tulis ke PR/RLR */
    IWDG->PR  = 3U;       /* prescaler /32 */
    IWDG->RLR = 500U;     /* ~500ms */
    IWDG->KR  = 0xAAAAU;  /* reload */
    IWDG->KR  = 0xCCCCU;  /* start — tidak bisa di-undo */
}

static inline void watchdog_feed(void)
{
    IWDG->KR = 0xAAAAU;
}

/* ===========================================================================
 * Indikator error boot
 * ===========================================================================
 * Dipanggil kalau clock gagal dikonfigurasi. Delay-nya sengaja pakai loop
 * kosong, bukan delay_ms(), karena SysTick belum tentu jalan di titik ini.
 */
static void boot_fail_blink(system_clock_status_t reason)
{
    /* Dipanggil SEBELUM BSP_PinMap_Init(), jadi GPIOC clock & konfigurasi
     * pin PC13 belum tentu sudah aktif -- fungsi ini TIDAK BOLEH
     * bergantung pada BSP_PinMap_Init() dan harus menyalakan GPIOC +
     * PC13 sendiri secara minimal di sini.
     *
     * DEBUG SEMENTARA: sebelumnya fungsi ini cuma busy-wait kosong tanpa
     * pernah menyentuh LED sama sekali -- kalau SystemClock_Config()
     * gagal, board terlihat "mati total" (LED tidak merespons apa pun),
     * padahal firmware sebenarnya masih berjalan (stuck di sini).
     *
     * Sekarang membedakan pola sesuai titik kegagalan (parameter `reason`
     * -- nilai balik SystemClock_Config()), supaya bisa dibedakan tanpa
     * debugger:
     *   - HSE timeout   (kristal 25MHz tidak terdeteksi) -> kedip 1x lalu
     *     jeda panjang, berulang terus.
     *   - PLL/VOS/flash-latency timeout (HSE OK tapi tahap sesudahnya
     *     gagal -- seharusnya jarang terjadi) -> kedip 2x lalu jeda
     *     panjang, berulang terus.
     * Pola "N-kali + jeda, berulang tanpa henti" ini sengaja dibuat beda
     * dari debug_blink_sensor_status() (yang cuma SATU putaran lalu
     * berhenti/lanjut boot) -- supaya dari luar langsung jelas kalau
     * board macet permanen di sini, bukan lanjut ke fase berikutnya.
     */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    __asm volatile ("nop");
    __asm volatile ("nop");

    /* PC13 sebagai output push-pull, speed low, no pull -- setara isi
     * PINMAP_MODE_OUTPUT/PINMAP_OTYPE_PP/PINMAP_PUPD_NONE di bsp_pinmap.c,
     * ditulis manual di sini karena BSP_PinMap_Init() belum tentu jalan. */
    GPIOC->MODER   = (GPIOC->MODER   & ~(3u << (13u * 2u))) | (1u << (13u * 2u)); /* output */
    GPIOC->OTYPER  &= ~(1u << 13u);                                              /* push-pull */
    GPIOC->OSPEEDR &= ~(3u << (13u * 2u));                                       /* low speed */
    GPIOC->PUPDR   &= ~(3u << (13u * 2u));                                       /* no pull */

    const int blink_count = (reason == SYSTEM_CLOCK_ERR_HSE_TIMEOUT) ? 1 : 2;

    for (;;) {
        for (int i = 0; i < blink_count; i++) {
#if LED_STATUS_ACTIVE_LOW
            GPIOC->BSRR = (1u << (13u + 16u)); /* nyala */
#else
            GPIOC->BSRR = (1u << 13u);
#endif
            for (volatile uint32_t d = 0; d < 200000U; d++) { }

#if LED_STATUS_ACTIVE_LOW
            GPIOC->BSRR = (1u << 13u);         /* mati */
#else
            GPIOC->BSRR = (1u << (13u + 16u));
#endif
            for (volatile uint32_t d = 0; d < 200000U; d++) { }
        }
        /* Jeda panjang antar putaran, supaya blink_count mudah dihitung. */
        for (volatile uint32_t d = 0; d < 1200000U; d++) { }
    }
}

/* ---------------------------------------------------------------------------
 * DEBUG SEMENTARA: kedip-kedip LED status (PC13) sesuai hasil init sensor,
 * supaya bisa didiagnosis tanpa debugger/UART cuma dengan mengamati LED
 * onboard saat boot. Dipanggil SEKALI di akhir Fase 4 (setelah semua
 * *_Init() sensor selesai), sebelum lanjut ke fase berikutnya.
 *
 * Pola: untuk tiap sensor yang GAGAL init, LED kedip cepat sebanyak N kali
 * (N = urutan sensor di bawah), lalu jeda panjang, lalu lanjut ke sensor
 * berikutnya yang gagal. Kalau SEMUA sensor OK, LED nyala solid ~1 detik
 * lalu mati (tanpa pola kedip) -- gampang dibedakan dari pola kedip gagal.
 *
 * Urutan N tetap (bukan sekadar berapa kali gagal), supaya bisa langsung
 * dikenali sensor mana yang bermasalah dari hitungan kedipannya:
 *   N=1 -> IMU primary (MPU6500)   N=2 -> Baro (BMP280)
 *   N=3 -> OLED (SSD1306)          N=4 -> Magnetometer (HMC5883/QMC5883)
 *
 * HAPUS pemanggilan debug_blink_sensor_status() ini (dan fungsinya) kalau
 * sudah tidak dibutuhkan lagi -- ini murni alat bantu diagnosa boot,
 * bukan bagian dari alur normal firmware.
 * ------------------------------------------------------------------------- */
static void debug_led_set(bool on)
{
#if LED_STATUS_ACTIVE_LOW
    if (on) {
        PIN_LED_STATUS_PORT->BSRR = (1u << (PIN_LED_STATUS_PIN + 16u)); /* LOW = nyala */
    } else {
        PIN_LED_STATUS_PORT->BSRR = (1u << PIN_LED_STATUS_PIN);        /* HIGH = mati */
    }
#else
    if (on) {
        PIN_LED_STATUS_PORT->BSRR = (1u << PIN_LED_STATUS_PIN);
    } else {
        PIN_LED_STATUS_PORT->BSRR = (1u << (PIN_LED_STATUS_PIN + 16u));
    }
#endif
}

static void debug_led_delay(uint32_t loops)
{
    for (volatile uint32_t i = 0; i < loops; i++) { }
}

static void debug_blink_sensor_status(void)
{
    #define DEBUG_BLINK_UNIT        250000u  /* ~durasi 1 kedip, busy-wait */
    #define DEBUG_BLINK_GAP_UNITS   3u       /* jeda antar sensor (kelipatan UNIT) */

    const bool ok[4] = {
        s_fc.imu_primary_present,   /* N=1 */
        s_fc.has_baro,              /* N=2 */
        s_fc.has_oled,              /* N=3 */
        s_fc.has_mag,               /* N=4 */
    };

    bool any_fail = false;
    for (int i = 0; i < 4; i++) {
        if (!ok[i]) {
            any_fail = true;
        }
    }

    if (!any_fail) {
        /* Semua sensor OK: nyala solid sebentar, beda pola dari kedipan gagal. */
        debug_led_set(true);
        debug_led_delay(DEBUG_BLINK_UNIT * 4u);
        debug_led_set(false);
        return;
    }

    for (int i = 0; i < 4; i++) {
        if (ok[i]) {
            continue;
        }
        for (int blink = 0; blink < (i + 1); blink++) {
            debug_led_set(true);
            debug_led_delay(DEBUG_BLINK_UNIT);
            debug_led_set(false);
            debug_led_delay(DEBUG_BLINK_UNIT);
        }
        debug_led_delay(DEBUG_BLINK_UNIT * DEBUG_BLINK_GAP_UNITS);
    }

    #undef DEBUG_BLINK_UNIT
    #undef DEBUG_BLINK_GAP_UNITS
}

/* ===========================================================================
 * Helper konversi iBus
 * ===========================================================================
 * Rentang channel iBus: 1000 (min) .. 1500 (center) .. 2000 (max). Sebelum
 * migrasi ini rentangnya 172/992/1811 (CRSF) -- logic rumus di bawah TIDAK
 * berubah sama sekali, cuma konstanta rentangnya yang beda protokol.
 */
#define IBUS_CH_MIN     1000.0f
#define IBUS_CH_CENTER  1500.0f
#define IBUS_CH_MAX     2000.0f

/* ===========================================================================
 * Pemetaan channel switch — radio FlySky i6 (iBus)
 * ===========================================================================
 * KEPUTUSAN TIM: mengikuti penugasan AUX di menu transmitter FlySky i6
 * yang dipakai (bukan lagi channel bebas pertama yang kosong seperti
 * sebelumnya) —
 *   - SWA (2 posisi)  -> channel 5 (1-indexed) -> array index [4] -> arm switch
 *   - SWC (3 posisi)  -> channel 6 (1-indexed) -> array index [5] -> mode switch
 * Index array di bawah 0-based (index 4 = channel 5, index 5 = channel 6),
 * konsisten dengan indexing IBUS_ChannelData_t.channel[] di rx_ibus.h.
 * Kalau nanti radio/receiver diganti dan penugasan AUX-nya beda, cukup
 * ubah dua angka ini di satu tempat — jangan cari-cari literal channel[]
 * lain di task_control(). */
#define IBUS_CH_ARM_SWITCH   4U   /* SWA, channel 5 */
#define IBUS_CH_MODE_SWITCH  5U   /* SWC, channel 6 */
#define IBUS_CH_RTH_SWITCH   6U  /* SWD, channel 7 */ 

/* SWC adalah switch 3 posisi (bukan 2 seperti SWA) -- dipetakan ke tiga
 * mode terbang lewat dua threshold di bawah, bukan satu center seperti
 * arm switch:
 *   raw < IBUS_CH_MODE_LOW_THRESHOLD   -> posisi bawah  -> PASSTHROUGH
 *   raw > IBUS_CH_MODE_HIGH_THRESHOLD  -> posisi atas   -> AUTO NAVIGATION
 *   selain itu (posisi tengah)                          -> ASSISTED
 * Nilai raw iBus 3-posisi FlySky biasanya dekat 1000/1500/2000 — threshold
 * diberi margin dari titik tengah supaya tidak flip-flop di sekitar
 * transisi mekanis switch (bukan hasil pengukuran radio sungguhan, sesuaikan
 * kalau ternyata posisi fisik SWC di radio yang dipakai tidak persis di situ). */
#define IBUS_CH_MODE_LOW_THRESHOLD   1300.0f
#define IBUS_CH_MODE_HIGH_THRESHOLD  1700.0f

static float ibus_to_signed(uint16_t raw)
{
    float v = (float)raw;
    if (v >= IBUS_CH_CENTER) {
        return (v - IBUS_CH_CENTER) / (IBUS_CH_MAX - IBUS_CH_CENTER);
    }
    return (v - IBUS_CH_CENTER) / (IBUS_CH_CENTER - IBUS_CH_MIN);
}

static float ibus_to_unsigned(uint16_t raw)
{
    float v = ((float)raw - IBUS_CH_MIN) / (IBUS_CH_MAX - IBUS_CH_MIN);
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

/* ===========================================================================
 * Callback RTH untuk failsafe RX
 * ===========================================================================*/
static void on_link_lost_trigger_rth(void *nav_context)
{
    Nav_TriggerRTH((NAV_State_t *)nav_context);
}

/* ===========================================================================
 * Task
 * ===========================================================================*/

static void task_ibus_drain(void *ctx, float dt_s)
{
    (void)ctx; (void)dt_s;

    /* Kosongkan seluruh ring buffer tiap siklus. Batas 64 byte per panggilan
     * mencegah satu burst besar mengunci scheduler — sisanya diambil siklus
     * berikutnya 1ms kemudian, jauh lebih cepat dari laju frame iBus. */
    uint8_t byte;
    uint8_t budget = 64U;
    while (budget-- > 0U && BSP_UART1_ReadByte(&byte)) {
        RxIbus_FeedByte(&s_fc.rx, byte, get_tick_ms());
    }
}

static void task_protocol(void *ctx, float dt_s);

static void task_imu_primary(void *ctx, float dt_s)
{
    (void)ctx; (void)dt_s;

    mpu6500_data_t raw;
    bool ok = s_fc.imu_primary_present && MPU6500_ReadRaw(&raw);

    if (ok) {
        imu_raw_to_sample(raw.accel_x, raw.accel_y, raw.accel_z,
                          raw.gyro_x, raw.gyro_y, raw.gyro_z,
                          &s_bias_primary, true, &s_fc.imu_primary);
        /* Salinan LSB mentah untuk CMD_IMU_RAW (read-only, tidak dipakai
         * jalur kontrol). */
        s_fc.imu_raw.primary[0] = raw.accel_x;
        s_fc.imu_raw.primary[1] = raw.accel_y;
        s_fc.imu_raw.primary[2] = raw.accel_z;
        s_fc.imu_raw.primary[3] = raw.gyro_x;
        s_fc.imu_raw.primary[4] = raw.gyro_y;
        s_fc.imu_raw.primary[5] = raw.gyro_z;
        s_fc.imu_raw.primary_valid = true;
    } else {
        s_fc.imu_primary.data_valid = false;
        s_fc.imu_raw.primary_valid  = false;
    }
}

static void task_ahrs(void *ctx, float dt_s)
{
    (void)ctx; (void)dt_s;
    static const AHRS_ImuSample_t no_secondary = { 0 };

    /* task_mag() (50Hz) lebih lambat dari task_ahrs() (200Hz) -- ~75%
     * siklus ini akan memakai s_fc.mag yang sama persis dengan siklus
     * sebelumnya (baro belum sempat update). Ini BUKAN bug, pola yang
     * sama sudah dipakai sensor lain (baro) di firmware ini. */
    AHRS_MagSample_t mag_sample;
    mag_sample.valid = s_fc.has_mag && Compass_HasCalibration(&s_fc.compass_calib);
    if (mag_sample.valid) {
        float cal_x, cal_y, cal_z;
        Compass_ApplyCalibration(&s_fc.compass_calib,
                                 s_fc.mag.x, s_fc.mag.y, s_fc.mag.z,
                                 &cal_x, &cal_y, &cal_z);
        float bx, by, bz;
        Compass_RemapToBody(cal_x, cal_y, cal_z, &bx, &by, &bz);
        mag_sample.heading_deg = Compass_ComputeHeadingDeg(bx, by, bz,
            s_fc.ahrs.attitude.roll_deg, s_fc.ahrs.attitude.pitch_deg);
    } else {
        mag_sample.heading_deg = 0.0f; /* tidak dibaca saat valid=false */
    }

    AHRS_Fusion_Update(&s_fc.ahrs, &s_fc.imu_primary, &no_secondary,
                       &mag_sample, get_tick_ms());
}

static float throttle_apply_deadband(float stick)
{
    if (stick <= ARM_THROTTLE_LOW_THRESHOLD) {
        return 0.0f;
    }
    return (stick - ARM_THROTTLE_LOW_THRESHOLD) / (1.0f - ARM_THROTTLE_LOW_THRESHOLD);
}

static float rth_bank_from_heading(float target_deg, float current_deg)
{
    /* target_deg (bearing nav) searah jarum jam. current_deg (att->yaw_deg
     * AHRS) berlawanan arah jarum jam -- konversi dulu sebelum dibandingkan,
     * pola sama seperti -mag->heading_deg di ahrs_fusion.c. */
    float current_cw_deg = -current_deg;

    float error = target_deg - current_cw_deg;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    float bank = error * RTH_HEADING_TO_BANK_GAIN;
    if (bank > RTH_MAX_BANK_DEG) return RTH_MAX_BANK_DEG;
    if (bank < -RTH_MAX_BANK_DEG) return -RTH_MAX_BANK_DEG;
    return bank;
}

#ifdef BENCH_TEST_FASE_C
volatile float g_bench_yaw_deg = 0.0f;
volatile float g_bench_bank_deg = 0.0f;
/* ---------------------------------------------------------------------------
 * Harness bangku Fase C -- HANYA aktif kalau BENCH_TEST_FASE_C didefinisikan
 * saat build (mis. `-D BENCH_TEST_FASE_C`). Prasyarat WAJIB Fase A (axis
 * remap compass.c, terkonfirmasi uji putar) dan Fase B
 * (rth_bank_from_heading() CW/CCW, host test lolos) -- lihat
 * prompt-fase-c-bangku-arah-bank.md. Tidak pernah aktif di build normal;
 * HAPUS/matikan flag ini sebelum build untuk terbang (lihat Bagian 3
 * prompt tsb).
 *
 * BEDA dari kode contoh di prompt: LED di sini NON-BLOCKING (state machine
 * berbasis get_tick_ms(), ditoggle tiap panggilan), bukan busy-wait
 * debug_led_delay(). debug_led_delay() dipakai di tempat lain HANYA di
 * boot_fail_blink()/debug_blink_sensor_status(), keduanya berjalan SEBELUM
 * scheduler+IWDG hidup (lihat komentar di kedua fungsi itu) -- memanggilnya
 * dari dalam task_control() (scheduler SUDAH jalan, IWDG timeout ~500 ms
 * hanya di-feed di akhir satu siklus scheduler PENUH, lihat watchdog_feed()
 * di main()) akan memblokir task lain & bisa memicu reset berulang persis
 * di tengah tes bangku. Konvensi LED yang diamati (solid/cepat/lambat)
 * tetap sama seperti dijelaskan di prompt, hanya mekanismenya yang beda. */
#ifndef BENCH_TEST_TARGET_BEARING_DEG
#define BENCH_TEST_TARGET_BEARING_DEG 90.0f  /* ganti & reflash untuk tiap bearing uji */
#endif
#define BENCH_TEST_LED_HALF_PERIOD_FAST_MS 100u  /* bank>0 (perlu kanan): ~5 Hz */
#define BENCH_TEST_LED_HALF_PERIOD_SLOW_MS 500u  /* bank<0 (perlu kiri):  ~1 Hz */

static void bench_test_rth_bank_led(void)
{
    const AHRS_Attitude_t *att = AHRS_Fusion_GetAttitude(&s_fc.ahrs);
    float bank = rth_bank_from_heading(BENCH_TEST_TARGET_BEARING_DEG, att->yaw_deg);

    g_bench_yaw_deg  = att->yaw_deg;
    g_bench_bank_deg = bank;

    /* Konvensi LED (PC13, sesuai prosedur uji di prompt):
     *  - |bank| < 2 deg -> nyala SOLID terus (di ambang/on-target)
     *  - bank > 0 (harus belok kanan) -> kedip CEPAT (~5 Hz)
     *  - bank < 0 (harus belok kiri)  -> kedip LAMBAT (~1 Hz)
     * Amati sambil board diputar manual -- lihat prosedur di prompt. */
    static bool     s_led_on;
    static uint32_t s_last_toggle_ms;

    if (bank > -2.0f && bank < 2.0f) {
        debug_led_set(true);
        s_led_on = true;
        s_last_toggle_ms = get_tick_ms();  /* reset -- kedip berikutnya mulai bersih begitu keluar zona on-target */
        return;
    }

    uint32_t half_period_ms = (bank > 0.0f) ? BENCH_TEST_LED_HALF_PERIOD_FAST_MS
                                             : BENCH_TEST_LED_HALF_PERIOD_SLOW_MS;
    uint32_t now = get_tick_ms();
    /* Selisih uint32_t wrap-safe (sama pola dengan pemakaian get_tick_ms()
     * lain di file ini, lihat komentar di ~baris 1316). */
    if ((now - s_last_toggle_ms) >= half_period_ms) {
        s_led_on = !s_led_on;
        debug_led_set(s_led_on);
        s_last_toggle_ms = now;
    }
}
#endif /* BENCH_TEST_FASE_C */

static void task_control(void *ctx, float dt_s)
{
    (void)ctx;

    const IBUS_ChannelData_t *ch = RxIbus_GetChannels(&s_fc.rx);
    bool link_ok = (RxIbus_GetLinkStatus(&s_fc.rx) == IBUS_LINK_OK) && ch->data_valid;
    bool attitude_ok = AHRS_Fusion_IsAttitudeValid(&s_fc.ahrs);

    /* --- Evaluasi arming ---
     * Migrasi ke armed_state.c (lihat TODO INTEGRASI di armed_state.h):
     * task_control() sekarang hanya menyusun snapshot ArmedStateInputs_t
     * dari sinyal cycle ini, lalu menyerahkan evaluasi arm/disarm +
     * preflight gate ke ArmedState_Update()/ArmedState_IsArmed() --
     * bukan menghitung want_armed sendiri secara inline seperti
     * sebelumnya. Efek yang SENGAJA berubah: link-loss saat sudah ARMED
     * tidak lagi memaksa disarm seketika dari sini (lihat "Perbedaan
     * sengaja" di armed_state.h) -- itu keputusan desain armed_state.c,
     * bukan bug migrasi ini. */
    float throttle_stick = link_ok ? ibus_to_unsigned(ch->channel[2]) : 0.0f;

    ArmedStateInputs_t armed_inputs;
    armed_inputs.arm_switch_engaged = (ch->channel[IBUS_CH_ARM_SWITCH] > (uint16_t)IBUS_CH_CENTER);
                                       /* level dari link_ok TIDAK di-AND
                                        * manual di sini -- rx_link_ok diisi
                                        * terpisah di bawah, armed_state.c
                                        * yang menggabungkan logikanya sendiri
                                        * lewat run_preflight(). */
    armed_inputs.throttle_low = (throttle_stick < ARM_THROTTLE_LOW_THRESHOLD);
    armed_inputs.rx_link_ok       = link_ok;
    armed_inputs.attitude_valid   = attitude_ok;
    /* KEPUTUSAN TIM (bukan bug): sejak task_imu_secondary() jadi
     * failover-murni (MPU6050 idle selama primer sehat -- lihat
     * komentar di task_imu_secondary()), cabang have_primary &&
     * have_secondary di AHRS_Fusion_Update() praktis tidak pernah
     * jalan selama primer hidup, jadi disagreement_flag EFEKTIF SELALU
     * false di kondisi itu -- termasuk tepat saat preflight arm
     * dievaluasi. PREFLIGHT_IMU_DISAGREE (armed_state.h) jadi gate
     * yang nonaktif secara efektif selama primer sehat: MPU6500 yang
     * hidup tapi diam-diam salah baca (bukan timeout) TIDAK akan
     * tertangkap lewat jalur ini lagi. Sengaja dibiarkan begini demi
     * mengurangi beban I2C1 -- lihat armed_state.h untuk catatan yang
     * sama di sisi gate-nya. */
    armed_inputs.imu_disagreement = s_fc.ahrs.attitude.disagreement_flag;

    /* KEPUTUSAN calibration_done (Opsi A, lebih ketat -- lihat catatan
     * ArmedStateInputs_t di armed_state.h untuk dua opsi yang dibolehkan):
     * dipetakan ke hasil kalibrasi SUNGGUHAN per IMU (flag .calibrated di
     * s_bias_primary/secondary), BUKAN hardcode true.
     *
     * KENAPA lewat CalibGate_ArmAllowed() dan bukan `primary.calibrated &&
     * secondary.calibrated` seperti versi sebelumnya: ekspresi AND itu tidak
     * pernah memeriksa KETERPASANGAN sensor (komentar lamanya mengklaim
     * "untuk IMU yang terpasang", padahal kodenya tidak memeriksa apa pun).
     * Akibatnya, kalau MPU6050 tidak dipasang atau Init()-nya gagal,
     * s_bias_secondary.calibrated tetap false SELAMANYA dan arming ditolak
     * permanen sampai power-cycle -- tanpa indikasi apa pun ke pengguna.
     * Kebijakan gerbangnya sekarang tinggal SATU tempat, calib_gate.c:
     * primer wajib hadir & terkalibrasi, sekunder wajib terkalibrasi hanya
     * jika hadir. Dihitung ulang tiap cycle, jadi begitu task_calib_recover()
     * berhasil memulihkan kalibrasi, gerbang ini ikut terbuka tanpa reboot. */
    armed_inputs.calibration_done = calib_gate_ok();

    /* Blackbox belum diaktifkan di scheduler ini (lihat catatan status
     * item di atas main()) -- hardcode true di sini supaya tidak
     * memblokir arming karena fitur yang memang belum jalan. HARUS
     * diganti begitu blackbox betulan aktif (lacak return code terakhir
     * Blackbox_Init()/Blackbox_WriteRecord()), jangan dibiarkan diam-diam
     * seperti ini selamanya. */
    armed_inputs.blackbox_ready = true;

    ArmedState_Update(&armed_inputs);
    s_fc.armed = ArmedState_IsArmed();

    if (s_fc.armed != s_fc.armed_prev) {
        Stabilize_OnArmedTransition(&s_fc.stabilize, s_fc.armed);
        s_fc.armed_prev = s_fc.armed;
    }

    /* --- Mode switch (SWC, channel 6) ---
     * 3 posisi -> 3 mode terbang:
     *   bawah  = PASSTHROUGH      (full manual, stabilize & nav di-skip total)
     *   tengah = ASSISTED         (stabilize PID aktif, stick = setpoint sudut)
     *   atas   = AUTO NAVIGATION  (nav mode di-engage — lihat di bawah)
     *
     * Sengaja di-AND dengan link_ok, pola sama seperti arm_switch:
     * kalau link iBus putus, nilai channel[] terakhir yang tersimpan di
     * parser bisa saja basi (bukan sinyal switch pilot yang sesungguhnya).
     * Kalau link putus:
     *   - passthrough dipaksa nonaktif (fallback ke stabilize) — sama
     *     seperti sebelumnya, full-manual tanpa link hidup tidak berarti
     *     apa-apa.
     *   - TAPI nav_mode SENGAJA TIDAK disentuh sama sekali di sini (lihat
     *     blok "else" di bawah) — RTH failsafe yang mungkin baru saja
     *     ter-trigger dari task_failsafe()/on_link_lost_trigger_rth() TIDAK
     *     boleh diam-diam dibatalkan cuma karena posisi switch mode yang
     *     basi kebetulan bukan "auto navigation". Failsafe RTH murni
     *     tanggung jawab jalur rx_ibus.c, bukan blok ini. */
    // if (link_ok) {
    //     uint16_t mode_ch_raw   = ch->channel[IBUS_CH_MODE_SWITCH];
    //     bool     want_passthrough = (mode_ch_raw < (uint16_t)IBUS_CH_MODE_LOW_THRESHOLD);
    //     bool     want_auto_nav    = (mode_ch_raw > (uint16_t)IBUS_CH_MODE_HIGH_THRESHOLD);
    //     /* posisi tengah: !want_passthrough && !want_auto_nav -> assisted,
    //      * tidak butuh variabel terpisah, cukup jadi kondisi "else" di bawah. */

    //     s_fc.passthrough_active = want_passthrough;

    //     /* Auto-nav: switch pilot SELALU menang atas mode navigasi otomatis.
    //      * - Posisi atas & belum ada nav mode aktif -> engage altitude-hold
    //      *   (satu-satunya mode auto yang bisa di-engage langsung dari
    //      *   switch tanpa mission/home ter-upload dulu; RTH/waypoint tetap
    //      *   hanya lewat failsafe link-loss atau mission seperti sebelumnya
    //      *   — tidak diambil alih paksa kalau kebetulan sudah aktif).
    //      * - Selain posisi atas (assisted ATAU passthrough) & nav mode lagi
    //      *   aktif -> Nav_ExitToIdle(), termasuk membatalkan RTH/waypoint
    //      *   yang sedang berjalan. Kedua cabang self-guarding lewat cek
    //      *   mode saat ini (bukan flag *_prev terpisah): begitu mode sudah
    //      *   sesuai target, cabangnya tidak terpanggil ulang cycle
    //      *   berikutnya. */
    //     NAV_Mode_t nav_mode_now = Nav_GetMode(&s_fc.nav);
    //     if (want_auto_nav) {
    //         if (nav_mode_now == NAV_MODE_IDLE) {
    //             if (!Nav_StartWaypoint(&s_fc.nav)) {
    //                 Nav_EnterAltitudeHold(&s_fc.nav, s_fc.altitude_m);
    //             }
    //         }
    //     } else if (nav_mode_now != NAV_MODE_IDLE) {
    //         Nav_ExitToIdle(&s_fc.nav);
    //     }
    // } else {
    //     s_fc.passthrough_active = false;
    // }

    if (link_ok) {
        bool rth_sw     = (ch->channel[IBUS_CH_RTH_SWITCH] > (uint16_t)IBUS_CH_CENTER);
        bool rth_rising = rth_sw && !s_fc.rth_switch_prev;
        s_fc.rth_switch_prev = rth_sw;

        uint16_t mode_ch_raw = ch->channel[IBUS_CH_MODE_SWITCH];
        bool want_passthrough = (mode_ch_raw < (uint16_t)IBUS_CH_MODE_LOW_THRESHOLD) && !rth_sw;
        bool want_auto_nav    = (mode_ch_raw > (uint16_t)IBUS_CH_MODE_HIGH_THRESHOLD);

        s_fc.passthrough_active = want_passthrough;

        NAV_Mode_t nav_mode_now = Nav_GetMode(&s_fc.nav);
        bool in_rth = (nav_mode_now == NAV_MODE_RTH ||
                    nav_mode_now == NAV_MODE_RTH_NO_GPS_FALLBACK);

        if (rth_sw) {
            if (rth_rising) {
                Nav_TriggerRTH(&s_fc.nav);
                if (!in_rth) {
                    s_fc.rth_switch_active = true;
                }
            }
        } else {
            if (s_fc.rth_switch_active) {
                s_fc.rth_switch_active = false;
                if (in_rth) {
                    Nav_ExitToIdle(&s_fc.nav);
                    nav_mode_now = NAV_MODE_IDLE;
                }
            }
            if (want_auto_nav) {
                if (nav_mode_now == NAV_MODE_IDLE) {
                    if (!Nav_StartWaypoint(&s_fc.nav)) {
                        Nav_EnterAltitudeHold(&s_fc.nav, s_fc.altitude_m);
                    }
                }
            } else if (nav_mode_now != NAV_MODE_IDLE) {
                Nav_ExitToIdle(&s_fc.nav);
            }
        }
    } else {
        s_fc.passthrough_active = false;
    }

    if (s_fc.passthrough_active != s_fc.passthrough_prev) {
        if (!s_fc.passthrough_active) {
            /* Transisi KEMBALI ke stabilize (passthrough -> stabilize):
             * buang integral yang mungkin sudah menumpuk/basi selama PID
             * tidak jalan, supaya tidak ada lonjakan output mendadak begitu
             * PID mulai lagi mengoreksi error. Tidak ada reset simetris
             * untuk arah sebaliknya (stabilize -> passthrough) karena PID
             * memang tidak dipanggil sama sekali selama passthrough aktif —
             * tidak ada state yang perlu "dibersihkan" sebelum masuk. */
            Stabilize_ResetIntegrators(&s_fc.stabilize);
        }
        s_fc.passthrough_prev = s_fc.passthrough_active;
    }

    if (s_fc.passthrough_active) {
        /* Full manual murni: stick mentah -> Mixer_Compute() -> output,
         * PID (Stabilize_Update()) DILEWATI SELURUHNYA. nav_mode/
         * Nav_GetSetpoint() juga diabaikan total di sini — auto-nav
         * (altitude hold/waypoint/RTH) secara desain butuh stabilize aktif,
         * jadi tidak masuk akal dicampur dengan passthrough (lihat
         * dokumentasi task ini).
         *
         * roll/pitch pakai ibus_to_signed() TANPA dikonversi ke derajat/
         * dikalikan limit sudut (beda dengan jalur stabilize di bawah yang
         * mengalikan 35.0f) — di sini nilainya langsung command mixer
         * [-1,1], bukan setpoint sudut untuk PID. yaw & throttle sama
         * seperti jalur stabilize (sudah tidak lewat PID di sana juga).
         *
         * Mixer_Compute()/OutputMap_WriteFromMixer() dipanggil langsung di
         * sini (bukan lewat Stabilize_Update()) memakai mixer_config dan
         * output_map milik s_fc.stabilize — field-field itu memang
         * diekspos publik justru untuk kebutuhan seperti ini (lihat
         * dokumentasi StabilizeContext_t di stabilize.h). Mixer TETAP
         * jalan (mixing V-tail adalah wiring airframe, bukan bagian
         * stabilisasi yang di-skip di sini). */
        MixerInput_t manual_input;
        manual_input.roll     = link_ok ? ibus_to_signed(ch->channel[0]) : 0.0f;
        manual_input.pitch    = link_ok ? ibus_to_signed(ch->channel[1]) : 0.0f;
        manual_input.yaw      = link_ok ? ibus_to_signed(ch->channel[3]) : 0.0f;

        manual_input.throttle = throttle_apply_deadband(throttle_stick);

        MixerOutput_t manual_output =
            Mixer_Compute(&s_fc.stabilize.mixer_config, &manual_input);
        OutputMap_WriteFromMixer(&s_fc.stabilize.output_map, &manual_output, s_fc.armed);
        return; /* jangan lanjut ke jalur stabilize di bawah */
    }

    /* --- Setpoint: dari nav kalau mode auto aktif, dari stick kalau manual --- */
    const AHRS_Attitude_t *att = AHRS_Fusion_GetAttitude(&s_fc.ahrs);
    const NAV_Setpoint_t  *nav_sp = Nav_GetSetpoint(&s_fc.nav);
    NAV_Mode_t nav_mode = Nav_GetMode(&s_fc.nav);

    StabilizeInput_t in;

    if (nav_mode == NAV_MODE_IDLE) {
        /* Manual: stick jadi target sudut. Batas +-35 derajat adalah angle
         * limit sementara — angka ini keputusan tim, belum ada di dokumen
         * manapun, dan harus diturunkan untuk penerbangan pertama. */
        in.roll_setpoint_deg  = link_ok ? ibus_to_signed(ch->channel[0]) * 35.0f : 0.0f;
        in.pitch_setpoint_deg = link_ok ? ibus_to_signed(ch->channel[1]) * 35.0f : 0.0f;
        in.yaw_command        = link_ok ? ibus_to_signed(ch->channel[3]) : 0.0f;
        in.throttle_command   = throttle_apply_deadband(throttle_stick);
    } else {
        in.roll_setpoint_deg  = nav_sp->target_roll_deg;
        in.pitch_setpoint_deg = nav_sp->target_pitch_deg;
        in.yaw_command        = 0.0f;
        if (nav_mode == NAV_MODE_RTH) {
            /* Heading-seeking bank dimatikan sementara untuk RTH: att->yaw_deg
             * belum punya referensi absolut yang terverifikasi bangku (axis
             * remap di compass.c masih placeholder identity), dan
             * rth_bank_from_heading() sendiri masih membandingkan konvensi CW
             * (bearing nav) vs CCW (yaw_deg AHRS) tanpa konversi. Sampai
             * axis remap compass.c dan konversi CW/CCW di
             * rth_bank_from_heading() selesai serta lolos uji putar fisik,
             * NAV_MODE_RTH wings-level saja (in.roll_setpoint_deg jatuh ke
             * nav_sp->target_roll_deg = 0.0f di atas), sama seperti
             * NAV_MODE_RTH_NO_GPS_FALLBACK. */
            in.throttle_command = RTH_FAILSAFE_THROTTLE;
        } else if (nav_mode == NAV_MODE_WAYPOINT) {
            in.roll_setpoint_deg = rth_bank_from_heading(
                nav_sp->target_yaw_deg, att->yaw_deg);
            in.throttle_command = throttle_apply_deadband(throttle_stick);
        } else if (nav_mode == NAV_MODE_RTH_NO_GPS_FALLBACK ||
                   (nav_mode == NAV_MODE_ALTITUDE_HOLD &&
                    Nav_IsFailsafeActive(&s_fc.nav))) {
            in.throttle_command = RTH_NO_GPS_THROTTLE;
        } else {
            in.throttle_command = throttle_apply_deadband(throttle_stick);
        }
    }

    in.roll_measured_deg  = att->roll_deg;
    in.pitch_measured_deg = att->pitch_deg;
    in.armed              = s_fc.armed;
    in.dt_seconds         = dt_s;

    Stabilize_Update(&s_fc.stabilize, &in);

#ifdef BENCH_TEST_FASE_C
    /* Harness bangku Fase C -- lihat definisi bench_test_rth_bank_led() di
     * atas untuk syarat & penjelasan lengkap. Tidak memengaruhi in./
     * Stabilize_Update() di atas sama sekali -- murni observasi lewat LED. */
    bench_test_rth_bank_led();
#endif
}

static void task_dshot_send(void *ctx, float dt_s)
{
    (void)ctx; (void)dt_s;
    DShot_SendFrames();
}

static void task_failsafe(void *ctx, float dt_s)
{
    (void)ctx; (void)dt_s;
    RxIbus_Update(&s_fc.rx, get_tick_ms(), on_link_lost_trigger_rth, &s_fc.nav);
}

static void task_baro(void *ctx, float dt_s)
{
    (void)ctx; (void)dt_s;

    if (s_fc.has_baro && BMP280_Read(&s_fc.baro)) {
        s_fc.altitude_m = s_fc.baro.altitude_m;
    }
}

static void task_mag(void *ctx, float dt_s)
{
    (void)ctx; (void)dt_s;

    if (!s_fc.has_mag) {
        return;
    }

    bool read_ok = false;
    if (s_active_mag_chip == MAG_CHIP_HMC5883) {
        HMC5883_RawData_t raw;
        if (MAG_HMC5883_ReadRaw(&raw) == HMC5883_OK) {
            s_fc.mag.x = raw.x;
            s_fc.mag.y = raw.y;
            s_fc.mag.z = raw.z;
            read_ok = true;
        }
    } else if (s_active_mag_chip == MAG_CHIP_QMC5883) {
        QMC5883_RawData_t raw;
        if (MAG_QMC5883_ReadRaw(&raw) == QMC5883_OK) {
            s_fc.mag.x = raw.x;
            s_fc.mag.y = raw.y;
            s_fc.mag.z = raw.z;
            read_ok = true;
        }
    }

    if (read_ok) {
        /* No-op kalau kalibrasi mag sedang tidak IN_PROGRESS -- lihat
         * kontrak CalibDispatcher_FeedMagSample()/CalibMag_FeedSample(),
         * aman dipanggil tiap cycle tanpa cek state di sini. */
        CalibDispatcher_FeedMagSample(s_fc.mag.x, s_fc.mag.y, s_fc.mag.z);
    }

    /* Deteksi transisi ke DONE sekali per transisi (bukan tiap cycle) --
     * begitu figure-8 selesai, salin hasilnya ke s_fc.compass_calib supaya
     * task_ahrs() langsung bisa memakainya tanpa reboot. */
    CalibMagState_t mag_state = CalibMag_GetState();
    if (mag_state == CALIB_MAG_STATE_DONE && s_mag_calib_state_prev != CALIB_MAG_STATE_DONE) {
        MagCalibResult_t result;
        if (CalibMag_GetResult(&result)) {
            Compass_SetCalibration(&s_fc.compass_calib, &result);
        }
    }
    s_mag_calib_state_prev = mag_state;
}

static void gps_tx(const uint8_t *d, uint16_t n)
{
    BSP_UART2_WriteBuf(d, n);
}

static void task_gps(void *ctx, float dt_s)
{
    (void)ctx; (void)dt_s;

    uint32_t now = get_tick_ms();
    uint8_t byte;
    uint8_t budget = 128U;

    while (budget-- > 0U && BSP_UART2_ReadByte(&byte)) {
        if (GPS_UBX_ProcessByte(byte)) {
            s_gps_last_frame_ms = now;
            s_gps_seen = true;
        }
    }

    /* Modul telat boot / config hilang: kirim ulang, satu message per tick */
    if (!GPS_UBX_ConfigDone()) {
        (void)GPS_UBX_ConfigStep(gps_tx);
    } else if ((now - s_gps_last_frame_ms) > GPS_CFG_RETRY_MS) {
        GPS_UBX_ConfigRestart();
        s_gps_last_frame_ms = now;
    }

    bool fresh = s_gps_seen && ((now - s_gps_last_frame_ms) <= GPS_STALE_MS);

    if (GPS_UBX_GetData(&s_fc.gps_raw)) {
        s_fc.gps_sample.lat_e7           = s_fc.gps_raw.lat_e7;
        s_fc.gps_sample.lon_e7           = s_fc.gps_raw.lon_e7;
        s_fc.gps_sample.ground_speed_mps = (float)s_fc.gps_raw.ground_speed_mm_s * 0.001f;
        s_fc.gps_sample.sat_count        = s_fc.gps_raw.num_sv;

        if (fresh && s_fc.gps_raw.fix_ok) {
            switch (s_fc.gps_raw.fix_type) {
                case GPS_FIX_2D:      s_fc.gps_sample.fix_type = NAV_GPS_FIX_2D; break;
                case GPS_FIX_3D:
                case GPS_FIX_GNSS_DR: s_fc.gps_sample.fix_type = NAV_GPS_FIX_3D; break;
                default:              s_fc.gps_sample.fix_type = NAV_GPS_FIX_NONE; break;
            }
        } else {
            s_fc.gps_sample.fix_type = NAV_GPS_FIX_NONE;
        }
    }
    s_fc.gps_sample.data_valid = fresh && s_fc.gps_raw.fix_ok;
}

static void task_nav(void *ctx, float dt_s)
{
    (void)ctx; (void)dt_s;
    Nav_Update(&s_fc.nav, &s_fc.gps_sample, s_fc.altitude_m, s_fc.armed);
}

static void task_battery(void *ctx, float dt_s)
{
    (void)ctx;
    Battery_Update(&s_fc.battery_config, &s_fc.battery,
                   BSP_ADC_GetVBatRaw(), BSP_ADC_GetCurrentRaw(), dt_s);
}

/**
 * Susun daftar syarat arming yang SUDAH terpenuhi, untuk checklist di
 * layar DISARMED. Sumbernya sama dengan ArmedStateInputs_t di
 * task_control() (link RX, attitude valid, gerbang kalibrasi, throttle
 * stick) -- dihitung ulang di sini karena run_preflight() di
 * armed_state.c hanya melaporkan kegagalan PERTAMA, sedangkan layar
 * butuh SEMUA syarat sekaligus. armed_state.c TIDAK diubah.
 *
 * Konsekuensi yang perlu diketahui: ini duplikasi kondisi, bukan
 * pembacaan langsung dari armed_state. Kalau syarat preflight baru
 * ditambahkan ke run_preflight() (mis. GPS fix), checklist ini TIDAK
 * otomatis ikut -- tambahkan bit OLED_PRE_* dan baris di sini juga.
 */
static uint8_t oled_prearm_mask(void)
{
    const IBUS_ChannelData_t *ch = RxIbus_GetChannels(&s_fc.rx);
    bool link_ok = (RxIbus_GetLinkStatus(&s_fc.rx) == IBUS_LINK_OK) && ch->data_valid;
    float throttle_stick = link_ok ? ibus_to_unsigned(ch->channel[2]) : 0.0f;

    uint8_t m = 0;
    if (link_ok)                                              m |= OLED_PRE_RX;
    if (AHRS_Fusion_IsAttitudeValid(&s_fc.ahrs))              m |= OLED_PRE_IMU;
    /* SATU sumber kebenaran dengan armed_inputs.calibration_done di
     * task_control(): keduanya memanggil calib_gate_ok() yang sama, supaya
     * checklist tidak bisa lagi bergeser dari gerbang aslinya. */
    if (calib_gate_ok())                                      m |= OLED_PRE_CALIB;
    /* Sama seperti armed_inputs.throttle_low di task_control(): tanpa
     * link, throttle_stick dipaksa 0 -> "rendah" -> centang. Itu
     * memang perilaku gate aslinya (rx_link_ok yang menahan), jadi
     * layar jujur mengikuti: RX BELUM, THROTTLE OK. */
    /* Stick MENTAH, bukan hasil throttle_apply_deadband(): checklist harus
     * menampilkan kondisi yang sama persis dengan yang dipakai gerbang
     * arming, dan gerbang itu memang membaca stick mentah. */
    if (throttle_stick < ARM_THROTTLE_LOW_THRESHOLD)          m |= OLED_PRE_THR;
    return m;
}

static OLED_FlightMode_t oled_flight_mode(void)
{
    /* Prioritas sama dengan task_control(): nav mode aktif (auto) menang
     * atas switch pilot; kalau IDLE, baru passthrough vs assisted. */
    switch (Nav_GetMode(&s_fc.nav)) {
        case NAV_MODE_ALTITUDE_HOLD:        return OLED_MODE_ALT_HOLD;
        case NAV_MODE_WAYPOINT:             return OLED_MODE_WAYPOINT;
        case NAV_MODE_RTH:                  return OLED_MODE_RTH;
        case NAV_MODE_RTH_NO_GPS_FALLBACK:  return OLED_MODE_RTH_NO_GPS;
        case NAV_MODE_IDLE:
        default:
            return s_fc.passthrough_active ? OLED_MODE_PASSTHROUGH
                                           : OLED_MODE_ASSISTED;
    }
}

/* Dipanggil 48 Hz (RATE_OLED_HZ): tiap panggilan mengirim SETENGAH page
 * (~1.6 ms), 16 panggilan = 1 refresh layar penuh (~3 Hz). Lihat alasan
 * di oled_ssd1306.h -- jangan diubah jadi kirim seluruh layar per tick. */
static void task_oled(void *ctx, float dt_s)
{
    (void)ctx; (void)dt_s;

    if (!s_fc.has_oled) {
        return;
    }

    OLED_View_t v;
    v.armed           = s_fc.armed;
    v.prearm_ok_mask  = oled_prearm_mask();
    v.mode            = oled_flight_mode();
    v.roll_deg        = s_fc.ahrs.attitude.roll_deg;
    v.pitch_deg       = s_fc.ahrs.attitude.pitch_deg;
    v.yaw_deg         = s_fc.ahrs.attitude.yaw_deg;
    v.altitude_m      = s_fc.altitude_m;
    v.battery_v       = s_fc.battery.voltage_volts;
    v.current_a       = s_fc.battery.current_amps;
    v.ground_speed_ms = (float)s_fc.gps_raw.ground_speed_mm_s * 0.001f;
    v.gps_fix         = (OLED_GpsFix_t)s_fc.gps_raw.fix_type;
    v.gps_sat_count   = s_fc.gps_raw.num_sv;
    /* C5: sumber "DIAM NN%" di baris KALIBRASI. calib_running sengaja
     * HANYA true saat state RUNNING (bukan BACKOFF), supaya persen yang
     * ditampilkan selalu persen yang sedang benar-benar dikumpulkan. Saat
     * BACKOFF (papan tergerak, menunggu ~1 detik sebelum mengulang), baris
     * itu kembali ke "BELUM" -- itu memang keadaan sebenarnya. */
    v.calib_running     = (s_calib_recover_state == CALIB_RECOVER_RUNNING);
    v.calib_progress_pct = v.calib_running ? CalibAccelGyro_GetProgressPercent() : 0u;

    OLED_Update(get_tick_ms(), &v);
}

/* ===========================================================================
 * Kalibrasi accel/gyro saat boot (blocking) — dipanggil dari fase 5 main()
 * ===========================================================================
 * calib_accel_gyro.c didesain ASYNC: caller Start(), lalu FeedSample()
 * berkali-kali tiap ada sample baru (biasanya dari task scheduler), poll
 * state sampai DONE/FAILED (lihat calib_accel_gyro.h). Scheduler BELUM
 * jalan di fase 5 boot, jadi run_boot_calibration() di bawah membaca
 * sensor & mem-feed sample-nya sendiri dalam loop blocking, dengan
 * TIMEOUT sebagai jaring pengaman terakhir supaya boot tidak gantung
 * selamanya kalau sensor mati/selalu gagal baca.
 *
 * calib_accel_gyro.c juga SINGLETON (satu state global) — header-nya
 * mencatat itu didesain untuk SATU IMU (primer) di v1, dan kalibrasi dua
 * IMU sekaligus butuh refactor jadi non-singleton (di luar cakupan
 * perubahan ini). TAPI API publiknya (Start/FeedSample/GetResult) generik
 * terhadap sumber sample-nya — tidak ada yang hardcode ke MPU6500 secara
 * spesifik — jadi aman dipakai BERURUTAN: kalibrasi primer sampai selesai
 * & hasilnya disalin keluar, baru Start() ulang dari nol untuk sekunder.
 * Ini bukan dua instance paralel (yang memang butuh refactor), cuma
 * modul yang sama dipanggil dua kali berurutan untuk dua sensor fisik
 * berbeda — main() di bawah memanggilnya dua kali lewat read_fn berbeda.
 */
#define CALIB_BOOT_TIMEOUT_MS   8000U  /* jauh di atas TARGET_SAMPLES=500
                                         * @asumsi ~100Hz (~5 detik) di
                                         * calib_accel_gyro.c -- di sini
                                         * sample dibaca secepat loop
                                         * blocking ini jalan (lebih cepat
                                         * dari 100Hz), jadi normalnya
                                         * jauh lebih cepat dari timeout
                                         * ini; timeout hanya jaring
                                         * pengaman kalau sensor tidak
                                         * pernah kirim sample valid. */

typedef bool (*RawImuReadFn_t)(int16_t *ax, int16_t *ay, int16_t *az,
                                int16_t *gx, int16_t *gy, int16_t *gz);

static bool read_raw_imu_primary(int16_t *ax, int16_t *ay, int16_t *az,
                                  int16_t *gx, int16_t *gy, int16_t *gz)
{
    mpu6500_data_t raw;
    if (!s_fc.imu_primary_present || !MPU6500_ReadRaw(&raw)) {
        return false;
    }
    *ax = raw.accel_x; *ay = raw.accel_y; *az = raw.accel_z;
    *gx = raw.gyro_x;  *gy = raw.gyro_y;  *gz = raw.gyro_z;
    return true;
}

/**
 * Salin hasil CalibAccelGyro_GetResult() ke ImuBias_t.
 *
 * Sebelumnya blok penyalinan ini DIDUPLIKASI di main() untuk primer dan
 * sekunder. Duplikasi itu berbahaya bukan karena panjang, tapi karena
 * perlakuan KHUSUS sumbu Z di bawah gampang ikut tersalin salah / terlupa
 * di salah satu dari dua salinan. Sejak task_calib_recover() (C4) jadi
 * pemakai KETIGA, penyalinannya dipusatkan di sini.
 *
 * Sumbu Z BUKAN bias murni -- lihat kontrak accel_bias_lsb[2] di ImuBias_t
 * di atas. src->accel_bias_z masih berisi ~+1g mentah, jadi dikurangi
 * ACCEL_LSB_PER_G supaya imu_raw_to_sample() MENYISAKAN referensi gravitasi
 * yang dibutuhkan AccelToRollPitch() di ahrs_fusion.c, bukan membuangnya.
 * ASUMSI mount: Z menghadap ke atas saat pesawat diam mendatar (terbaca
 * +1g) -- kalau IMU board ini ternyata terpasang terbalik, tanda di baris
 * itu harus dibalik jadi "+ ACCEL_LSB_PER_G"; VERIFIKASI di bench (baca
 * CMD_ATTITUDE saat pesawat didiamkan rata) sebelum uji terbang.
 */
static void apply_calib_result(ImuBias_t *dst, const AccelGyroBias_t *src)
{
    dst->gyro_bias_lsb[0]  = (float)src->gyro_bias_x;
    dst->gyro_bias_lsb[1]  = (float)src->gyro_bias_y;
    dst->gyro_bias_lsb[2]  = (float)src->gyro_bias_z;
    dst->accel_bias_lsb[0] = (float)src->accel_bias_x;
    dst->accel_bias_lsb[1] = (float)src->accel_bias_y;
    dst->accel_bias_lsb[2] = (float)src->accel_bias_z - ACCEL_LSB_PER_G;
    dst->calibrated        = true;
}

/* Hasil satu percobaan kalibrasi boot. Sebelumnya fungsi ini cuma
 * mengembalikan bool, sehingga "papan tergerak" (bisa ditolong dengan
 * mencoba lagi) dan "sensor tidak pernah kirim sample" (tidak bisa)
 * tercampur jadi satu nilai false -- caller tidak punya dasar untuk
 * memutuskan retry. */
typedef enum {
    BOOT_CALIB_OK = 0,   /* DONE, *out terisi                                  */
    BOOT_CALIB_MOTION,   /* CALIB_AG_STATE_FAILED: variance gyro di atas ambang */
    BOOT_CALIB_TIMEOUT   /* melewati CALIB_BOOT_TIMEOUT_MS                      */
} BootCalibResult_t;

/* Jumlah percobaan (percobaan pertama + retry) per IMU saat boot, dan jeda
 * antar percobaan. 3 dipilih sebagai kompromi: satu retry sering tidak cukup
 * kalau pengguna baru saja meletakkan papan dan masih ada getaran sisa,
 * sementara lebih dari 3 hanya menambah waktu boot untuk kasus yang jelas
 * tidak akan membaik (papan dipegang terus) -- kasus itu sekarang ditangani
 * task_calib_recover() yang tidak memblokir boot sama sekali. Jeda 300 ms
 * memberi waktu getaran mereda; delay_ms() aman di sini karena SysTick sudah
 * aktif sejak Fase 1 dan watchdog BARU dinyalakan di Fase 7. */
#define CALIB_BOOT_MAX_ATTEMPTS (3u)
#define CALIB_BOOT_RETRY_DELAY_MS (300u)

/**
 * Jalankan SATU percobaan kalibrasi accel/gyro blocking sampai DONE/FAILED/
 * timeout, pakai read_fn sebagai sumber sample (lihat read_raw_imu_primary/
 * secondary di atas). Asumsi pesawat diam total selama loop ini jalan.
 *
 * @return BOOT_CALIB_OK kalau *out terisi. Selain itu *out TIDAK diisi dan
 *         caller WAJIB menolak pemakaian IMU terkait (biarkan .calibrated
 *         false), BUKAN diam-diam lanjut dengan *out yang tidak terisi.
 */
static BootCalibResult_t run_boot_calibration_once(RawImuReadFn_t read_fn,
                                                   AccelGyroBias_t *out)
{
    CalibAccelGyro_Start();

    uint32_t start_ms = get_tick_ms();
    for (;;) {
        int16_t ax, ay, az, gx, gy, gz;
        if (read_raw_imu_primary(&ax, &ay, &az, &gx, &gy, &gz)) {
            CalibAccelGyro_FeedSample(ax, ay, az, gx, gy, gz);
        }

        CalibAccelGyroState_t state = CalibAccelGyro_GetState();
        if (state == CALIB_AG_STATE_DONE) {
            return CalibAccelGyro_GetResult(out) ? BOOT_CALIB_OK
                                                 : BOOT_CALIB_TIMEOUT;
        }
        if (state == CALIB_AG_STATE_FAILED) {
            CalibAccelGyro_Stop();
            return BOOT_CALIB_MOTION;
        }
        if ((get_tick_ms() - start_ms) > CALIB_BOOT_TIMEOUT_MS) {
            CalibAccelGyro_Stop();
            return BOOT_CALIB_TIMEOUT;
        }
    }
}

/**
 * Bungkus run_boot_calibration_once() dengan retry TERBATAS.
 *
 * Hanya BOOT_CALIB_MOTION yang di-retry: itu kegagalan yang penyebabnya
 * eksternal dan sementara (papan tergerak), jadi mencoba lagi punya peluang
 * nyata berhasil. BOOT_CALIB_TIMEOUT sengaja TIDAK di-retry -- artinya
 * sensor tidak pernah mengirim sample valid, dan mengulanginya hanya
 * membuang CALIB_BOOT_TIMEOUT_MS (8 detik) lagi untuk hasil yang sama.
 *
 * Gagal di sini BUKAN akhir: task_calib_recover() akan terus mencoba lagi
 * secara non-blocking setelah scheduler jalan, jadi boot tidak perlu
 * berkeras sampai berhasil.
 */
static bool run_boot_calibration(RawImuReadFn_t read_fn, AccelGyroBias_t *out)
{
    for (unsigned attempt = 0; attempt < CALIB_BOOT_MAX_ATTEMPTS; attempt++) {
        BootCalibResult_t r = run_boot_calibration_once(read_fn, out);
        if (r == BOOT_CALIB_OK) {
            return true;
        }
        if (r != BOOT_CALIB_MOTION) {
            return false;   /* TIMEOUT -- lihat alasan di komentar di atas */
        }
        if ((attempt + 1u) < CALIB_BOOT_MAX_ATTEMPTS) {
            delay_ms(CALIB_BOOT_RETRY_DELAY_MS);
        }
    }
    return false;
}

/* ===========================================================================
 * task_calib_recover -- pemulihan kalibrasi tanpa reboot (C4)
 * ===========================================================================
 * Sebelum ini, kalibrasi hanya terjadi sekali di boot. Kalau papan kebetulan
 * tergerak saat itu, .calibrated tetap false dan arming ditolak PERMANEN
 * sampai power-cycle -- tanpa indikasi apa pun. Task ini menghapus jalan
 * buntu itu: pengguna cukup meletakkan papan diam, kalibrasi pulih sendiri,
 * dan gerbang arming (yang dihitung ulang tiap cycle) ikut terbuka.
 *
 * Sifat yang WAJIB dijaga:
 *   - TIDAK memblokir. Satu tick = satu sample, sama persis dengan cara
 *     calib_accel_gyro.c memang didesain dipakai (lihat header-nya).
 *   - TIDAK pernah jalan saat armed (invarian keselamatan). Kalau armed
 *     datang di tengah jalan, kalibrasi dibatalkan, bukan diselesaikan.
 *   - Satu IMU pada satu waktu, karena calib_accel_gyro.c itu SINGLETON.
 *     Primer didahulukan; sekunder dikerjakan di sesi berikutnya.
 *
 * Sengaja TIDAK memakai CalibDispatcher_* (komentar asli, saat modul itu
 * belum terhubung ke mana pun). Sejak Comms #3, CMD_CALIB_* punya handler
 * di calib_dispatcher.c dan memakai singleton calib_accel_gyro.c yang SAMA
 * -- lihat CalibRecover_IsActive() di bawah: guard yang menolak start
 * manual accel/gyro selama task ini aktif. Siapa yang mengalah
 * (recovery vs operator) masih keputusan desain terbuka.
 * ===========================================================================*/
#define CALIB_RECOVER_BACKOFF_MS  (1000u)  /* jeda setelah FAILED: beri waktu
                                            * pengguna menaruh papan & getaran
                                            * mereda sebelum mencoba lagi */
#define CALIB_RECOVER_TIMEOUT_MS  (8000u)  /* jaring pengaman: kalau sensor
                                            * berhenti mengirim sample di
                                            * tengah sesi, jangan tersangkut
                                            * di RUNNING selamanya (layar akan
                                            * membeku di "DIAM 0%") */

/* "now sudah mencapai/melewati deadline", ditulis lewat selisih unsigned
 * supaya tetap benar saat get_tick_ms() wrap di ~49 hari. Perbandingan
 * langsung (now >= deadline) TIDAK aman melewati wrap. */
static inline bool tick_reached(uint32_t now, uint32_t deadline)
{
    return (uint32_t)(now - deadline) < 0x80000000u;
}

static void calib_recover_abort(void)
{
    if (s_calib_recover_state != CALIB_RECOVER_IDLE) {
        CalibAccelGyro_Stop();
        s_calib_recover_state = CALIB_RECOVER_IDLE;
    }
}

static void task_calib_recover(void *ctx, float dt_s)
{
    (void)ctx; (void)dt_s;

    /* Invarian keselamatan: tidak boleh jalan saat armed. Dicek PALING AWAL,
     * sebelum apa pun yang lain, supaya tidak ada jalur yang lolos. */
    if (s_fc.armed) {
        calib_recover_abort();
        return;
    }

    /* Gerbang sudah terpenuhi -> tidak ada yang perlu dipulihkan. Ini juga
     * yang menghentikan sesi begitu IMU terakhir selesai. */
    if (calib_gate_ok()) {
        calib_recover_abort();
        return;
    }

    uint32_t now = get_tick_ms();

    switch (s_calib_recover_state) {
    case CALIB_RECOVER_IDLE: {
        bool want_primary = s_fc.imu_primary_present && !s_bias_primary.calibrated;
        if (!want_primary) {
            /* Gerbang gagal karena IMU primer memang TIDAK TERPASANG, bukan
             * karena kalibrasi. Itu masalah hardware, bukan sesuatu yang bisa
             * diperbaiki dengan mengumpulkan sample. */
            return;
        }
        s_calib_recover_on_primary  = true;
        s_calib_recover_deadline_ms = now + CALIB_RECOVER_TIMEOUT_MS;
        CalibAccelGyro_Start();
        s_calib_recover_state = CALIB_RECOVER_RUNNING;
        break;
    }

    case CALIB_RECOVER_RUNNING: {
        if (!s_fc.imu_primary_present) {
            calib_recover_abort();
            return;
        }

        int16_t ax, ay, az, gx, gy, gz;
        if (read_raw_imu_primary(&ax, &ay, &az, &gx, &gy, &gz)) {
            CalibAccelGyro_FeedSample(ax, ay, az, gx, gy, gz);
        }

        CalibAccelGyroState_t st = CalibAccelGyro_GetState();
        if (st == CALIB_AG_STATE_DONE) {
            AccelGyroBias_t res;
            if (CalibAccelGyro_GetResult(&res)) {
                apply_calib_result(&s_bias_primary, &res);
            }
            calib_recover_abort();   /* kembali IDLE; IMU berikutnya (kalau ada)
                                      * diambil di tick selanjutnya */
        } else if (st == CALIB_AG_STATE_FAILED
                   || tick_reached(now, s_calib_recover_deadline_ms)) {
            /* FAILED = papan tergerak; deadline lewat = sensor berhenti kirim
             * sample. Keduanya ditangani sama: berhenti, tunggu sebentar, lalu
             * ulangi otomatis. */
            CalibAccelGyro_Stop();
            s_calib_recover_deadline_ms = now + CALIB_RECOVER_BACKOFF_MS;
            s_calib_recover_state = CALIB_RECOVER_BACKOFF;
        }
        break;
    }

    case CALIB_RECOVER_BACKOFF:
        if (tick_reached(now, s_calib_recover_deadline_ms)) {
            s_calib_recover_state = CALIB_RECOVER_IDLE;
        }
        break;

    default:
        s_calib_recover_state = CALIB_RECOVER_IDLE;
        break;
    }
}

/* Dibaca calib_dispatcher.c (guard CMD_CALIB_ACCEL_GYRO_START/STOP, Comms #3).
 * READ-ONLY -- tidak mengubah perilaku task_calib_recover() sama sekali.
 *
 * "Aktif" = sedang memakai singleton calib_accel_gyro.c (RUNNING/BACKOFF,
 * state != IDLE) ATAU segera akan memakainya: di state IDLE task ini
 * akan Start() pada tick berikutnya persis kalau kondisi yang sama dengan
 * cabang IDLE di atas terpenuhi (tidak armed, IMU primer ada tapi belum
 * terkalibrasi). Klausa kedua menutup celah satu-tick di antara dua
 * sesi (mis. antara BACKOFF -> IDLE -> RUNNING), kalau tidak start manual
 * yang lolos di celah itu akan langsung di-Start() ulang (di-reset) oleh
 * recovery. Kalau papan sudah terkalibrasi, recovery tidak pernah
 * jalan lagi dan operator bebas kalibrasi ulang manual. */
bool CalibRecover_IsActive(void)
{
    if (s_calib_recover_state != CALIB_RECOVER_IDLE) {
        return true;
    }
    return !s_fc.armed && s_fc.imu_primary_present && !s_bias_primary.calibrated;
}

/* Push telemetri berkala (Comms #4): CMD_ATTITUDE + CMD_GPS_DATA +
 * CMD_BATTERY lewat Protocol_SendUnsolicited() (request_id=0x00).
 * CMD_IMU_RAW TIDAK di sini -- on-demand (Telemetry_RegisterCommands()).
 *
 * GATE BSP_USB_CDC_IsHostOpen() (bukan sekadar IsConfigured): Protocol_
 * SendFrame() untuk request_id=0 menulis ke USB CDC yang blocking TANPA
 * timeout; kalau USB sudah enumerasi tapi tidak ada program yang membuka
 * port, endpoint IN tak pernah di-ACK -> busy-wait selamanya -> IWDG
 * (~500 ms) reset berulang. Konsekuensi gate ini: klien yang hanya
 * memakai jalur USART6 (USB-TTL cadangan) TIDAK menerima push -- keputusan
 * desain terbuka untuk tim, lihat laporan Comms #4.
 *
 * BIAYA WAKTU: kedua transport sama-sama blocking. USART6 115200 baud
 * = ~87 us/byte; 3 frame (20+19+11 = 50 byte) = ~4.3 ms busy-wait per
 * panggilan, hampir satu periode ctrl 200 Hz (5 ms) dan ~4 sampel IMU
 * 1 kHz. Sekali per 100 ms, dan hanya saat host membuka port. */
static void task_telemetry_push(void *ctx, float dt_s)
{
    (void)ctx; (void)dt_s;

    if (!BSP_USB_CDC_IsHostOpen()) {
        return;
    }

    const AHRS_Attitude_t *att = AHRS_Fusion_GetAttitude(&s_fc.ahrs);

    TelemetrySnapshot_t snap;
    snap.roll_deg          = att->roll_deg;
    snap.pitch_deg         = att->pitch_deg;
    snap.yaw_deg           = att->yaw_deg;
    snap.active_imu        = (uint8_t)att->active_imu;   /* MPU6500=0, MPU6050=1, cocok enum web */
    snap.lat_e7            = s_fc.gps_raw.lat_e7;
    snap.lon_e7            = s_fc.gps_raw.lon_e7;
    snap.ground_speed_mm_s = s_fc.gps_raw.ground_speed_mm_s;
    snap.nav_fix_type      = (int)s_fc.gps_sample.fix_type;
    snap.sat_count         = s_fc.gps_raw.num_sv;
    snap.voltage_volts     = s_fc.battery.voltage_volts;
    snap.current_amps      = s_fc.battery.current_amps;

    Telemetry_PushAll(&snap);
}

/* ===========================================================================
 * Gain PID awal — starting point uji terbang pertama (BUKAN hasil tuning)
 * ===========================================================================
 * Stabilize_Init() sengaja menyisakan gain 0,0,0 (lihat komentar di
 * stabilize.c) supaya "belum di-tuning" selalu berarti "diam", bukan
 * bergerak liar dengan angka acak. Konstanta di bawah ini yang mengisinya
 * di fase 5 main() -- dipilih KONSERVATIF untuk uji terbang PERTAMA
 * (verifikasi arah respons & kestabilan dasar di udara), BUKAN hasil
 * tuning lapangan. WAJIB ditinjau ulang (biasanya dinaikkan bertahap)
 * setelah sesi tuning pertama -- jangan anggap angka ini final.
 *
 * Dasar pemilihan angka:
 *   - Setpoint/measurement PID roll & pitch dalam derajat (lihat
 *     StabilizeInput_t di stabilize.h), output di-clamp ke [-1,1] oleh
 *     PID_Init() (lihat pid.c), dan limit sudut manual di task_control()
 *     adalah +-35 derajat. Kp=0.02 -> pada error maksimum 35 derajat,
 *     kontribusi P murni ~0.7, MASIH ada headroom sebelum output mentok
 *     +-1.0 -- supaya lonjakan sesaat (noise/disagreement dual-IMU) tidak
 *     langsung membanting permukaan kontrol ke limit fisiknya.
 *   - Ki=0.0 (integral OFF) untuk uji terbang pertama: arah & besar trim
 *     mekanis airframe ini belum pernah diverifikasi di udara. Integral
 *     yang aktif sebelum arah dikonfirmasi benar berisiko windup diam-diam
 *     ke arah yang salah sebelum operator sempat sadar. Aktifkan belakangan
 *     lewat SETTING_SET setelah P-only terbukti stabil.
 *   - Kd=0.002 (kecil) sekadar damping tambahan meredam overshoot dari
 *     P-term, cukup kecil supaya tidak ikut memperkuat noise gyro mentah
 *     (d_filter_alpha juga masih default 0.0 dari PID_Init(), tidak
 *     diubah di sini).
 *   - Roll & pitch dimulai dari angka yang SAMA karena belum ada data
 *     bench/lapangan yang membedakan respons kedua axis di airframe V-tail
 *     ini -- pisahkan begitu tuning nyata dimulai per axis.
 * ===========================================================================*/
#define STABILIZE_INITIAL_ROLL_KP    0.02f
#define STABILIZE_INITIAL_ROLL_KI    0.0f
#define STABILIZE_INITIAL_ROLL_KD    0.002f

#define STABILIZE_INITIAL_PITCH_KP   0.02f
#define STABILIZE_INITIAL_PITCH_KI   0.0f
#define STABILIZE_INITIAL_PITCH_KD   0.002f

/* ===========================================================================
 * main
 * ===========================================================================*/
int main(void)
{
    /* --- Fase 1: clock. Harus paling awal, sebelum BSP apa pun. --- */
    system_clock_status_t clk_status = SystemClock_Config();
    if (clk_status != SYSTEM_CLOCK_OK) {
        boot_fail_blink(clk_status); /* tidak pernah kembali */
    }

    SysTick_Init(SYSTEM_CORE_CLOCK_HZ);

    /* --- Fase 2: GPIO & bus --- */
    BSP_PinMap_Init();
    BSP_SPI1_Init();
    BSP_I2C1_Init();
    BSP_UART1_Init();          /* iBus RX @115200 */
    BSP_UART2_Init(9600UL);    /* GPS; TODO konfirmasi baud default modul */
    BSP_ADC_Init();
    BSP_USB_CDC_Init();
    BSP_UART6_Init(USART6_BAUD_CMD);  /* command link cadangan via USB-to-TTL, lihat bsp_uart.h */
    Protocol_Init();
    CommandHandler_Init();

    /* --- Fase 3: output. Diinisialisasi SEBELUM sensor, supaya servo
     *     langsung ke posisi netral dan ESC menerima sinyal "motor off"
     *     secepat mungkin setelah power-on — jangan tunggu sensor selesai
     *     probing (yang bisa makan ratusan ms kalau ada yang timeout). --- */
    DShot_Init();
    PwmServo_Init();

    /* --- Fase 4: sensor --- */
    /* Keterpasangan dicatat DI SINI, sebelum kalibrasi apa pun dijalankan
     * -- lihat komentar imu_*_present di FcState_t. */
    delay_ms(500U);
    s_fc.imu_primary_present   = MPU6500_Init();
    s_fc.has_baro          = BMP280_Init();
    s_fc.has_oled          = (OLED_Init(get_tick_ms()) == OLED_OK);
    GPS_UBX_Init();
    for (uint8_t i = 0; i < 10U && !GPS_UBX_ConfigDone(); i++) {
        (void)GPS_UBX_ConfigStep(gps_tx);
        delay_ms(30U);
    }
    s_gps_last_frame_ms = get_tick_ms();

    /* Mag: coba HMC5883 dulu, fallback ke QMC5883 (urutan arbitrer --
     * lihat catatan mag_hmc5883.h). Tidak coba keduanya kalau salah
     * satu terdeteksi tapi Init()-nya gagal -- itu tanda bus sedang
     * tidak sehat, bukan alasan untuk lanjut menebak chip lain. */
    if (MAG_HMC5883_Detect()) {
        s_fc.has_mag    = (MAG_HMC5883_Init() == HMC5883_OK);
        s_active_mag_chip = s_fc.has_mag ? MAG_CHIP_HMC5883 : MAG_CHIP_NONE;
    } else if (MAG_QMC5883_Detect()) {
        s_fc.has_mag    = (MAG_QMC5883_Init() == QMC5883_OK);
        s_active_mag_chip = s_fc.has_mag ? MAG_CHIP_QMC5883 : MAG_CHIP_NONE;
    } else {
        s_fc.has_mag = false;
        s_active_mag_chip = MAG_CHIP_NONE;
    }

    /* Baseline altitude di ground, disarmed. */
    if (s_fc.has_baro) {
        bmp280_data_t sample;
        if (BMP280_Read(&sample)) {
            BMP280_SetReferencePressure(sample.pressure_pa);
        }
    }

    /* DEBUG SEMENTARA: laporkan status init sensor lewat kedipan LED PC13.
     * Lihat komentar di atas debug_blink_sensor_status() untuk arti pola
     * kedipannya. Hapus baris ini (dan fungsinya di atas) setelah selesai
     * dipakai untuk diagnosa. */
    debug_blink_sensor_status();

    /* --- Fase 5: modul logika ---
     * RESOLVED — gain PID roll/pitch: sebelumnya 0,0,0 dari Stabilize_Init()
     * dan tidak pernah di-set, jadi permukaan kontrol tidak merespons
     * attitude sama sekali. Sekarang di-set manual segera setelah
     * Stabilize_Init() lewat Stabilize_SetRollGains()/SetPitchGains(),
     * pakai nilai starting point konservatif STABILIZE_INITIAL_* (lihat
     * blok komentar di atas main() untuk alasan pemilihan angka) --
     * SEMENTARA untuk uji terbang pertama, BUKAN hasil tuning final.
     * Kalibrasi gyro/accel: lihat blok RESOLVED terpisah tepat di bawah
     * Battery_Init() -- gain PID ini tidak bergantung urutan terhadap
     * kalibrasi (bias sensor & gain kontrol dua hal independen), jadi
     * sengaja dipanggil duluan di sini tanpa menunggu kalibrasi selesai. */
    AHRS_Fusion_Init(&s_fc.ahrs);

    /* Grup setting "compass" (offset_x/y/z, scale_x/y/z) -- didaftarkan
     * di sini supaya siap dipakai kalau/begitu Settings_Init()/
     * Settings_FlashLoad() disambungkan ke boot sequence. CATATAN: per
     * saat ditulis, main() ini BELUM memanggil Settings_Init() atau
     * Settings_FlashLoad() SAMA SEKALI (item terbuka LAIN, di luar
     * cakupan task compass fusion ini -- grup "mixer" yang menurut
     * protocol.md Bagian 9 sudah "final" juga belum terdaftar dengan
     * cara yang sama). Registrasi di bawah karena itu belum membuat
     * compass.offset_x dst. bisa diakses lewat CMD_SETTING_GET/SET dari
     * web sampai infrastruktur Settings itu disambungkan -- lihat
     * catatan panjang di fusion/compass.c. Kalibrasi hasil figure-8
     * (task_mag()) TETAP berfungsi tanpa ini (disimpan langsung ke
     * s_fc.compass_calib, bukan lewat jalur Settings). */
    Compass_RegisterSettings(&s_fc.compass_calib);

    Nav_Init(&s_fc.nav);
    {
        static const NAV_MissionItem_t wp0 = {
            .seq          = 0,
            .lat_e7       = -63319200,
            .lon_e7       = 1066934920,
            .altitude_m   = 2,                  /* sesuaikan */
            .action_type  = NAV_ACTION_WAYPOINT,
            .action_param = 0
        };
        static const NAV_MissionItem_t wp1 = {   /* opsional: pulang ke home setelah wp0 */
            .seq          = 1,
            .lat_e7       = 0,
            .lon_e7       = 0,
            .altitude_m   = 50,
            .action_type  = NAV_ACTION_RTH,
            .action_param = 0
        };
        Nav_ClearMission(&s_fc.nav);
        (void)Nav_LoadMissionItem(&s_fc.nav, &wp0);
        (void)Nav_LoadMissionItem(&s_fc.nav, &wp1);
    }
    Nav_RegisterProtocolHandlers(&s_fc.nav);
    Stabilize_Init(&s_fc.stabilize);
    /* CMD_MOTOR_TEST/CMD_SERVO_TEST (Comms #2) -- terikat ke instance
     * OutputMap_t yang sama dipakai jalur mixer normal
     * (OutputMap_WriteFromMixer() di task_control(), lihat di bawah),
     * supaya remap output lewat setting (kalau nanti ada) otomatis
     * konsisten di kedua jalur. Dipanggil SETELAH CommandHandler_Init()
     * (baris di atas, awal main()) supaya tidak ikut ter-reset tabel
     * registrasinya -- pola sama seperti Nav_RegisterProtocolHandlers()
     * tepat di atas. */
    OutputMap_RegisterCommands(&s_fc.stabilize.output_map);
    /* CMD_CALIB_* (Comms #3) -- SETELAH CommandHandler_Init() (awal main()),
     * sama alasan dengan OutputMap_RegisterCommands() di atas. */
    CalibDispatcher_RegisterCommands();
    /* CMD_IMU_RAW on-demand (Comms #4) -- setelah CommandHandler_Init(). */
    Telemetry_RegisterCommands(&s_fc.imu_raw);
    ArmedState_Init(); /* WAJIB dipanggil sekali sebelum scheduler jalan --
                         * lihat armed_state.h. Reset ke ARMED_STATE_DISARMED,
                         * sumber kebenaran status armed dipakai task_control()
                         * di bawah (menggantikan want_armed inline lama). */

    Stabilize_SetRollGains(&s_fc.stabilize,
                            STABILIZE_INITIAL_ROLL_KP,
                            STABILIZE_INITIAL_ROLL_KI,
                            STABILIZE_INITIAL_ROLL_KD);
    Stabilize_SetPitchGains(&s_fc.stabilize,
                             STABILIZE_INITIAL_PITCH_KP,
                             STABILIZE_INITIAL_PITCH_KI,
                             STABILIZE_INITIAL_PITCH_KD);

    RxIbus_Init(&s_fc.rx);
    /* TODO(Orang 3): SETELAH failsafe per-channel dikonfigurasi manual di
     * menu transmitter (mis. FS-i6/i6X: RX Setup -> Failsafe), isi channel
     * index + nilai failsafe-nya di sini -- lihat disclaimer "KENAPA TIDAK
     * CUKUP PAKAI DETEKSI TIMEOUT SAJA" di rx_ibus.h. Contoh kalau throttle
     * (channel index 2) di-set failsafe -100% di TX:
     *   RxIbus_SetFailsafeChannel(&s_fc.rx, 2U, IBUS_CH_VALUE_MIN, 20U);
     * Dibiarkan TIDAK dipanggil dulu (mekanisme #2 disabled by default) --
     * JANGAN uji terbang sebelum salah satu dari mekanisme #2 (ini) atau
     * #3 (bit-flag, lihat rx_ibus.c) terkonfirmasi jalan, karena timeout
     * murni (#1) TIDAK cukup untuk receiver FlySky/iA6B, lihat rx_ibus.h. */
    Battery_Init(&s_fc.battery_config, 3U); /* TODO: jumlah sel dari setting */

    /* RESOLVED — kalibrasi accel/gyro: sebelumnya tidak pernah dijalankan,
     * jadi s_bias_primary/secondary selalu nol (drift tidak terkompensasi).
     * Sekarang dijalankan blocking di sini lewat run_boot_calibration()
     * (lihat blok komentar di atas main() untuk alasan desainnya),
     * ASUMSI PESAWAT DIAM total selama fase ini. Primer & sekunder
     * dikalibrasi BERURUTAN (bukan bersamaan -- lihat catatan singleton*/
    {
        AccelGyroBias_t calib_result;

        /* Penyalinan hasil kini lewat apply_calib_result() (satu tempat,
         * termasuk perlakuan khusus sumbu Z) -- lihat fungsinya di atas.
         *
         * Kegagalan di sini TIDAK LAGI menghapus flag keterpasangan IMU:
         * imu_*_present diisi dari hasil Init() di Fase 4 dan tidak pernah
         * disentuh lagi. Yang menahan arming cukup .calibrated == false.
         * Ini aman karena CalibGate_ArmAllowed() menolak arming selama ada
         * IMU terpasang yang belum terkalibrasi, jadi data IMU tanpa bias
         * paling jauh hanya dipakai AHRS saat DISARMED. */
        if (s_fc.imu_primary_present && run_boot_calibration(read_raw_imu_primary, &calib_result)) {
            apply_calib_result(&s_bias_primary, &calib_result);
        } else {
            s_bias_primary.calibrated = false;
        }

    }

    /* --- Fase 6: scheduler --- */
    Scheduler_Init(&s_sched);

    Scheduler_AddTask(&s_sched, "ibus",   task_ibus_drain,    NULL, RATE_RX_DRAIN_HZ);
    Scheduler_AddTask(&s_sched, "usb",    task_protocol,       NULL, RATE_PROTOCOL_HZ);
    Scheduler_AddTask(&s_sched, "imu1",   task_imu_primary,   NULL, RATE_IMU_PRIMARY_HZ);
    Scheduler_AddTask(&s_sched, "ahrs",   task_ahrs,          NULL, RATE_AHRS_HZ);
    Scheduler_AddTask(&s_sched, "ctrl",   task_control,       NULL, RATE_CONTROL_HZ);
    Scheduler_AddTask(&s_sched, "dshot",  task_dshot_send,    NULL, RATE_DSHOT_HZ);
    Scheduler_AddTask(&s_sched, "fsafe",  task_failsafe,      NULL, RATE_FAILSAFE_HZ);
    Scheduler_AddTask(&s_sched, "baro",   task_baro,          NULL, RATE_BARO_HZ);
    Scheduler_AddTask(&s_sched, "mag",    task_mag,           NULL, RATE_MAG_HZ);
    Scheduler_AddTask(&s_sched, "gps",    task_gps,           NULL, RATE_GPS_HZ);
    Scheduler_AddTask(&s_sched, "nav",    task_nav,           NULL, RATE_NAV_HZ);
    Scheduler_AddTask(&s_sched, "batt",   task_battery,       NULL, RATE_BATTERY_HZ);
    Scheduler_AddTask(&s_sched, "oled",   task_oled,          NULL, RATE_OLED_HZ);
    /* Task ke-14 dari SCHEDULER_MAX_TASKS=16. Pemulihan kalibrasi tanpa
     * reboot (C4) -- lihat blok komentar di atas task_calib_recover(). */
    Scheduler_AddTask(&s_sched, "cal",    task_calib_recover, NULL, RATE_CALIB_RECOVER_HZ);
    /* Task ke-15 dari SCHEDULER_MAX_TASKS=16 (sisa 1 slot). */
    Scheduler_AddTask(&s_sched, "telem",  task_telemetry_push, NULL, RATE_TELEMETRY_HZ);

    /* --- Fase 7: watchdog, lalu loop utama --- */
    watchdog_init();

    for (;;) {
        Scheduler_Tick(&s_sched);
        watchdog_feed();
    }
}

static void task_protocol(void *ctx, float dt_s)
{
    (void)ctx;
    (void)dt_s;
    Protocol_Poll();
}