/**
 * @file    ahrs_fusion.h
 * @brief   AHRS / Fusion dual-IMU + logika arbitrasi disagreement.
 *
 * Tanggung jawab: Orang 3 (Fusion, Navigasi & Failsafe)
 *
 * Modul ini mengonsumsi data mentah dari dua IMU:
 *   - MPU6500 (SPI1, primer)   -> driver: sensors/imu_mpu6500.c (Orang 2)
 *   - MPU6050 (I2C1, sekunder) -> driver: sensors/imu_mpu6050.c (Orang 2)
 *
 * Output modul ini adalah attitude (roll/pitch/yaw) + flag IMU yang
 * dipakai hasil arbitrasi, sesuai payload CMD_ATTITUDE (protocol.md
 * Bagian 5, ID 0x0101):
 *   "roll/pitch/yaw + flag IMU aktif hasil arbitrasi dual-IMU"
 *
 * Arbitrasi TIDAK hanya cek salah satu IMU mati — juga mendeteksi
 * disagreement (keduanya hidup tapi datanya beda signifikan), sesuai
 * pembagian tugas (pembagian-tugas-firmware-4-orang.md, Orang 3).
 */

#ifndef AHRS_FUSION_H
#define AHRS_FUSION_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------- */
/* Konfigurasi (TODO: tuning pasca uji terbang / bench test)            */
/* ------------------------------------------------------------------- */

/** Rate loop fusion yang diharapkan (Hz). Koordinasikan ke Orang 1
 *  supaya scheduler menjadwalkan AHRS_Fusion_Update() di rate ini. */
#define AHRS_FUSION_RATE_HZ            200U
#define AHRS_FUSION_DT_S               (1.0f / AHRS_FUSION_RATE_HZ)

/** Konstanta complementary filter (bobot gyro vs accel/mag).
 *  TODO: tuning — ini nilai awal yang wajar, bukan final. */
#define AHRS_COMP_FILTER_GYRO_WEIGHT   0.98f

/** Ambang disagreement antar dua IMU sebelum dianggap "tidak sepakat".
 *  Dalam derajat, dibandingkan tiap update. TODO: tuning berdasarkan
 *  noise floor MPU6500 vs MPU6050 hasil kalibrasi Orang 2. */
#define AHRS_DISAGREEMENT_THRESHOLD_DEG    15.0f

/** Berapa banyak update berturut-turut disagreement sebelum failover
 *  IMU aktif (hindari flapping akibat noise sesaat). */
#define AHRS_DISAGREEMENT_CONFIRM_COUNT    5U

/* ------------------------------------------------------------------- */
/* Tipe data                                                            */
/* ------------------------------------------------------------------- */

/** Identitas IMU, dipakai untuk flag "IMU aktif" di CMD_ATTITUDE. */
typedef enum {
    AHRS_IMU_MPU6500 = 0,   /* primer, SPI1 */
    AHRS_IMU_MPU6050 = 1,   /* sekunder, I2C1 */
} AHRS_ImuSource_t;

/** Status kesehatan tiap IMU dari sudut pandang fusion. */
typedef enum {
    AHRS_IMU_STATUS_OK = 0,
    AHRS_IMU_STATUS_TIMEOUT,     /* tidak ada data baru dalam batas waktu */
    AHRS_IMU_STATUS_DISAGREE,    /* hidup tapi menyimpang dari IMU lain */
} AHRS_ImuStatus_t;

/** Sample mentah dari satu IMU. Diisi oleh caller (dari driver Orang 2)
 *  sebelum dipanggil ke AHRS_Fusion_Update(). Unit: gyro rad/s,
 *  accel dalam satuan g. */
typedef struct {
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    bool  data_valid;      /* false kalau driver gagal baca frame ini */
} AHRS_ImuSample_t;

/** Sample heading dari kompas (fusion/compass.c), dibentuk caller
 *  sekali per panggilan AHRS_Fusion_Update() dan dipakai sebagai
 *  referensi tambahan untuk yaw di KEDUA cabang IMU (bukan dua sample
 *  terpisah -- kompas cuma satu, dipakai bersama).
 *
 *  heading_deg: searah jarum jam dari utara magnetik, [0,360) -- sama
 *  konvensi dengan RTH_CalcDistanceBearing() (nav/rth.c). INI BEDA
 *  dari AHRS_Attitude_t.yaw_deg (naik berlawanan jarum jam kalau
 *  gyro_z positif) -- konversi ditangani internal di ahrs_fusion.c,
 *  caller tidak perlu mengubah heading_deg sebelum mengisi field ini.
 *
 *  valid=false (belum ada mag, atau ada tapi belum terkalibrasi) WAJIB
 *  membuat AHRS_Fusion_Update() fallback persis ke perilaku gyro-only
 *  sebelumnya -- lihat komentar di ComputeSingleImuAttitude()
 *  (ahrs_fusion.c). heading_deg tidak dibaca sama sekali saat
 *  valid=false. */
typedef struct {
    float heading_deg;
    bool  valid;
} AHRS_MagSample_t;

/** Output attitude hasil fusion — field ini yang dipetakan langsung
 *  ke payload CMD_ATTITUDE oleh command_handler.c (Orang 1). */
typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    AHRS_ImuSource_t active_imu;   /* IMU yang jadi sumber attitude saat ini */
    bool  disagreement_flag;       /* true kalau sedang dalam status disagree */
} AHRS_Attitude_t;

/** State internal modul. Caller cukup punya satu instance ini,
 *  di-init sekali lalu di-update tiap loop. */
typedef struct {
    AHRS_Attitude_t   attitude;
    AHRS_ImuStatus_t  status_primary;
    AHRS_ImuStatus_t  status_secondary;
    uint32_t          disagreement_count;
    uint32_t          last_update_tick_ms;
    bool              initialized;
} AHRS_FusionState_t;

/* ------------------------------------------------------------------- */
/* API                                                                  */
/* ------------------------------------------------------------------- */

/**
 * @brief  Inisialisasi state fusion. Panggil sekali saat boot,
 *         setelah kedua driver IMU (Orang 2) selesai init.
 */
void AHRS_Fusion_Init(AHRS_FusionState_t *state);

/**
 * @brief  Update fusion dengan sample terbaru dari kedua IMU.
 *         Dipanggil oleh scheduler (Orang 1) di rate AHRS_FUSION_RATE_HZ.
 *
 * @param  state       State fusion (in/out).
 * @param  primary     Sample dari MPU6500 (SPI1). data_valid=false kalau
 *                      driver gagal baca / timeout.
 * @param  secondary   Sample dari MPU6050 (I2C1). data_valid=false kalau
 *                      driver gagal baca / timeout (mis. I2C1 macet —
 *                      lihat BSP_I2C1_BusRecovery() di pinout doc Bagian 2).
 * @param  mag         Heading kompas (fusion/compass.c), dipakai sebagai
 *                      referensi yaw tambahan di kedua cabang IMU. valid=false
 *                      -> fallback gyro-only, regresi nol dari perilaku
 *                      sebelumnya (lihat AHRS_MagSample_t).
 * @param  tick_ms     Timestamp saat ini (ms), untuk deteksi timeout &
 *                      integrasi gyro yang presisi kalau rate tidak stabil.
 */
void AHRS_Fusion_Update(AHRS_FusionState_t *state,
                         const AHRS_ImuSample_t *primary,
                         const AHRS_ImuSample_t *secondary,
                         const AHRS_MagSample_t *mag,
                         uint32_t tick_ms);

/**
 * @brief  Ambil hasil attitude terakhir. Dipanggil command_handler.c
 *         saat menyusun payload CMD_ATTITUDE (0x0101).
 */
const AHRS_Attitude_t *AHRS_Fusion_GetAttitude(const AHRS_FusionState_t *state);

/**
 * @brief  Cek apakah attitude saat ini layak dipakai untuk kontrol
 *         (mis. kedua IMU timeout -> hasil tidak reliable). Dipakai
 *         Orang 4 (mixer/PID) dan Orang 3 sendiri (failsafe/RTH) untuk
 *         memutuskan fallback.
 */
bool AHRS_Fusion_IsAttitudeValid(const AHRS_FusionState_t *state);

#ifdef __cplusplus
}
#endif

#endif /* AHRS_FUSION_H */
