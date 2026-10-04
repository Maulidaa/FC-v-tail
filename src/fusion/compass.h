/**
 * @file    compass.h
 * @brief   Kompas: kalibrasi hard/soft-iron + heading tilt-compensated
 *          dari magnetometer, dipakai sebagai referensi yaw absolut di
 *          ahrs_fusion.c.
 *
 * Tanggung jawab: Orang 3 (Fusion, Navigasi & Failsafe) -- modul ini
 * murni komputasi (tidak membaca sensor sendiri, tidak menyimpan state
 * kalibrasi sesi -- itu tetap tanggung jawab calibration/calib_mag.c).
 * compass.c hanya menerjemahkan MagCalibResult_t (hasil sesi kalibrasi)
 * dan sample raw mag jadi heading derajat yang siap dipakai fusion.
 *
 * ALUR PAKAI (lihat task_ahrs() di main.c):
 *   1. Compass_ApplyCalibration()  -- raw LSB -> unit terkalibrasi (float)
 *   2. Compass_RemapToBody()       -- axis chip -> body frame (X depan,
 *                                      Y kiri, Z atas). Terverifikasi bangku
 *                                      untuk QMC5883 (lihat catatan di
 *                                      compass.c) -- BELUM diverifikasi untuk
 *                                      HMC5883, dan main.c belum bercabang
 *                                      per-chip di titik panggilnya.
 *   3. Compass_ComputeHeadingDeg() -- body frame + roll/pitch -> heading
 *                                      derajat, searah jarum jam dari utara
 *                                      magnetik [0,360), TANPA deklinasi.
 *
 * KONVENSI HEADING: disamakan dengan RTH_CalcDistanceBearing() (nav/rth.c)
 * -- searah jarum jam dari utara, [0,360) -- supaya nav (task terpisah
 * setelah ini) tidak perlu konversi ganda. INI BUKAN konvensi yang sama
 * dengan AHRS_Attitude_t.yaw_deg (yang naik berlawanan jarum jam kalau
 * gyro_z positif) -- konversi antara keduanya jadi tanggung jawab
 * ahrs_fusion.c saat blending (lihat komentar ComputeSingleImuAttitude()),
 * BUKAN compass.c.
 */

#ifndef COMPASS_H
#define COMPASS_H

#include <stdbool.h>
#include <stdint.h>

#include "calib_mag.h"   /* MagCalibResult_t */

#ifdef __cplusplus
extern "C" {
#endif

/** State kalibrasi kompas yang sedang dipakai runtime. Caller (main.c)
 *  memegang satu instance ini di FcState_t (mis. s_fc.compass_calib) --
 *  modul ini tidak menyimpan salinan sendiri untuk nilai kalibrasi
 *  (beda dari Settings_RegisterField(), lihat catatan di compass.c). */
typedef struct {
    int16_t offset_x, offset_y, offset_z;   /* hard-iron, satuan raw LSB */
    float   scale_x, scale_y, scale_z;      /* soft-iron */
    bool    valid;                          /* default false */
} CompassCalib_t;

/**
 * @brief  Salin hasil kalibrasi (dari CalibMag_GetResult()) ke *ctx dan
 *         set valid=true. Dipanggil sekali per transisi kalibrasi ke
 *         DONE (lihat task_mag() di main.c), atau dari readback
 *         Settings_FlashLoad() kalau grup "compass" punya nilai valid.
 */
void Compass_SetCalibration(CompassCalib_t *ctx, const MagCalibResult_t *result);

/** @brief  true kalau *ctx sudah pernah diisi kalibrasi valid. */
bool Compass_HasCalibration(const CompassCalib_t *ctx);

/**
 * @brief  Terapkan hard-iron offset + soft-iron scale ke satu sample
 *         raw mag. No-op (mengisi 0.0f ke ketiga output) kalau
 *         !Compass_HasCalibration(ctx) -- caller WAJIB cek
 *         Compass_HasCalibration() sendiri sebelum memakai hasilnya
 *         untuk heading (lihat task_ahrs()), fungsi ini tidak menolak
 *         memproses secara eksplisit.
 */
void Compass_ApplyCalibration(const CompassCalib_t *ctx,
                              int16_t raw_x, int16_t raw_y, int16_t raw_z,
                              float *out_x, float *out_y, float *out_z);

/**
 * @brief  Remap axis mag (sudah terkalibrasi) ke body frame firmware
 *         ini (X depan, Y kiri, Z atas -- sama seperti konvensi
 *         accel/gyro di ahrs_fusion.c). Sudah diverifikasi bangku untuk
 *         QMC5883 (lihat komentar di compass.c untuk derivasi dan
 *         angka error numerik) -- masih WAJIB satu kali uji putar fisik
 *         langsung di board sebagai konfirmasi akhir sebelum terbang
 *         (heading naik saat diputar ke kanan, stabil saat roll/pitch
 *         berubah). BELUM diverifikasi untuk HMC5883, dan main.c
 *         (task_ahrs()) belum bercabang per-chip di titik panggilnya --
 *         lihat catatan di compass.c.
 */
void Compass_RemapToBody(float cal_x, float cal_y, float cal_z,
                         float *out_bx, float *out_by, float *out_bz);

/**
 * @brief  Heading tilt-compensated dari sample mag body-frame (X
 *         depan, Y kiri, Z atas -- output Compass_RemapToBody()) dan
 *         roll/pitch saat ini (derajat, konvensi AHRS_Attitude_t:
 *         roll positif = sayap kanan turun, pitch positif = hidung
 *         turun).
 *
 * @return Heading searah jarum jam dari utara magnetik, [0,360).
 *         Deklinasi TIDAK diterapkan (declination=0 untuk v1).
 */
float Compass_ComputeHeadingDeg(float mag_x, float mag_y, float mag_z,
                                float roll_deg, float pitch_deg);

/**
 * @brief  Daftarkan grup setting "compass" (offset_x/y/z, scale_x/y/z)
 *         lewat Settings_RegisterField(), pola sama seperti contoh grup
 *         "mixer" di settings.h. WAJIB dipanggil setelah Settings_Init()
 *         dan sebelum Settings_FlashLoad() di boot sequence main() --
 *         lihat CATATAN PENTING di compass.c: saat ditulis, main.c BELUM
 *         memanggil Settings_Init()/Settings_FlashLoad() sama sekali
 *         (item terbuka lain, di luar cakupan task ini -- lihat
 *         kalibrasi-imu.md), jadi field ini terdaftar tapi belum bisa
 *         diakses lewat CMD_SETTING_* sampai infrastruktur itu
 *         disambungkan.
 *
 * *ctx disimpan sebagai pointer statis internal (dipakai callback
 * get_fn/set_fn yang tidak punya parameter context) -- ctx WAJIB tetap
 * hidup selama program berjalan (pakai s_fc.compass_calib, bukan
 * variabel stack).
 */
void Compass_RegisterSettings(CompassCalib_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* COMPASS_H */
