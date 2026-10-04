/**
 * calib_accel_gyro.h
 *
 * Kalibrasi bias accel & gyro, device didiamkan di permukaan datar
 * (protocol.md Bagian 10 & command CMD_CALIB_ACCEL_GYRO_START/STOP,
 * ID 0x0401/0x0402).
 *
 * Modul ini TIDAK membaca sensor sendiri — caller (scheduler/main loop
 * yang sudah membaca IMU untuk keperluan lain) memanggil
 * CalibAccelGyro_FeedSample() tiap ada sample baru selama state
 * IN_PROGRESS. Desain ini menghindari duplikasi akses SPI1/I2C1 dari
 * dua tempat berbeda.
 *
 * Sumber sample: pakai IMU primer (MPU6500, SPI1) sebagai referensi
 * kalibrasi utama — bukan IMU sekunder (MPU6050). Kalau nanti perlu
 * kalibrasi kedua IMU terpisah, panggil modul ini dua kali dengan
 * instance berbeda (perlu refactor jadi non-singleton dulu; untuk v1
 * cukup satu instance karena hanya IMU primer yang dipakai kontrol).
 */

#ifndef CALIB_ACCEL_GYRO_H
#define CALIB_ACCEL_GYRO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Nilai selaras dengan field `state` di payload CMD_CALIB_STATUS
 * (protocol.md Bagian 10): 0=idle,1=in_progress,2=done,3=failed —
 * dipakai langsung tanpa mapping supaya command_handler tinggal cast. */
typedef enum {
    CALIB_AG_STATE_IDLE        = 0,
    CALIB_AG_STATE_IN_PROGRESS = 1,
    CALIB_AG_STATE_DONE        = 2,
    CALIB_AG_STATE_FAILED      = 3
} CalibAccelGyroState_t;

/* Bias hasil kalibrasi, satuan sama dengan raw sensor (LSB) — konversi
 * ke unit fisik dilakukan di layer fusion (Orang 3) yang sudah tahu
 * skala full-range IMU yang dipakai. */
typedef struct {
    int16_t accel_bias_x;
    int16_t accel_bias_y;
    int16_t accel_bias_z; /* bias di sekitar +1g, bukan dikurangi gravitasi */
    int16_t gyro_bias_x;
    int16_t gyro_bias_y;
    int16_t gyro_bias_z;
} AccelGyroBias_t;

/**
 * Mulai kalibrasi: reset akumulator, set state IN_PROGRESS.
 * Return false kalau kalibrasi lain (bukan modul ini) sedang berjalan —
 * pengecekan itu tanggung jawab caller (calib_dispatcher), modul ini
 * sendiri tidak tahu soal kalibrasi jenis lain.
 */
void CalibAccelGyro_Start(void);

/** Batalkan kalibrasi yang sedang berjalan, kembali ke IDLE. */
void CalibAccelGyro_Stop(void);

/**
 * Umpankan satu sample raw IMU. No-op kalau state bukan IN_PROGRESS
 * (aman dipanggil terus tiap cycle tanpa cek state di caller).
 */
void CalibAccelGyro_FeedSample(int16_t ax, int16_t ay, int16_t az,
                                int16_t gx, int16_t gy, int16_t gz);

CalibAccelGyroState_t CalibAccelGyro_GetState(void);

/** 0-100, dihitung dari jumlah_sample_terkumpul / target_sample. */
uint8_t CalibAccelGyro_GetProgressPercent(void);

/**
 * Salin hasil ke *out. Return false kalau state bukan DONE (out tidak
 * diisi) — caller wajib cek GetState() == DONE dulu, atau cukup cek
 * return value ini.
 */
bool CalibAccelGyro_GetResult(AccelGyroBias_t *out);

#ifdef __cplusplus
}
#endif

#endif /* CALIB_ACCEL_GYRO_H */
