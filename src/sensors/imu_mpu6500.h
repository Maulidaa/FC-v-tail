#ifndef IMU_MPU6500_H
#define IMU_MPU6500_H

#include <stdint.h>
#include <stdbool.h>

/* ===== Struct data output ===== */
typedef struct {
    int16_t accel_x;   // raw, sumbu X
    int16_t accel_y;   // raw, sumbu Y
    int16_t accel_z;   // raw, sumbu Z
    int16_t gyro_x;    // raw, sumbu X
    int16_t gyro_y;    // raw, sumbu Y
    int16_t gyro_z;    // raw, sumbu Z
    int16_t temp_raw;  // raw suhu, opsional untuk kompensasi
    uint32_t timestamp_ms;
    bool valid;         // true kalau pembacaan terakhir sukses
} mpu6500_data_t;

/* ===== API publik ===== */

/**
 * Inisialisasi peripheral SPI1 + GPIO (register langsung, tanpa HAL),
 * lalu cek WHO_AM_I dan konfigurasi MPU6500 (wake, sample rate, DLPF, range).
 * Return true kalau device terdeteksi dan konfigurasi sukses.
 */
bool MPU6500_Init(void);

/**
 * Baca data accel+gyro (+ temp) sekali burst read (14 byte berurutan
 * dari ACCEL_XOUT_H). Mengisi struct mpu6500_data_t.
 * Return true kalau transaksi SPI sukses.
 */
bool MPU6500_ReadRaw(mpu6500_data_t *out);

/**
 * Cek ulang apakah device masih menjawab di bus (WHO_AM_I).
 * Dipakai modul fusion untuk flag "active IMU".
 */
bool MPU6500_IsAlive(void);

#endif // IMU_MPU6500_H
