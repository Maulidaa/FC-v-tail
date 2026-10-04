/**
 * imu_mpu6050.h
 *
 * Driver untuk MPU6050 (IMU sekunder, dipakai untuk arbitrasi dual-IMU
 * bersama MPU6500 pada SPI1). Sesuai pinout-fc-stm32f411.md Bagian 2:
 *   - Bus   : I2C1 (PB6=SCL, PB7=SDA, AF4) — sudah ada di bsp_i2c.c
 *   - Addr  : 0x68 (7-bit)
 *
 * Modul ini HANYA membaca data mentah (raw accel/gyro). Logika arbitrasi
 * disagreement dual-IMU adalah tanggung jawab Orang 3 (fusion/AHRS) —
 * driver ini tidak boleh melakukan filtering/fusion apa pun.
 *
 * CATATAN ASUMSI API bsp_i2c.c (sesuaikan nama fungsi kalau berbeda
 * dengan implementasi aktual Orang 1):
 *   HAL_StatusTypeDef BSP_I2C1_ReadReg(uint8_t dev_addr, uint8_t reg,
 *                                      uint8_t *buf, uint16_t len);
 *   HAL_StatusTypeDef BSP_I2C1_WriteReg(uint8_t dev_addr, uint8_t reg,
 *                                       uint8_t value);
 *   void BSP_I2C1_BusRecovery(void);
 *
 * Kalau signature sebenarnya beda, cukup ubah bagian wrapper di
 * imu_mpu6050.c — struct publik & fungsi publik di header ini tidak perlu
 * ikut berubah.
 */

#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_I2C_ADDR        (0x68u)
#define MPU6050_I2C_ADDR_ALT    (0x69u)
#define MPU6050_WHOAMI_EXPECTED (0x68u)

/* Status hasil operasi driver, dipakai konsisten di seluruh fungsi publik */
typedef enum {
    MPU6050_OK = 0,
    MPU6050_ERR_NOT_FOUND,   /* WHO_AM_I tidak cocok / device tidak menjawab */
    MPU6050_ERR_I2C,         /* transaksi I2C gagal (NACK/timeout) */
    MPU6050_ERR_NOT_INIT     /* dipanggil sebelum MPU6050_Init() sukses */
} MPU6050_Status_t;

/* Data mentah, satuan LSB (belum diskalakan) — konversi ke unit fisik
 * (g / deg-per-s) dilakukan di layer fusion (Orang 3), bukan di sini,
 * supaya driver ini tetap sensor-agnostic terhadap full-scale range. */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp_raw;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} MPU6050_RawData_t;

/**
 * Inisialisasi device: cek WHO_AM_I, wake dari sleep mode (PWR_MGMT_1),
 * set sample rate divider + full-scale range default.
 * Harus dipanggil sekali di boot, setelah BSP_I2C1_Init().
 */
MPU6050_Status_t MPU6050_Init(void);

/**
 * Baca WHO_AM_I register (0x75), untuk self-test / health check berkala
 * tanpa perlu re-init penuh.
 */
MPU6050_Status_t MPU6050_WhoAmI(uint8_t *out_id);

/**
 * Baca seluruh blok data (accel+temp+gyro, 14 byte berurutan mulai
 * ACCEL_XOUT_H 0x3B) dalam satu transaksi I2C.
 */
MPU6050_Status_t MPU6050_ReadRaw(MPU6050_RawData_t *out);

/**
 * True kalau Init() sudah pernah sukses. Dipakai caller (mis. scheduler
 * atau logika arbitrasi Orang 3) untuk skip IMU ini dari fusion kalau
 * memang tidak pernah terpasang / gagal init, tanpa menganggapnya error
 * berulang di setiap cycle.
 */
bool MPU6050_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_MPU6050_H */
