#ifndef BSP_I2C_H
#define BSP_I2C_H

#include "stm32f4xx.h"   /* CMSIS device header, target STM32F411xE */
#include <stdint.h>

/* ============================================================================
 * bsp_i2c.h
 * I2C1 -- PB6=SCL, PB7=SDA, AF4, open-drain + pull-up (sudah dikonfigurasi
 * oleh BSP_PinMap_Init(), modul ini hanya menyalakan peripheral I2C1 +
 * bus recovery).
 *
 * Mencakup 4 device di satu bus (update dari pinout-fc-stm32f411.md
 * Bagian 2):
 *   MPU6050 (IMU sekunder)   0x68
 *   BMP280  (baro)           0x76
 *   HMC5883 (mag, alt A)     0x1E
 *   QMC5883 (mag, alt B)     0x0D
 *   OLED SSD1306 (display)   0x3C
 *
 * Konsekuensi langsung dari 4 device di satu bus: satu slave yang macet
 * (menahan SDA low, mis. mati di tengah transaksi) bisa mengunci SEMUA
 * device lain di bus ini. BSP_I2C1_BusRecovery() ADALAH mitigasi untuk
 * kasus ini -- toggle SCL manual 9x (memberi slave macet kesempatan
 * menyelesaikan clock yang tertahan) diikuti STOP condition manual,
 * baru peripheral I2C1 di-reset & di-enable ulang.
 *
 * Clock tree acuan (final): APB1_CLOCK_HZ = 48 MHz (lihat bsp_spi.h,
 * definisi APB1_CLOCK_HZ dipakai bersama di sini supaya satu sumber
 * kebenaran, tidak didefinisikan ulang).
 *
 * Kecepatan bus: Fast Mode 400 kHz (cukup untuk baca IMU/baro/mag rutin
 * + refresh OLED tanpa membebani scheduler). Ganti I2C1_USE_FAST_MODE
 * ke 0 kalau tim memutuskan Standard Mode 100kHz saja.
 * ============================================================================ */

#define I2C1_USE_FAST_MODE   1   /* 1 = Fast Mode 400kHz, 0 = Standard Mode 100kHz */

/** @brief Kode hasil operasi I2C1. */
typedef enum {
    BSP_I2C_OK             = 0,
    BSP_I2C_ERR_TIMEOUT    = -1, /* salah satu tahap (START/ADDR/TXE/RXNE/BTF) timeout */
    BSP_I2C_ERR_NACK       = -2, /* slave NACK alamat atau data */
    BSP_I2C_ERR_BUS_BUSY   = -3, /* bus tidak kunjung idle sebelum START */
} bsp_i2c_status_t;

/**
 * @brief Enable clock APB1 untuk I2C1 dan konfigurasi peripheral (CCR,
 *        TRISE, mode Fast/Standard sesuai I2C1_USE_FAST_MODE). GPIO
 *        SCL/SDA diasumsikan sudah dikonfigurasi AF4 open-drain oleh
 *        BSP_PinMap_Init() -- dipanggil SEBELUM fungsi ini.
 *        Dipanggil sekali saat boot.
 */
void BSP_I2C1_Init(void);

/**
 * @brief Recovery bus I2C1 yang macet (slave menahan SDA low sehingga
 *        transaksi normal selalu timeout). Prosedur:
 *          1. Disable peripheral I2C1, ubah PB6/PB7 jadi GPIO manual
 *             (open-drain, sama seperti konfigurasi normalnya).
 *          2. Toggle SCL (PB6) HIGH-LOW sebanyak 9x sambil memantau SDA
 *             (PB7) -- 9 clock cukup untuk slave manapun menyelesaikan
 *             byte yang sedang ditahannya (worst case 8 bit data + 1 ACK).
 *          3. Generate STOP condition manual: pastikan SCL HIGH, lalu
 *             SDA LOW->HIGH selagi SCL tetap HIGH.
 *          4. Kembalikan PB6/PB7 ke mode AF4 I2C1 (lewat
 *             BSP_GPIO_ConfigPin() dari bsp_pinmap.h), lalu panggil
 *             BSP_I2C1_Init() ulang untuk reset penuh peripheral.
 *        Dipanggil oleh caller (mis. dari command_handler atau sensor
 *        driver) ketika operasi I2C1 mengembalikan BSP_I2C_ERR_TIMEOUT
 *        berulang kali (bukan dipanggil otomatis di dalam setiap
 *        read/write, supaya caller tetap kontrol atas retry policy-nya).
 * @return 0 kalau SDA berhasil dibebaskan (bus recovery sukses),
 *         -1 kalau SDA masih tertahan LOW setelah 9x toggle (kemungkinan
 *         short hardware / slave rusak permanen -- di luar yang bisa
 *         diperbaiki lewat protokol I2C).
 */
int BSP_I2C1_BusRecovery(void);

/**
 * @brief Tulis N byte ke device I2C1 (mis. untuk write-register:
 *        byte pertama = alamat register, sisanya = nilai).
 * @param dev_addr7 alamat 7-bit device (0x68, 0x76, 0x1E, 0x0D, 0x3C, dst)
 * @param data buffer yang dikirim
 * @param len jumlah byte di buffer
 * @return BSP_I2C_OK atau kode error
 */
bsp_i2c_status_t BSP_I2C1_Write(uint8_t dev_addr7, const uint8_t *data, uint16_t len);

/**
 * @brief Baca N byte dari device I2C1 (repeated-START, tanpa STOP di
 *        antara fase write-register dan read-data).
 * @param dev_addr7 alamat 7-bit device
 * @param reg alamat register awal yang mau dibaca
 * @param data buffer tujuan hasil baca
 * @param len jumlah byte yang mau dibaca
 * @return BSP_I2C_OK atau kode error
 */
bsp_i2c_status_t BSP_I2C1_ReadRegs(uint8_t dev_addr7, uint8_t reg, uint8_t *data, uint16_t len);

/**
 * @brief Tulis 1 register (helper tipis di atas BSP_I2C1_Write, dipakai
 *        hampir semua driver sensor: MPU6050/BMP280/HMC5883/QMC5883/OLED).
 */
bsp_i2c_status_t BSP_I2C1_WriteReg(uint8_t dev_addr7, uint8_t reg, uint8_t value);

#endif /* BSP_I2C_H */
