/**
 * mag_hmc5883.h
 *
 * Driver magnetometer HMC5883L, I2C1, address tetap 0x1E.
 * Sesuai firmware-architecture-stm32f411.md Bagian 2 (struktur folder
 * sensors/) — file terpisah dari mag_qmc5883.c/h karena register map
 * dua chip ini beda total meski sering dipasang di modul fisik yang
 * sama (pin-compatible).
 *
 * Caller (mis. main.c atau fusion/ahrs.c) yang menentukan chip mana
 * yang benar-benar terpasang: panggil MAG_HMC5883_Detect() dan
 * MAG_QMC5883_Detect() di boot, pakai yang mengembalikan true. Kedua
 * driver sengaja API-nya simetris (Detect/Init/ReadRaw/IsReady) supaya
 * logic pemilihan di caller sederhana — lihat catatan yang sama di
 * mag_qmc5883.h.
 *
 * CATATAN ASUMSI API bsp_i2c.c (sama seperti driver I2C lain):
 *   HAL_StatusTypeDef BSP_I2C1_ReadReg(uint8_t dev_addr, uint8_t reg,
 *                                      uint8_t *buf, uint16_t len);
 *   HAL_StatusTypeDef BSP_I2C1_WriteReg(uint8_t dev_addr, uint8_t reg,
 *                                       uint8_t value);
 *   void BSP_I2C1_BusRecovery(void);
 */

#ifndef MAG_HMC5883_H
#define MAG_HMC5883_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HMC5883L_I2C_ADDR (0x1Eu)

typedef enum {
    HMC5883_OK = 0,
    HMC5883_ERR_NOT_FOUND, /* identification register tidak cocok */
    HMC5883_ERR_I2C,       /* transaksi I2C gagal (NACK/timeout) */
    HMC5883_ERR_NOT_INIT   /* dipanggil sebelum Init() sukses */
} HMC5883_Status_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} HMC5883_RawData_t;

/**
 * Cek keberadaan chip lewat identification register (0x0A-0x0C, harus
 * berisi 'H','4','3'), tanpa mengubah konfigurasi apa pun. Dipakai
 * caller untuk memilih HMC5883L vs QMC5883L sebelum memanggil Init().
 */
bool MAG_HMC5883_Detect(void);

/**
 * Inisialisasi: set Config A (averaging+rate), Config B (gain), Mode
 * (continuous-measurement). Panggil setelah MAG_HMC5883_Detect()
 * mengembalikan true.
 */
HMC5883_Status_t MAG_HMC5883_Init(void);

/** Baca satu sample raw X/Y/Z. Urutan register native chip ini adalah
 * X,Z,Y (bukan X,Y,Z) — sudah dinormalisasi jadi {x,y,z} di sini. */
HMC5883_Status_t MAG_HMC5883_ReadRaw(HMC5883_RawData_t *out);

/** True kalau Init() sudah pernah sukses. */
bool MAG_HMC5883_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif /* MAG_HMC5883_H */
