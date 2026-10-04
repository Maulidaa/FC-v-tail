/**
 * mag_qmc5883.h
 *
 * Driver magnetometer QMC5883L, I2C1, address tetap 0x0D.
 * Alternatif pin-compatible dari HMC5883L (lihat mag_hmc5883.h) —
 * sering dipasang di modul fisik yang sama, tapi register map beda
 * total, karena itu file terpisah sesuai
 * firmware-architecture-stm32f411.md Bagian 2.
 *
 * API sengaja dibuat simetris dengan mag_hmc5883.h (Detect/Init/
 * ReadRaw/IsReady) supaya caller bisa coba salah satu chip dulu, lalu
 * yang lain kalau tidak terdeteksi — lihat contoh pola di
 * mag_hmc5883.h.
 *
 * ORIENTASI FISIK: axis mentah dari MAG_QMC5883_ReadRaw() di bawah
 * BELUM sejajar body frame firmware (X depan, Y kiri, Z atas) — chip
 * ini terpasang terputar 90 derajat pada bidang horizontal relatif
 * body. Remap ke body frame (hasil verifikasi bangku) ada di
 * fusion/compass.c (Compass_RemapToBody()), BUKAN di file ini — driver
 * ini sengaja tetap murni register read/write tanpa pengetahuan
 * tentang body frame.
 *

 * CATATAN ASUMSI API bsp_i2c.c (sama seperti driver I2C lain):
 *   HAL_StatusTypeDef BSP_I2C1_ReadReg(uint8_t dev_addr, uint8_t reg,
 *                                      uint8_t *buf, uint16_t len);
 *   HAL_StatusTypeDef BSP_I2C1_WriteReg(uint8_t dev_addr, uint8_t reg,
 *                                       uint8_t value);
 *   void BSP_I2C1_BusRecovery(void);
 */

#ifndef MAG_QMC5883_H
#define MAG_QMC5883_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define QMC5883L_I2C_ADDR (0x0Du)

typedef enum {
    QMC5883_OK = 0,
    QMC5883_ERR_NOT_FOUND, /* chip ID register tidak cocok */
    QMC5883_ERR_I2C,       /* transaksi I2C gagal (NACK/timeout) */
    QMC5883_ERR_NOT_INIT   /* dipanggil sebelum Init() sukses */
} QMC5883_Status_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} QMC5883_RawData_t;

/**
 * Cek keberadaan chip lewat chip ID register (0x0D, harus berisi
 * 0xFF), tanpa mengubah konfigurasi apa pun. Dipakai caller untuk
 * memilih QMC5883L vs HMC5883L sebelum memanggil Init().
 */
bool MAG_QMC5883_Detect(void);

/**
 * Inisialisasi: set SET/RESET period, Control1 (OSR/range/output
 * rate/mode), Control2 (pointer roll-over). Panggil setelah
 * MAG_QMC5883_Detect() mengembalikan true.
 */
QMC5883_Status_t MAG_QMC5883_Init(void);

/** Baca satu sample raw X/Y/Z. Urutan register native chip ini sudah
 * X,Y,Z little-endian — tidak perlu penyusunan ulang seperti HMC5883L. */
QMC5883_Status_t MAG_QMC5883_ReadRaw(QMC5883_RawData_t *out);

/** True kalau Init() sudah pernah sukses. */
bool MAG_QMC5883_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif /* MAG_QMC5883_H */
