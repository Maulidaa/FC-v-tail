#ifndef BARO_BMP280_H
#define BARO_BMP280_H

#include <stdint.h>
#include <stdbool.h>

/* Address 7-bit tergantung pin SDO: LOW=0x76 (default), HIGH=0x77 */
#define BMP280_I2C_ADDR   0x76

typedef struct {
    int32_t temperature_c_x100; // suhu dalam 0.01 derajat C (mis. 2534 = 25.34C)
    uint32_t pressure_pa;        // tekanan dalam Pascal
    float altitude_m;            // ketinggian relatif terhadap tekanan referensi
    uint32_t timestamp_ms;
    bool valid;
} bmp280_data_t;

/**
 * Inisialisasi BMP280: cek chip ID, baca koefisien kalibrasi dari register
 * chip (wajib, tanpa ini data mentah tidak bisa dikompensasi), lalu set
 * mode operasi (normal mode, oversampling standar untuk FC).
 */
bool BMP280_Init(void);

/**
 * Baca data mentah, kompensasi jadi suhu+tekanan pakai koefisien kalibrasi,
 * lalu hitung altitude relatif dari tekanan referensi permukaan laut standar.
 */
bool BMP280_Read(bmp280_data_t *out);

/**
 * Set tekanan referensi (Pa) sebagai acuan altitude=0, dipanggil sekali
 * saat ground/disarmed sebelum takeoff (baseline lokal, bukan sea-level).
 */
void BMP280_SetReferencePressure(uint32_t pressure_pa);

#endif // BARO_BMP280_H
