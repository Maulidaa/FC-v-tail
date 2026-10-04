#include "baro_bmp280.h"
#include "bsp_i2c.h"
#include <math.h>
#include <stddef.h> // untuk NULL

extern uint32_t get_tick_ms(void);

#define BMP280_REG_CHIP_ID     0xD0
#define BMP280_REG_CTRL_MEAS   0xF4
#define BMP280_REG_CONFIG      0xF5
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_CALIB_START 0x88  // 26 byte koefisien kalibrasi, dari sini

#define BMP280_CHIP_ID_VAL     0x58

/* ===== Koefisien kalibrasi, dibaca sekali dari chip saat init.
 * Nama variabel & formula kompensasi mengikuti datasheet Bosch BMP280
 * (referensi resmi, wajib supaya nilai raw jadi satuan fisik yang benar). */
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
} bmp280_calib_t;

static bmp280_calib_t s_calib;
static int32_t s_t_fine;
static uint32_t s_reference_pressure_pa = 101325; // default: sea-level standar,
                                                     // sebaiknya di-override via
                                                     // BMP280_SetReferencePressure() saat ground

static bool bmp280_read_calibration(void)
{
    uint8_t buf[26];
    if (BSP_I2C1_ReadRegs(BMP280_I2C_ADDR, BMP280_REG_CALIB_START, buf, sizeof(buf)) != BSP_I2C_OK) {
        return false;
    }

    s_calib.dig_T1 = (uint16_t)(buf[0]  | (buf[1]  << 8));
    s_calib.dig_T2 = (int16_t) (buf[2]  | (buf[3]  << 8));
    s_calib.dig_T3 = (int16_t) (buf[4]  | (buf[5]  << 8));
    s_calib.dig_P1 = (uint16_t)(buf[6]  | (buf[7]  << 8));
    s_calib.dig_P2 = (int16_t) (buf[8]  | (buf[9]  << 8));
    s_calib.dig_P3 = (int16_t) (buf[10] | (buf[11] << 8));
    s_calib.dig_P4 = (int16_t) (buf[12] | (buf[13] << 8));
    s_calib.dig_P5 = (int16_t) (buf[14] | (buf[15] << 8));
    s_calib.dig_P6 = (int16_t) (buf[16] | (buf[17] << 8));
    s_calib.dig_P7 = (int16_t) (buf[18] | (buf[19] << 8));
    s_calib.dig_P8 = (int16_t) (buf[20] | (buf[21] << 8));
    s_calib.dig_P9 = (int16_t) (buf[22] | (buf[23] << 8));

    return true;
}

/* Kompensasi suhu, formula resmi Bosch (int32, fixed-point).
 * Return suhu dalam 0.01 derajat C, sekaligus set t_fine untuk dipakai
 * kompensasi tekanan (saling bergantung, sesuai algoritma datasheet). */
static int32_t bmp280_compensate_temperature(int32_t adc_T)
{
    int32_t var1, var2;

    var1 = ((((adc_T >> 3) - ((int32_t)s_calib.dig_T1 << 1))) * ((int32_t)s_calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)s_calib.dig_T1)) *
              ((adc_T >> 4) - ((int32_t)s_calib.dig_T1))) >> 12) * ((int32_t)s_calib.dig_T3)) >> 14;

    s_t_fine = var1 + var2;
    return (s_t_fine * 5 + 128) >> 8;
}

/* Kompensasi tekanan, formula resmi Bosch (int64, fixed-point Q24.8).
 * Perlu t_fine dari kompensasi suhu yang baru dihitung (urutan panggil penting). */
static uint32_t bmp280_compensate_pressure(int32_t adc_P)
{
    int64_t var1, var2, p;

    var1 = ((int64_t)s_t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)s_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)s_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)s_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)s_calib.dig_P3) >> 8) + ((var1 * (int64_t)s_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)s_calib.dig_P1) >> 33;

    if (var1 == 0) {
        return 0; // hindari divide-by-zero, chip belum terkalibrasi/rusak
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)s_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)s_calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)s_calib.dig_P7) << 4);

    return (uint32_t)(p / 256); // Q24.8 -> Pascal murni
}

bool BMP280_Init(void)
{
    BSP_I2C1_Init(); // aman dipanggil ulang, sudah di-init sensor lain di bus yang sama

    uint8_t chip_id = 0;
    if (BSP_I2C1_ReadRegs(BMP280_I2C_ADDR, BMP280_REG_CHIP_ID, &chip_id, 1) != BSP_I2C_OK) {
        return false;
    }
    if (chip_id != BMP280_CHIP_ID_VAL) {
        return false; // chip tidak terdeteksi / salah address / salah wiring
    }

    if (!bmp280_read_calibration()) {
        return false;
    }

    /* ctrl_meas: osrs_t=1 (x1, suhu tidak perlu presisi tinggi untuk kompensasi),
     * osrs_p=3 (x4, standar untuk FC -- cukup presisi, tidak terlalu lambat),
     * mode=3 (normal mode, sampling terus-menerus otomatis) */
    uint8_t ctrl_meas = (1 << 5) | (3 << 2) | 3;
    if (BSP_I2C1_WriteReg(BMP280_I2C_ADDR, BMP280_REG_CTRL_MEAS, ctrl_meas) != BSP_I2C_OK) {
        return false;
    }

    /* config: standby time 0.5ms (t_sb=0), filter koefisien 4 (redam noise
     * turbulensi udara di sekitar FC tanpa bikin altitude hold lag berlebih) */
    uint8_t config = (0 << 5) | (4 << 2);
    BSP_I2C1_WriteReg(BMP280_I2C_ADDR, BMP280_REG_CONFIG, config);

    return true;
}

bool BMP280_Read(bmp280_data_t *out)
{
    if (out == NULL) return false;

    uint8_t buf[6];
    if (BSP_I2C1_ReadRegs(BMP280_I2C_ADDR, BMP280_REG_PRESS_MSB, buf, sizeof(buf)) != BSP_I2C_OK) {
        out->valid = false;
        return false;
    }

    int32_t adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[2] >> 4);
    int32_t adc_T = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | (buf[5] >> 4);

    out->temperature_c_x100 = bmp280_compensate_temperature(adc_T); // urutan wajib sebelum pressure
    out->pressure_pa = bmp280_compensate_pressure(adc_P);

    /* Formula barometric standar, pakai float single-precision (powf) --
     * sesuai FPU hardware M4F, hindari double supaya tidak jatuh ke
     * emulasi software float yang berat (lihat catatan arsitektur). */
    float ratio = (float)out->pressure_pa / (float)s_reference_pressure_pa;
    out->altitude_m = 44330.0f * (1.0f - powf(ratio, 0.190295f));

    out->timestamp_ms = get_tick_ms();
    out->valid = true;

    return true;
}

void BMP280_SetReferencePressure(uint32_t pressure_pa)
{
    s_reference_pressure_pa = pressure_pa;
}
