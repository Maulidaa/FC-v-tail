/**
 * imu_mpu6050.c
 *
 * Lihat imu_mpu6050.h untuk kontrak publik & catatan asumsi API bsp_i2c.c.
 * TODO (Orang 2): ganti nama fungsi BSP_I2C1_* di bawah kalau signature
 * asli di bsp_i2c.c berbeda dari asumsi.
 */

#include <stddef.h>
#include "imu_mpu6050.h"
#include "bsp_i2c.h"   /* TODO: sesuaikan path/nama header BSP I2C1 Orang 1 */

/* ---- Register map MPU6050 (datasheet register map rev 4.2) ---- */
#define REG_WHO_AM_I        (0x75u)
#define REG_PWR_MGMT_1      (0x6Bu)
#define REG_SMPLRT_DIV      (0x19u)
#define REG_CONFIG          (0x1Au)
#define REG_GYRO_CONFIG     (0x1Bu)
#define REG_ACCEL_CONFIG    (0x1Cu)
#define REG_ACCEL_XOUT_H    (0x3Bu)

#define RAW_BLOCK_LEN       (14u) /* accel(6) + temp(2) + gyro(6) */

static bool s_ready = false;
static uint8_t s_i2c_addr = MPU6050_I2C_ADDR;

static bsp_i2c_status_t i2c_read_at(uint8_t address, uint8_t reg,
                                    uint8_t *buf, uint16_t len)
{
    return BSP_I2C1_ReadRegs(address, reg, buf, len);
}

static MPU6050_Status_t i2c_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (i2c_read_at(s_i2c_addr, reg, buf, len) != BSP_I2C_OK) {
        BSP_I2C1_BusRecovery();
        return MPU6050_ERR_I2C;
    }
    return MPU6050_OK;
}

static MPU6050_Status_t i2c_write(uint8_t reg, uint8_t value)
{
    if (BSP_I2C1_WriteReg(s_i2c_addr, reg, value) != BSP_I2C_OK) {
        BSP_I2C1_BusRecovery();
        return MPU6050_ERR_I2C;
    }
    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_WhoAmI(uint8_t *out_id)
{
    if (out_id == NULL) {
        return MPU6050_ERR_I2C;
    }
    return i2c_read(REG_WHO_AM_I, out_id, 1);
}

MPU6050_Status_t MPU6050_Init(void)
{
    uint8_t who = 0;
    MPU6050_Status_t st;
    const uint8_t candidate_addresses[] = {
        MPU6050_I2C_ADDR,
        MPU6050_I2C_ADDR_ALT
    };

    s_ready = false;

    /* AD0 memilih alamat I2C: coba kedua kemungkinan alamat 7-bit. */
    st = MPU6050_ERR_NOT_FOUND;
    for (uint8_t i = 0; i < sizeof(candidate_addresses); i++) {
        s_i2c_addr = candidate_addresses[i];
        if (i2c_read_at(s_i2c_addr, REG_WHO_AM_I, &who, 1) == BSP_I2C_OK &&
            who == MPU6050_WHOAMI_EXPECTED) {
            st = MPU6050_OK;
            break;
        }
    }
    if (st != MPU6050_OK) {
        BSP_I2C1_BusRecovery();
        return MPU6050_ERR_NOT_FOUND;
    }

    /* Wake dari sleep mode default, pilih clock source PLL gyro-X
     * (lebih stabil dari internal 8MHz oscillator) */
    st = i2c_write(REG_PWR_MGMT_1, 0x01);
    if (st != MPU6050_OK) {
        return st;
    }

    /* Sample rate = 1kHz / (1 + SMPLRT_DIV). Divider=9 -> 100Hz,
     * cukup untuk role sekunder/arbitrasi (bukan primary control loop). */
    st = i2c_write(REG_SMPLRT_DIV, 9);
    if (st != MPU6050_OK) {
        return st;
    }

    /* DLPF ~44Hz bandwidth (CONFIG=3), gyro ±500 dps (GYRO_CONFIG=0x08),
     * accel ±4g (ACCEL_CONFIG=0x08). Nilai default, boleh dipindah ke
     * setting tunable kalau nanti dibutuhkan lewat SETTING_GET/SET. */
    st = i2c_write(REG_CONFIG, 0x03);
    if (st != MPU6050_OK) {
        return st;
    }
    st = i2c_write(REG_GYRO_CONFIG, 0x08);
    if (st != MPU6050_OK) {
        return st;
    }
    st = i2c_write(REG_ACCEL_CONFIG, 0x08);
    if (st != MPU6050_OK) {
        return st;
    }

    s_ready = true;
    return MPU6050_OK;
}

MPU6050_Status_t MPU6050_ReadRaw(MPU6050_RawData_t *out)
{
    uint8_t raw[RAW_BLOCK_LEN];
    MPU6050_Status_t st;

    if (out == NULL) {
        return MPU6050_ERR_I2C;
    }
    if (!s_ready) {
        return MPU6050_ERR_NOT_INIT;
    }

    st = i2c_read(REG_ACCEL_XOUT_H, raw, RAW_BLOCK_LEN);
    if (st != MPU6050_OK) {
        return st;
    }

    out->accel_x  = (int16_t)((raw[0]  << 8) | raw[1]);
    out->accel_y  = (int16_t)((raw[2]  << 8) | raw[3]);
    out->accel_z  = (int16_t)((raw[4]  << 8) | raw[5]);
    out->temp_raw = (int16_t)((raw[6]  << 8) | raw[7]);
    out->gyro_x   = (int16_t)((raw[8]  << 8) | raw[9]);
    out->gyro_y   = (int16_t)((raw[10] << 8) | raw[11]);
    out->gyro_z   = (int16_t)((raw[12] << 8) | raw[13]);

    return MPU6050_OK;
}

bool MPU6050_IsReady(void)
{
    return s_ready;
}
