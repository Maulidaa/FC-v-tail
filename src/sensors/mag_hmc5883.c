/**
 * mag_hmc5883.c
 *
 * Lihat mag_hmc5883.h untuk kontrak publik & catatan asumsi API
 * bsp_i2c.c.
 * TODO (Orang 2): ganti nama fungsi BSP_I2C1_* di bawah kalau signature
 * asli di bsp_i2c.c berbeda dari asumsi.
 */

#include "mag_hmc5883.h"
#include "bsp_i2c.h"   /* TODO: sesuaikan path/nama header BSP I2C1 Orang 1 */
#include <stddef.h>

#define REG_CONFIG_A  (0x00u)
#define REG_CONFIG_B  (0x01u)
#define REG_MODE      (0x02u)
#define REG_DATA_X_H  (0x03u) /* urutan data: X, Z, Y (bukan X,Y,Z!) */
#define REG_IDENT_A   (0x0Au)
#define IDENT_A_VAL   (0x48u) /* 'H' */
#define IDENT_B_VAL   (0x34u) /* '4' */
#define IDENT_C_VAL   (0x33u) /* '3' */

static bool s_ready = false;

static HMC5883_Status_t i2c_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (BSP_I2C1_ReadRegs(HMC5883L_I2C_ADDR, reg, buf, len) != BSP_I2C_OK) {
        BSP_I2C1_BusRecovery();
        return HMC5883_ERR_I2C;
    }
    return HMC5883_OK;
}

static HMC5883_Status_t i2c_write(uint8_t reg, uint8_t value)
{
    if (BSP_I2C1_WriteReg(HMC5883L_I2C_ADDR, reg, value) != BSP_I2C_OK) {
        BSP_I2C1_BusRecovery();
        return HMC5883_ERR_I2C;
    }
    return HMC5883_OK;
}

bool MAG_HMC5883_Detect(void)
{
    uint8_t ident[3];
    if (i2c_read(REG_IDENT_A, ident, 3) != HMC5883_OK) {
        return false;
    }
    return (ident[0] == IDENT_A_VAL &&
            ident[1] == IDENT_B_VAL &&
            ident[2] == IDENT_C_VAL);
}

HMC5883_Status_t MAG_HMC5883_Init(void)
{
    HMC5883_Status_t st;

    s_ready = false;

    /* Config A: 8-sample average, 15Hz output rate, normal measurement */
    st = i2c_write(REG_CONFIG_A, 0x70);
    if (st != HMC5883_OK) return st;

    /* Config B: gain = 1090 LSB/Gauss, range default, cukup untuk heading */
    st = i2c_write(REG_CONFIG_B, 0x20);
    if (st != HMC5883_OK) return st;

    /* Mode: continuous-measurement */
    st = i2c_write(REG_MODE, 0x00);
    if (st != HMC5883_OK) return st;

    s_ready = true;
    return HMC5883_OK;
}

HMC5883_Status_t MAG_HMC5883_ReadRaw(HMC5883_RawData_t *out)
{
    uint8_t raw[6];
    HMC5883_Status_t st;

    if (out == NULL) {
        return HMC5883_ERR_I2C;
    }
    if (!s_ready) {
        return HMC5883_ERR_NOT_INIT;
    }

    st = i2c_read(REG_DATA_X_H, raw, 6);
    if (st != HMC5883_OK) {
        return st;
    }

    /* Urutan output chip ini: X, Z, Y — big-endian */
    out->x = (int16_t)((raw[0] << 8) | raw[1]);
    out->z = (int16_t)((raw[2] << 8) | raw[3]);
    out->y = (int16_t)((raw[4] << 8) | raw[5]);

    return HMC5883_OK;
}

bool MAG_HMC5883_IsReady(void)
{
    return s_ready;
}
