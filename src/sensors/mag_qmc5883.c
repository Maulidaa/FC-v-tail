/**
 * mag_qmc5883.c
 *
 * Lihat mag_qmc5883.h untuk kontrak publik & catatan asumsi API
 * bsp_i2c.c.
 * TODO (Orang 2): ganti nama fungsi BSP_I2C1_* di bawah kalau signature
 * asli di bsp_i2c.c berbeda dari asumsi.
 */

#include "mag_qmc5883.h"
#include "bsp_i2c.h"   /* TODO: sesuaikan path/nama header BSP I2C1 Orang 1 */
#include <stddef.h>

#define REG_DATA_X_L  (0x00u) /* urutan data: X, Y, Z, little-endian */
#define REG_CONTROL1  (0x09u)
#define REG_CONTROL2  (0x0Au)
#define REG_SETRESET  (0x0Bu)
#define REG_CHIP_ID   (0x0Du)
#define CHIP_ID_VAL   (0xFFu)

static bool s_ready = false;

static QMC5883_Status_t i2c_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (BSP_I2C1_ReadRegs(QMC5883L_I2C_ADDR, reg, buf, len) != BSP_I2C_OK) {
        BSP_I2C1_BusRecovery();
        return QMC5883_ERR_I2C;
    }
    return QMC5883_OK;
}

static QMC5883_Status_t i2c_write(uint8_t reg, uint8_t value)
{
    if (BSP_I2C1_WriteReg(QMC5883L_I2C_ADDR, reg, value) != BSP_I2C_OK) {
        BSP_I2C1_BusRecovery();
        return QMC5883_ERR_I2C;
    }
    return QMC5883_OK;
}

bool MAG_QMC5883_Detect(void)
{
    uint8_t id = 0;
    if (i2c_read(REG_CHIP_ID, &id, 1) != QMC5883_OK) {
        return false;
    }
    return (id == CHIP_ID_VAL);
}

QMC5883_Status_t MAG_QMC5883_Init(void)
{
    QMC5883_Status_t st;

    s_ready = false;

    /* Reset period register, nilai rekomendasi datasheet QMC5883L */
    st = i2c_write(REG_SETRESET, 0x01);
    if (st != QMC5883_OK) return st;

    /* Control1: OSR=512, full-scale=8G, ODR=200Hz, mode=continuous */
    st = i2c_write(REG_CONTROL1, 0x1D);
    if (st != QMC5883_OK) return st;

    /* Control2: pointer roll-over enable, interrupt disable */
    st = i2c_write(REG_CONTROL2, 0x40);
    if (st != QMC5883_OK) return st;

    s_ready = true;
    return QMC5883_OK;
}

QMC5883_Status_t MAG_QMC5883_ReadRaw(QMC5883_RawData_t *out)
{
    uint8_t raw[6];
    QMC5883_Status_t st;

    if (out == NULL) {
        return QMC5883_ERR_I2C;
    }
    if (!s_ready) {
        return QMC5883_ERR_NOT_INIT;
    }

    st = i2c_read(REG_DATA_X_L, raw, 6);
    if (st != QMC5883_OK) {
        return st;
    }

    /* Urutan output chip ini: X, Y, Z — little-endian, tidak perlu
     * penyusunan ulang seperti HMC5883L */
    out->x = (int16_t)(raw[0] | (raw[1] << 8));
    out->y = (int16_t)(raw[2] | (raw[3] << 8));
    out->z = (int16_t)(raw[4] | (raw[5] << 8));

    return QMC5883_OK;
}

bool MAG_QMC5883_IsReady(void)
{
    return s_ready;
}
