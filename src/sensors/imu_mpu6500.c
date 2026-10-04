#include "imu_mpu6500.h"
#include "bsp_spi.h"
#include <stddef.h> // untuk NULL

/* ---- GPIOA (CS=PA4) ---- */
//#define GPIOA_BASE          0x40020000UL
#define GPIOA_MODER         (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_OSPEEDR       (*(volatile uint32_t *)(GPIOA_BASE + 0x08))
#define GPIOA_BSRR          (*(volatile uint32_t *)(GPIOA_BASE + 0x18))
#define RCC_AHB1ENR         (*(volatile uint32_t *)(0x40023800UL + 0x30))
//#define RCC_AHB1ENR_GPIOAEN (1UL << 0)

#define MPU6500_CS_PIN       4U   /* PA4, TODO: sesuaikan pinout-fc-stm32f411.md */

/* Timing: disediakan modul sistem lain (mis. systick/bsp_timer), TIDAK
 * diimplementasi ulang di sini supaya tidak dobel dengan timer module kamu. */
extern uint32_t get_tick_ms(void);
extern void     delay_ms(uint32_t ms);

/* ===== Register map MPU6500 (subset yang dipakai) ===== */
#define MPU6500_REG_SMPLRT_DIV     0x19
#define MPU6500_REG_CONFIG         0x1A
#define MPU6500_REG_GYRO_CONFIG    0x1B
#define MPU6500_REG_ACCEL_CONFIG   0x1C
#define MPU6500_REG_ACCEL_XOUT_H   0x3B
#define MPU6500_REG_PWR_MGMT_1     0x6B
#define MPU6500_REG_WHO_AM_I       0x75

#define MPU6500_WHO_AM_I_VAL       0x70  // nilai default chip MPU6500
#define MPU6500_WHO_AM_I_ALT_VAL   0x68
#define MPU6500_WHO_AM_I_MPU9250   0x73
#define MPU6500_READ_BIT           0x80  // MSB=1 -> operasi read

/* ===== CS pin -- ini hak MPU6500 sendiri, bukan urusan bsp_spi (bus generik) ===== */

static void mpu6500_cs_gpio_init(void)
{
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA_MODER &= ~(3UL << (MPU6500_CS_PIN * 2));
    GPIOA_MODER |=  (1UL << (MPU6500_CS_PIN * 2));   // 01 = output
    GPIOA_OSPEEDR |= (3UL << (MPU6500_CS_PIN * 2));  // high speed
    GPIOA_BSRR = (1UL << MPU6500_CS_PIN);            // idle HIGH
}

static inline void mpu6500_cs_low(void)
{
    GPIOA_BSRR = (1UL << (MPU6500_CS_PIN + 16)); // reset bit -> CS low
}

static inline void mpu6500_cs_high(void)
{
    GPIOA_BSRR = (1UL << MPU6500_CS_PIN); // set bit -> CS high
}

/* ===== Register read/write MPU6500, lewat bsp_spi (bus generik) ===== */

static bool mpu6500_write_reg(uint8_t reg, uint8_t value)
{
    mpu6500_cs_low();
    BSP_SPI1_TransferByte((uint8_t)(reg & 0x7F)); // MSB=0 untuk write
    BSP_SPI1_TransferByte(value);
    BSP_SPI1_WaitNotBusy();
    mpu6500_cs_high();
    return true;
}

static bool mpu6500_read_reg(uint8_t reg, uint8_t *value)
{
    if (value == NULL) return false;

    mpu6500_cs_low();
    BSP_SPI1_TransferByte((uint8_t)(reg | MPU6500_READ_BIT));
    *value = BSP_SPI1_TransferByte(0x00); // dummy byte untuk clock out data
    BSP_SPI1_WaitNotBusy();
    mpu6500_cs_high();
    return true;
}

static bool mpu6500_read_regs(uint8_t start_reg, uint8_t *buf, uint8_t len)
{
    if (buf == NULL || len == 0) return false;

    mpu6500_cs_low();
    BSP_SPI1_TransferByte((uint8_t)(start_reg | MPU6500_READ_BIT));
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = BSP_SPI1_TransferByte(0x00);
    }
    BSP_SPI1_WaitNotBusy();
    mpu6500_cs_high();
    return true;
}

static bool mpu6500_is_valid_id(uint8_t id)
{
    return (id == MPU6500_WHO_AM_I_VAL ||
            id == MPU6500_WHO_AM_I_ALT_VAL ||
            id == MPU6500_WHO_AM_I_MPU9250);
}

/* ===== API publik ===== */

bool MPU6500_Init(void)
{
    BSP_SPI1_Init();          // init bus (clock, GPIO AF, SPI1 peripheral)
    mpu6500_cs_gpio_init();   // init CS pin milik chip ini sendiri

    mpu6500_cs_high(); // pastikan idle high sebelum mulai

    uint8_t who_am_i = 0;
    if (!mpu6500_read_reg(MPU6500_REG_WHO_AM_I, &who_am_i)) {
        return false;
    }
    if (!mpu6500_is_valid_id(who_am_i)) {
        return false; // chip tidak terdeteksi / salah wiring
    }

    // Wake dari sleep mode, pilih clock source PLL gyro X (lebih stabil)
    mpu6500_write_reg(MPU6500_REG_PWR_MGMT_1, 0x01);
    delay_ms(10);

    // Sample rate divider: 1kHz gyro output / (1 + 0) = 1kHz (bisa disesuaikan)
    mpu6500_write_reg(MPU6500_REG_SMPLRT_DIV, 0x00);

    // DLPF: bandwidth 20Hz, mengurangi noise gyro saat kalibrasi dan fusion
    mpu6500_write_reg(MPU6500_REG_CONFIG, 0x04);

    // Gyro range +-500 dps (FS_SEL=1)
    mpu6500_write_reg(MPU6500_REG_GYRO_CONFIG, 0x08);

    // Accel range +-4g (AFS_SEL=1)
    mpu6500_write_reg(MPU6500_REG_ACCEL_CONFIG, 0x08);

    return true;
}

bool MPU6500_ReadRaw(mpu6500_data_t *out)
{
    if (out == NULL) {
        return false;
    }

    uint8_t buf[14];
    if (!mpu6500_read_regs(MPU6500_REG_ACCEL_XOUT_H, buf, sizeof(buf))) {
        out->valid = false;
        return false;
    }

    out->accel_x  = (int16_t)((buf[0]  << 8) | buf[1]);
    out->accel_y  = (int16_t)((buf[2]  << 8) | buf[3]);
    out->accel_z  = (int16_t)((buf[4]  << 8) | buf[5]);
    out->temp_raw = (int16_t)((buf[6]  << 8) | buf[7]);
    out->gyro_x   = (int16_t)((buf[8]  << 8) | buf[9]);
    out->gyro_y   = (int16_t)((buf[10] << 8) | buf[11]);
    out->gyro_z   = (int16_t)((buf[12] << 8) | buf[13]);

    out->timestamp_ms = get_tick_ms();
    out->valid = true;

    return true;
}

bool MPU6500_IsAlive(void)
{
    uint8_t who_am_i = 0;
    if (!mpu6500_read_reg(MPU6500_REG_WHO_AM_I, &who_am_i)) {
        return false;
    }
    return mpu6500_is_valid_id(who_am_i);
}
