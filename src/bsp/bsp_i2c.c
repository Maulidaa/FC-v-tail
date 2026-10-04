#include "bsp_i2c.h"
#include "bsp_pinmap.h"

/* ============================================================================
 * bsp_i2c.c
 * Implementasi bare-metal I2C1 (register langsung, peripheral I2C klasik
 * STM32F4 -- bukan I2C v2), termasuk bus recovery untuk 4 device di bus
 * ini (MPU6050, BMP280, HMC5883/QMC5883, OLED SSD1306).
 *
 * CATATAN TIMEOUT: loop timeout di modul ini pakai counter busy-wait
 * (bukan systick/RTC), karena modul scheduler (checklist Bagian C) belum
 * ada saat file ini ditulis. Nilai I2C_TIMEOUT_LOOPS dipilih longgar
 * (jauh di atas waktu wajar satu transaksi di 400kHz) supaya tidak
 * false-positive timeout di kondisi normal. Kalau scheduler dengan
 * time-base sudah ada, pertimbangkan ganti ke timeout berbasis waktu
 * asli -- tidak mengubah API publik di bsp_i2c.h.
 * ============================================================================ */

#define I2C_TIMEOUT_LOOPS   100000u

/* APB1_CLOCK_HZ didefinisikan di bsp_spi.h (satu sumber kebenaran untuk
 * clock APB1, dipakai bersama SPI2 & I2C1). Include tidak langsung lewat
 * bsp_spi.h di sini supaya bsp_i2c.c tetap independen -- definisikan
 * ulang nilainya secara eksplisit dan jaga tetap sinkron manual dengan
 * bsp_spi.h kalau clock tree berubah lagi di masa depan. */
#define APB1_CLOCK_MHZ   48u
#define APB1_CLOCK_HZ    (APB1_CLOCK_MHZ * 1000000u)

/* ---------------------------------------------------------------------------
 * Helper: tunggu sampai (reg & mask) == expect, atau timeout.
 * Return 1 kalau berhasil, 0 kalau timeout.
 * ------------------------------------------------------------------------- */
static int wait_flag(const volatile uint32_t *reg, uint32_t mask, uint32_t expect)
{
    uint32_t loops = I2C_TIMEOUT_LOOPS;
    while (((*reg) & mask) != expect) {
        if (--loops == 0u) {
            return 0;
        }
    }
    return 1;
}

/* Cek apakah slave NACK alamat (AF flag di SR1). Kalau ya, clear AF +
 * generate STOP supaya bus tidak nyangkut, lalu return 1 (NACK terjadi). */
static int check_and_handle_nack(void)
{
    if (I2C1->SR1 & I2C_SR1_AF) {
        I2C1->SR1 &= ~I2C_SR1_AF; /* clear AF */
        I2C1->CR1 |= I2C_CR1_STOP;
        return 1;
    }
    return 0;
}

/* ---------------------------------------------------------------------------
 * Init
 * ------------------------------------------------------------------------- */

void BSP_I2C1_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    __asm volatile ("nop");
    __asm volatile ("nop");

    I2C1->CR1 = 0; /* disable dulu, reset konfigurasi */

    /* CR2: frekuensi clock APB1 dalam MHz, dipakai peripheral untuk
     * timing internal (bukan kecepatan bus I2C itu sendiri -- itu diatur
     * CCR di bawah). */
    I2C1->CR2 = (I2C1->CR2 & ~I2C_CR2_FREQ) | (APB1_CLOCK_MHZ & 0x3Fu);

#if I2C1_USE_FAST_MODE
    /* Fast Mode 400kHz, duty cycle Tlow/Thigh = 2 (duty=0):
     * CCR = PCLK1 / (3 * Fscl) = 48MHz / (3*400kHz) = 40
     * TRISE = (PCLK1_MHz * 300 / 1000) + 1 = (48*300/1000)+1 = 15 */
    {
        uint32_t ccr = APB1_CLOCK_HZ / (3u * 400000u);
        I2C1->CCR = I2C_CCR_FS | (ccr & I2C_CCR_CCR); /* FS=1 (fast mode), DUTY=0 */
        I2C1->TRISE = ((APB1_CLOCK_MHZ * 300u) / 1000u) + 1u;
    }
#else
    /* Standard Mode 100kHz:
     * CCR = PCLK1 / (2 * Fscl) = 48MHz / (2*100kHz) = 240
     * TRISE = PCLK1_MHz + 1 = 49 */
    {
        uint32_t ccr = APB1_CLOCK_HZ / (2u * 100000u);
        I2C1->CCR = (ccr & I2C_CCR_CCR); /* FS=0 (standard mode) */
        I2C1->TRISE = APB1_CLOCK_MHZ + 1u;
    }
#endif

    I2C1->CR1 |= I2C_CR1_ACK; /* ACK default ON, ditoggle per-transaksi saat read */
    I2C1->CR1 |= I2C_CR1_PE;  /* enable peripheral */
}

/* ---------------------------------------------------------------------------
 * Bus recovery
 * ------------------------------------------------------------------------- */

int BSP_I2C1_BusRecovery(void)
{
    /* 1. Matikan peripheral I2C1, pindah SCL/SDA ke GPIO manual open-drain
     *    (kondisi elektris sama seperti mode AF I2C1: open-drain + pull-up,
     *    supaya sesuai spec bus I2C selama bit-banging recovery). */
    I2C1->CR1 &= ~I2C_CR1_PE;

    BSP_GPIO_ConfigPin(PIN_I2C1_SCL_PORT, PIN_I2C1_SCL_PIN,
                        PINMAP_MODE_OUTPUT, 0,
                        PINMAP_OTYPE_OD, PINMAP_SPEED_HIGH, PINMAP_PUPD_UP);
    BSP_GPIO_ConfigPin(PIN_I2C1_SDA_PORT, PIN_I2C1_SDA_PIN,
                        PINMAP_MODE_INPUT, 0,
                        PINMAP_OTYPE_OD, PINMAP_SPEED_HIGH, PINMAP_PUPD_UP);

    /* 2. Toggle SCL 9x, pantau SDA tiap siklus. 9 clock = cukup untuk
     *    slave manapun menuntaskan byte yang sedang ditahan (maks 8 bit
     *    data + 1 bit ACK) lalu melepas SDA. */
    int sda_released = 0;
    for (int i = 0; i < 9; i++) {
        /* SCL LOW */
        PIN_I2C1_SCL_PORT->BSRR = (1u << (PIN_I2C1_SCL_PIN + 16u));
        for (volatile int d = 0; d < 200; d++) { /* delay singkat, ~standard mode half-period */ }

        /* SCL HIGH */
        PIN_I2C1_SCL_PORT->BSRR = (1u << PIN_I2C1_SCL_PIN);
        for (volatile int d = 0; d < 200; d++) { }

        if (PIN_I2C1_SDA_PORT->IDR & (1u << PIN_I2C1_SDA_PIN)) {
            sda_released = 1;
            /* Tetap lanjutkan toggle sampai 9x meski sudah released di
             * tengah jalan -- supaya slave benar2 selesai frame-nya,
             * bukan berhenti di posisi ambigu. */
        }
    }

    if (!sda_released && !(PIN_I2C1_SDA_PORT->IDR & (1u << PIN_I2C1_SDA_PIN))) {
        /* SDA masih LOW setelah 9x toggle penuh -- recovery gagal. */
        return -1;
    }

    /* 3. Generate STOP condition manual: SDA LOW -> SCL HIGH -> SDA HIGH
     *    (SDA naik selagi SCL HIGH = STOP condition standar I2C). */
    BSP_GPIO_ConfigPin(PIN_I2C1_SDA_PORT, PIN_I2C1_SDA_PIN,
                        PINMAP_MODE_OUTPUT, 0,
                        PINMAP_OTYPE_OD, PINMAP_SPEED_HIGH, PINMAP_PUPD_UP);

    PIN_I2C1_SDA_PORT->BSRR = (1u << (PIN_I2C1_SDA_PIN + 16u)); /* SDA LOW */
    for (volatile int d = 0; d < 200; d++) { }
    PIN_I2C1_SCL_PORT->BSRR = (1u << PIN_I2C1_SCL_PIN);          /* SCL HIGH (sudah HIGH, jaga-jaga) */
    for (volatile int d = 0; d < 200; d++) { }
    PIN_I2C1_SDA_PORT->BSRR = (1u << PIN_I2C1_SDA_PIN);          /* SDA HIGH -> STOP */
    for (volatile int d = 0; d < 200; d++) { }

    /* 4. Kembalikan PB6/PB7 ke AF4 I2C1, lalu reset penuh peripheral
     *    lewat BSP_I2C1_Init() ulang. */
    BSP_GPIO_ConfigPin(PIN_I2C1_SCL_PORT, PIN_I2C1_SCL_PIN,
                        PINMAP_MODE_AF, AF4_I2C1,
                        PINMAP_OTYPE_OD, PINMAP_SPEED_HIGH, PINMAP_PUPD_UP);
    BSP_GPIO_ConfigPin(PIN_I2C1_SDA_PORT, PIN_I2C1_SDA_PIN,
                        PINMAP_MODE_AF, AF4_I2C1,
                        PINMAP_OTYPE_OD, PINMAP_SPEED_HIGH, PINMAP_PUPD_UP);

    BSP_I2C1_Init();
    return 0;
}

/* ---------------------------------------------------------------------------
 * Write
 * ------------------------------------------------------------------------- */

bsp_i2c_status_t BSP_I2C1_Write(uint8_t dev_addr7, const uint8_t *data, uint16_t len)
{
    if (!wait_flag(&I2C1->SR2, I2C_SR2_BUSY, 0)) {
        return BSP_I2C_ERR_BUS_BUSY;
    }

    I2C1->CR1 |= I2C_CR1_START;
    if (!wait_flag(&I2C1->SR1, I2C_SR1_SB, I2C_SR1_SB)) {
        return BSP_I2C_ERR_TIMEOUT;
    }

    I2C1->DR = (uint32_t)(dev_addr7 << 1) | 0u; /* write direction */
    while (!(I2C1->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF))) {
        /* tunggu ADDR (ack) atau AF (nack) -- tanpa timeout counter
         * terpisah di sini karena AF akan set duluan kalau NACK */
    }
    if (check_and_handle_nack()) {
        return BSP_I2C_ERR_NACK;
    }
    (void)I2C1->SR2; /* clear ADDR dengan membaca SR2 setelah SR1 */

    for (uint16_t i = 0; i < len; i++) {
        if (!wait_flag(&I2C1->SR1, I2C_SR1_TXE, I2C_SR1_TXE)) {
            I2C1->CR1 |= I2C_CR1_STOP;
            return BSP_I2C_ERR_TIMEOUT;
        }
        I2C1->DR = data[i];
        if (I2C1->SR1 & I2C_SR1_AF) {
            check_and_handle_nack();
            return BSP_I2C_ERR_NACK;
        }
    }

    if (!wait_flag(&I2C1->SR1, I2C_SR1_BTF, I2C_SR1_BTF)) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return BSP_I2C_ERR_TIMEOUT;
    }

    I2C1->CR1 |= I2C_CR1_STOP;
    return BSP_I2C_OK;
}

bsp_i2c_status_t BSP_I2C1_WriteReg(uint8_t dev_addr7, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return BSP_I2C1_Write(dev_addr7, buf, 2u);
}

/* ---------------------------------------------------------------------------
 * Read (register pointer write, lalu repeated-START + baca N byte)
 * Mengikuti algoritma standar peripheral I2C klasik STM32F4 untuk kasus
 * N==1, N==2, dan N>=3 (masing-masing butuh urutan clear-ACK/STOP yang
 * berbeda supaya tidak kirim ACK ekstra di byte terakhir).
 * ------------------------------------------------------------------------- */

bsp_i2c_status_t BSP_I2C1_ReadRegs(uint8_t dev_addr7, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (len == 0u) {
        return BSP_I2C_OK;
    }

    if (!wait_flag(&I2C1->SR2, I2C_SR2_BUSY, 0)) {
        return BSP_I2C_ERR_BUS_BUSY;
    }

    I2C1->CR1 |= I2C_CR1_ACK; /* pastikan ACK enabled sebelum mulai */

    /* --- Fase 1: kirim alamat register (write direction) --- */
    I2C1->CR1 |= I2C_CR1_START;
    if (!wait_flag(&I2C1->SR1, I2C_SR1_SB, I2C_SR1_SB)) {
        return BSP_I2C_ERR_TIMEOUT;
    }
    I2C1->DR = (uint32_t)(dev_addr7 << 1) | 0u;
    while (!(I2C1->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF))) { }
    if (check_and_handle_nack()) {
        return BSP_I2C_ERR_NACK;
    }
    (void)I2C1->SR2;

    if (!wait_flag(&I2C1->SR1, I2C_SR1_TXE, I2C_SR1_TXE)) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return BSP_I2C_ERR_TIMEOUT;
    }
    I2C1->DR = reg;
    if (!wait_flag(&I2C1->SR1, I2C_SR1_BTF, I2C_SR1_BTF)) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return BSP_I2C_ERR_TIMEOUT;
    }

    /* --- Fase 2: repeated START, alamat lagi (read direction) --- */
    I2C1->CR1 |= I2C_CR1_START;
    if (!wait_flag(&I2C1->SR1, I2C_SR1_SB, I2C_SR1_SB)) {
        return BSP_I2C_ERR_TIMEOUT;
    }
    I2C1->DR = (uint32_t)(dev_addr7 << 1) | 1u;
    while (!(I2C1->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF))) { }
    if (check_and_handle_nack()) {
        return BSP_I2C_ERR_NACK;
    }

    if (len == 1u) {
        I2C1->CR1 &= ~I2C_CR1_ACK;
        (void)I2C1->SR2; /* clear ADDR */
        I2C1->CR1 |= I2C_CR1_STOP;

        if (!wait_flag(&I2C1->SR1, I2C_SR1_RXNE, I2C_SR1_RXNE)) {
            return BSP_I2C_ERR_TIMEOUT;
        }
        data[0] = (uint8_t)I2C1->DR;
    } else if (len == 2u) {
        I2C1->CR1 |= I2C_CR1_POS;
        I2C1->CR1 &= ~I2C_CR1_ACK;
        (void)I2C1->SR2; /* clear ADDR */

        if (!wait_flag(&I2C1->SR1, I2C_SR1_BTF, I2C_SR1_BTF)) {
            return BSP_I2C_ERR_TIMEOUT;
        }
        I2C1->CR1 |= I2C_CR1_STOP;
        data[0] = (uint8_t)I2C1->DR;
        data[1] = (uint8_t)I2C1->DR;
        I2C1->CR1 &= ~I2C_CR1_POS; /* reset POS untuk transaksi berikutnya */
    } else {
        (void)I2C1->SR2; /* clear ADDR, ACK masih enabled untuk byte awal */

        for (uint16_t i = 0; i < (uint16_t)(len - 2u); i++) {
            if (!wait_flag(&I2C1->SR1, I2C_SR1_RXNE, I2C_SR1_RXNE)) {
                return BSP_I2C_ERR_TIMEOUT;
            }
            data[i] = (uint8_t)I2C1->DR;
        }

        if (!wait_flag(&I2C1->SR1, I2C_SR1_BTF, I2C_SR1_BTF)) {
            return BSP_I2C_ERR_TIMEOUT;
        }
        I2C1->CR1 &= ~I2C_CR1_ACK;
        data[len - 2u] = (uint8_t)I2C1->DR;
        I2C1->CR1 |= I2C_CR1_STOP;

        if (!wait_flag(&I2C1->SR1, I2C_SR1_RXNE, I2C_SR1_RXNE)) {
            return BSP_I2C_ERR_TIMEOUT;
        }
        data[len - 1u] = (uint8_t)I2C1->DR;
    }

    I2C1->CR1 |= I2C_CR1_ACK; /* restore ACK default untuk transaksi berikutnya */
    return BSP_I2C_OK;
}
