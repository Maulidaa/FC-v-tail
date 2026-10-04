#include "bsp_spi.h"
#include "bsp_pinmap.h"

/* ============================================================================
 * bsp_spi.c
 * Implementasi bare-metal (register langsung, tanpa HAL) untuk SPI1 (IMU
 * MPU6500) dan SPI2 (flash W25Q64 blackbox).
 *
 * CR1.BR[2:0] prescaler encoding (sama untuk SPI1 & SPI2):
 *   000 -> /2    001 -> /4    010 -> /8    011 -> /16
 *   100 -> /32   101 -> /64   110 -> /128  111 -> /256
 * ============================================================================ */

#define SPI_BR_DIV2    (0x0u << 3)
#define SPI_BR_DIV8    (0x2u << 3)
#define SPI_BR_DIV256  (0x7u << 3)

/* ---------------------------------------------------------------------------
 * Helper generik: konfigurasi SPIx sebagai master, 8-bit, software NSS,
 * mode CPOL/CPHA tertentu, prescaler tertentu. Tidak menyentuh GPIO --
 * itu tanggung jawab bsp_pinmap.c.
 * ------------------------------------------------------------------------- */
static void spi_configure(SPI_TypeDef *spi, uint32_t br_bits, bsp_spi_mode_t mode)
{
    /* Disable dulu sebelum reconfig (aman dipanggil ulang mis. ganti speed) */
    spi->CR1 &= ~SPI_CR1_SPE;

    uint32_t cr1 = 0;
    cr1 |= SPI_CR1_MSTR;      /* mode master */
    cr1 |= SPI_CR1_SSM | SPI_CR1_SSI; /* software NSS, internal slave select = 1 (idle) */
    cr1 |= br_bits;           /* prescaler baud rate */

    if (mode == BSP_SPI_MODE3) {
        cr1 |= SPI_CR1_CPOL | SPI_CR1_CPHA;
    }
    /* mode0 (CPOL=0, CPHA=0): bit tetap 0, tidak perlu di-set */

    spi->CR1 = cr1;
    spi->CR2 = 0; /* tidak pakai DMA/IT untuk transfer register biasa */

    spi->CR1 |= SPI_CR1_SPE; /* enable peripheral */
}

static uint8_t spi_transfer_byte(SPI_TypeDef *spi, uint8_t data)
{
    while (!(spi->SR & SPI_SR_TXE)) {
        /* tunggu TX buffer kosong */
    }
    *(volatile uint8_t *)&spi->DR = data;

    while (!(spi->SR & SPI_SR_RXNE)) {
        /* tunggu data masuk RX buffer */
    }
    return (uint8_t)spi->DR;
}

static void spi_wait_not_busy(SPI_TypeDef *spi)
{
    /* Wajib dicek sebelum CS dinaikkan lagi, supaya transaksi benar2 selesai
     * di level bus (bukan cuma shift register kosong). */
    while (spi->SR & SPI_SR_BSY) {
        /* busy-wait */
    }
}

/* ---------------------------------------------------------------------------
 * SPI1 - IMU MPU6500
 * ------------------------------------------------------------------------- */

void BSP_SPI1_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    __asm volatile ("nop");
    __asm volatile ("nop");

    /* CS idle HIGH sebelum peripheral dinyalakan */
    BSP_SPI1_CS_High();

    /* Kecepatan aman untuk power-up/config register MPU6500: APB2/256 */
    spi_configure(SPI1, SPI_BR_DIV256, BSP_SPI_MODE0);
}

void BSP_SPI1_SetFastMode(void)
{
    /* Kecepatan operasi normal baca data burst: APB2/8 = 12 MHz */
    spi_configure(SPI1, SPI_BR_DIV8, BSP_SPI_MODE0);
}

void BSP_SPI1_CS_Low(void)
{
    PIN_IMU_CS_PORT->BSRR = (1u << (PIN_IMU_CS_PIN + 16u)); /* reset -> LOW */
}

void BSP_SPI1_CS_High(void)
{
    spi_wait_not_busy(SPI1);
    PIN_IMU_CS_PORT->BSRR = (1u << PIN_IMU_CS_PIN); /* set -> HIGH */
}

uint8_t BSP_SPI1_TransferByte(uint8_t data)
{
    return spi_transfer_byte(SPI1, data);
}

void BSP_SPI1_WaitNotBusy(void)
{
    spi_wait_not_busy(SPI1);
}

/* ---------------------------------------------------------------------------
 * SPI2 - Flash W25Q64
 * ------------------------------------------------------------------------- */

void BSP_SPI2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    __asm volatile ("nop");
    __asm volatile ("nop");

    /* CS idle HIGH sebelum peripheral dinyalakan */
    BSP_SPI2_CS_High();

    /* W25Q64 tidak butuh slow-init; langsung APB1/2 = 24 MHz, mode0 */
    spi_configure(SPI2, SPI_BR_DIV2, BSP_SPI_MODE0);
}

void BSP_SPI2_CS_Low(void)
{
    PIN_FLASH_CS_PORT->BSRR = (1u << (PIN_FLASH_CS_PIN + 16u)); /* reset -> LOW */
}

void BSP_SPI2_CS_High(void)
{
    spi_wait_not_busy(SPI2);
    PIN_FLASH_CS_PORT->BSRR = (1u << PIN_FLASH_CS_PIN); /* set -> HIGH */
}

uint8_t BSP_SPI2_TransferByte(uint8_t data)
{
    return spi_transfer_byte(SPI2, data);
}
