#ifndef BSP_SPI_H
#define BSP_SPI_H

#include "stm32f4xx.h"   /* CMSIS device header, target STM32F411xE */
#include <stdint.h>

/* ============================================================================
 * bsp_spi.h
 * SPI1 -> IMU MPU6500 (PA4=CS manual, PA5=SCK, PA6=MISO, PA7=MOSI), AF5
 * SPI2 -> Flash W25Q64 blackbox (PB12=CS manual, PB13=SCK, PB14=MISO,
 *         PB15=MOSI), AF5
 *
 * GPIO mode/AF untuk pin-pin di atas SUDAH dikonfigurasi oleh
 * BSP_PinMap_Init() (lihat bsp_pinmap.c) -- modul ini HANYA menyentuh
 * register peripheral SPI1/SPI2 + toggle pin CS (yang sudah GPIO output
 * sejak BSP_PinMap_Init).
 *
 * Clock tree acuan (final, lihat firmware-architecture-stm32f411.md /
 * pinout-fc-stm32f411.md Bagian F):
 *   CORE_CLOCK_HZ = 96 MHz
 *   APB2 (SPI1)   = 96 MHz  (prescaler /1)
 *   APB1 (SPI2)   = 48 MHz  (prescaler /2)
 *
 * MPU6500: datasheet mewajibkan SPI <=1MHz selama sequence power-up /
 * register config awal, baru boleh naik ke <=20MHz untuk baca data
 * burst rutin. Modul ini expose dua mode kecepatan eksplisit supaya
 * driver imu_mpu6500.c (Orang 2) yang menentukan kapan pindah mode,
 * bukan diasumsikan di sini.
 * ============================================================================ */

#define APB2_CLOCK_HZ   96000000UL
#define APB1_CLOCK_HZ   48000000UL   /* == APB1_CLOCK_MHZ 48, dipakai juga oleh bsp_i2c.c */

typedef enum {
    BSP_SPI_MODE0 = 0, /* CPOL=0, CPHA=0 */
    BSP_SPI_MODE3 = 3  /* CPOL=1, CPHA=1 -- MPU6500 mendukung mode0 & mode3 */
} bsp_spi_mode_t;

/* ---------------------------------------------------------------------------
 * SPI1 - IMU MPU6500
 * ------------------------------------------------------------------------- */

/**
 * @brief Init SPI1 di kecepatan AMAN untuk power-up/konfigurasi register
 *        MPU6500 (APB2/256 = 96MHz/256 = 375 kHz, di bawah batas 1MHz
 *        datasheet). Mode SPI: mode0 (CPOL=0, CPHA=0).
 *        Panggil BSP_SPI1_SetFastMode() setelah sensor selesai
 *        diinisialisasi supaya baca data burst rutin lebih cepat.
 *        CS (PA4) dipastikan idle HIGH (assumsi sudah HIGH dari
 *        BSP_PinMap_Init, tapi dipaksa ulang di sini untuk jaga-jaga).
 */
void BSP_SPI1_Init(void);

/**
 * @brief Pindah SPI1 ke kecepatan operasi normal setelah init selesai
 *        (APB2/8 = 96MHz/8 = 12 MHz -- aman di bawah batas MPU6500 20MHz,
 *        dengan margin untuk kualitas sinyal jalur SPI di board nyata).
 */
void BSP_SPI1_SetFastMode(void);

/** @brief CS (PA4) LOW -- mulai transaksi SPI1. */
void BSP_SPI1_CS_Low(void);

/** @brief CS (PA4) HIGH -- akhiri transaksi SPI1. */
void BSP_SPI1_CS_High(void);

/**
 * @brief Transfer 1 byte full-duplex blocking di SPI1.
 * @param data byte yang dikirim (MOSI)
 * @return byte yang diterima (MISO)
 */
uint8_t BSP_SPI1_TransferByte(uint8_t data);

/**
 * @brief Tunggu sampai bus SPI1 benar-benar selesai transaksi (BSY flag clear).
 *        Panggil ini sebelum menaikkan CS secara manual (kalau tidak lewat
 *        BSP_SPI1_CS_High()), supaya transaksi tidak terpotong di level bus.
 */
void BSP_SPI1_WaitNotBusy(void);

/* ---------------------------------------------------------------------------
 * SPI2 - Flash W25Q64 (blackbox)
 * ------------------------------------------------------------------------- */

/**
 * @brief Init SPI2 untuk flash W25Q64. W25Q64 tidak punya syarat "slow
 *        init" seperti MPU6500 (spec standard SPI read/write commands
 *        sampai puluhan MHz sejak power-up), jadi langsung dikonfigurasi
 *        di kecepatan operasi (APB1/2 = 48MHz/2 = 24 MHz), mode0.
 *        CS (PB12) dipastikan idle HIGH.
 */
void BSP_SPI2_Init(void);

/** @brief CS (PB12) LOW -- mulai transaksi SPI2. */
void BSP_SPI2_CS_Low(void);

/** @brief CS (PB12) HIGH -- akhiri transaksi SPI2. */
void BSP_SPI2_CS_High(void);

/**
 * @brief Transfer 1 byte full-duplex blocking di SPI2.
 * @param data byte yang dikirim (MOSI)
 * @return byte yang diterima (MISO)
 */
uint8_t BSP_SPI2_TransferByte(uint8_t data);

#endif /* BSP_SPI_H */
