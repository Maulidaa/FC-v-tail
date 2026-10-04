#ifndef BSP_ADC_H
#define BSP_ADC_H

#include "stm32f4xx.h"   /* CMSIS device header, target STM32F411xE */
#include <stdint.h>

/* ============================================================================
 * bsp_adc.h
 * ADC1 -> VBAT sense (PA0, ADC1_IN0) + Current sense (PA1, ADC1_IN1).
 * GPIO mode analog untuk kedua pin SUDAH dikonfigurasi oleh BSP_PinMap_Init()
 * -- modul ini hanya menyentuh register ADC1 + DMA2 Stream0.
 *
 * DESAIN: scan mode + continuous conversion + DMA circular, JALAN TERUS
 * di background sejak BSP_ADC_Init() dipanggil. Konsumen (power_monitor.c,
 * Orang 4) tinggal panggil getter kapan saja tanpa memicu konversi manual
 * -- nilai selalu "recent enough" (siklus scan 2 channel jauh lebih cepat
 * dari kebutuhan update rate power monitoring).
 *
 * DMA mapping (final, closing item checklist Bagian G):
 *   ADC1 -> DMA2 Stream0 / Channel0 (tidak bentrok dengan TIM1_CH1/DShot
 *   yang dipetakan ke DMA2 Stream1 / Channel6 -- lihat bsp_timer_pwm.h).
 *
 * Clock ADC: ADCCLK = APB2_CLOCK_HZ / 4 = 96MHz / 4 = 24MHz (di bawah batas
 * 36MHz datasheet STM32F411, dengan margin aman).
 *
 * PEMBAGIAN TANGGUNG JAWAB: modul ini HANYA mengurus akuisisi raw ADC count
 * (12-bit, 0-4095) dan konversi ke tegangan DI PIN MCU (asumsi Vref 3.3V).
 * Rasio voltage-divider VBAT dan sensitivitas sensor arus adalah detail
 * hardware/kalibrasi milik power_monitor.c (Orang 4) -- BUKAN di-hardcode
 * di sini, supaya bsp_adc tetap reusable kalau nilai resistor divider
 * berubah tanpa harus menyentuh BSP.
 * ============================================================================ */

#define ADC_VREF_VOLTS   3.3f
#define ADC_RESOLUTION   4095u   /* 12-bit */

/**
 * @brief Init ADC1 (scan, continuous, DMA circular) untuk channel VBAT
 *        (IN0) dan current sense (IN1), lalu mulai konversi (SWSTART).
 *        Setelah dipanggil, ADC1 berjalan terus di background -- caller
 *        tidak perlu memicu konversi lagi.
 *        Panggil sekali saat boot, setelah BSP_PinMap_Init().
 */
void BSP_ADC_Init(void);

/** @brief Raw ADC count (0-4095) hasil konversi VBAT (PA0) terbaru. */
uint16_t BSP_ADC_GetVBatRaw(void);

/** @brief Raw ADC count (0-4095) hasil konversi current sense (PA1) terbaru. */
uint16_t BSP_ADC_GetCurrentRaw(void);

/**
 * @brief Konversi raw ADC count ke tegangan DI PIN MCU (belum dibagi rasio
 *        voltage-divider). Rumus: raw * ADC_VREF_VOLTS / ADC_RESOLUTION.
 *        power_monitor.c yang mengalikan hasil ini dengan rasio divider
 *        VBAT atau sensitivitas sensor arus (mV/A) sesuai skematik.
 * @param raw nilai dari BSP_ADC_GetVBatRaw() / BSP_ADC_GetCurrentRaw()
 * @return tegangan dalam volt di pin ADC
 */
float BSP_ADC_RawToPinVoltage(uint16_t raw);

#endif /* BSP_ADC_H */
