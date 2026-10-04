#ifndef BSP_PINMAP_H
#define BSP_PINMAP_H

#include "stm32f4xx.h"   /* CMSIS device header, target STM32F411xE */
#include <stdint.h>

/* ============================================================================
 * bsp_pinmap.h
 * Satu-satunya sumber kebenaran DI KODE untuk alokasi pin fisik.
 * Sinkron dengan dokumen: pinout-fc-stm32f411.md (sumber kebenaran dokumen).
 *
 * Status per pin mengikuti tabel di dokumen tsb:
 *   [SUDAH ADA DI KODE]  -> driver terkait sudah mengasumsikan pin ini
 *   [BARU]                -> dialokasikan final di sini, driver menyusul
 *   spare                 -> belum dipakai untuk fungsi apa pun
 *
 * CLOSING ITEM (Orang 1):
 *   PA9 sebelumnya dicadangkan untuk USART1_TX (telemetry balik --
 *   awalnya untuk CRSF, sekarang tidak relevan lagi karena RX diganti
 *   ke iBus, tapi kesimpulannya tetap sama).
 *   Keputusan tim: RX-only untuk v1 (iBus servo/channel frame memang
 *   satu arah) -> PA9 DIBEBASKAN jadi spare.
 *   Jangan inisialisasi PA9 sebagai USART1_TX di bsp_uart tanpa revisi
 *   keputusan ini dan update dokumen.
 * ============================================================================ */

/* ---------------------------------------------------------------------------
 * Tipe konfigurasi generik (bare-metal, tanpa HAL)
 * ------------------------------------------------------------------------- */
typedef enum {
    PINMAP_MODE_INPUT  = 0x0,
    PINMAP_MODE_OUTPUT = 0x1,
    PINMAP_MODE_AF     = 0x2,
    PINMAP_MODE_ANALOG = 0x3
} pinmap_mode_t;

typedef enum {
    PINMAP_OTYPE_PP = 0,  /* push-pull */
    PINMAP_OTYPE_OD = 1   /* open-drain */
} pinmap_otype_t;

typedef enum {
    PINMAP_SPEED_LOW    = 0,
    PINMAP_SPEED_MEDIUM = 1,
    PINMAP_SPEED_HIGH   = 2,
    PINMAP_SPEED_VHIGH  = 3
} pinmap_speed_t;

typedef enum {
    PINMAP_PUPD_NONE = 0,
    PINMAP_PUPD_UP   = 1,
    PINMAP_PUPD_DOWN = 2
} pinmap_pupd_t;

/* Nomor Alternate Function (AFRL/AFRH), sesuai datasheet STM32F411 */
#define AF0_SYS       0
#define AF1_TIM1      1
#define AF2_TIM3_4    2
#define AF4_I2C1      4
#define AF5_SPI1_2    5
#define AF7_USART1_2  7
#define AF8_USART6    8
#define AF10_OTG_FS   10

/* ---------------------------------------------------------------------------
 * ADC1 - Power monitoring (Orang 4, dipakai power_monitor.c)
 * ------------------------------------------------------------------------- */
#define PIN_VBAT_SENSE_PORT      GPIOA
#define PIN_VBAT_SENSE_PIN       0u    /* ADC1_IN0 */
#define PIN_CURRENT_SENSE_PORT   GPIOA
#define PIN_CURRENT_SENSE_PIN    1u    /* ADC1_IN1 */

/* ---------------------------------------------------------------------------
 * USART2 - GPS  [SUDAH ADA DI KODE]
 * ------------------------------------------------------------------------- */
#define PIN_GPS_TX_PORT          GPIOA
#define PIN_GPS_TX_PIN           2u    /* USART2_TX, AF7 */
#define PIN_GPS_RX_PORT          GPIOA
#define PIN_GPS_RX_PIN           3u    /* USART2_RX, AF7 */

/* ---------------------------------------------------------------------------
 * SPI1 - IMU MPU6500  [SUDAH ADA DI KODE]
 * ------------------------------------------------------------------------- */
#define PIN_IMU_CS_PORT          GPIOA
#define PIN_IMU_CS_PIN           4u    /* GPIO output manual, bukan SPI1_NSS hw */
#define PIN_SPI1_SCK_PORT        GPIOA
#define PIN_SPI1_SCK_PIN         5u    /* AF5 */
#define PIN_SPI1_MISO_PORT       GPIOA
#define PIN_SPI1_MISO_PIN        6u    /* AF5 */
#define PIN_SPI1_MOSI_PORT       GPIOA
#define PIN_SPI1_MOSI_PIN        7u    /* AF5 */

/* ---------------------------------------------------------------------------
 * TIM1 - MOTOR0 DShot  [BARU]
 * DMA: DMA2 Stream1 / Channel6 (lihat catatan Bagian G checklist)
 * ------------------------------------------------------------------------- */
#define PIN_MOTOR0_PORT          GPIOA
#define PIN_MOTOR0_PIN           8u    /* TIM1_CH1, AF1 */

/* ---------------------------------------------------------------------------
 * USART1 - iBus  [BARU]  --  RX-only (keputusan tim, lihat header file ini)
 * ------------------------------------------------------------------------- */
#define PIN_IBUS_RX_PORT         GPIOA
#define PIN_IBUS_RX_PIN          10u   /* USART1_RX, AF7 */

/* PA9: eks USART1_TX, sekarang SPARE. Jangan konfigurasi AF7 di sini. */
#define PIN_SPARE_PA9_PORT       GPIOA
#define PIN_SPARE_PA9_PIN        9u

/* ---------------------------------------------------------------------------
 * USART6 - Command/Config link cadangan lewat USB-to-TTL  [BARU]
 * Dipakai kalau device disambung via adapter USB-to-TTL ke pin UART
 * (bukan lewat konektor USB native STM32) -- protocol.c mem-poll USART6
 * ini SEKALIGUS USB CDC, request_id dibalas ke jalur asal masing2
 * (lihat comms/protocol.c). PC6/PC7 sebelumnya belum dipakai fungsi apa
 * pun, jadi tidak bentrok dengan USART1 (iBus)/USART2 (GPS)/USB.
 * ------------------------------------------------------------------------- */
#define PIN_CMD_UART_TX_PORT     GPIOC
#define PIN_CMD_UART_TX_PIN      6u    /* USART6_TX, AF8 */
#define PIN_CMD_UART_RX_PORT     GPIOC
#define PIN_CMD_UART_RX_PIN      7u    /* USART6_RX, AF8 */

/* ---------------------------------------------------------------------------
 * USB OTG FS - fixed silicon, jangan dipindah
 * ------------------------------------------------------------------------- */
#define PIN_USB_DM_PORT          GPIOA
#define PIN_USB_DM_PIN           11u   /* AF10 */
#define PIN_USB_DP_PORT          GPIOA
#define PIN_USB_DP_PIN           12u   /* AF10 */

/* ---------------------------------------------------------------------------
 * Debug SWD - dicadangkan, JANGAN REALOKASI, JANGAN disentuh init GPIO apa pun
 * ------------------------------------------------------------------------- */
#define PIN_SWDIO_PORT           GPIOA
#define PIN_SWDIO_PIN            13u
#define PIN_SWCLK_PORT           GPIOA
#define PIN_SWCLK_PIN            14u

/* ---------------------------------------------------------------------------
 * Spare - eks jalur JTAG penuh (JTDI/JTDO/NJTRST)
 * Default reset: AF0 (fungsi debug). WAJIB panggil
 * BSP_PinMap_ReleaseJTAGPin() sebelum salah satu pin ini dipakai
 * sebagai GPIO/timer biasa. Saat ini ketiganya belum dipakai fungsi lain.
 * ------------------------------------------------------------------------- */
#define PIN_SPARE_PA15_PORT      GPIOA
#define PIN_SPARE_PA15_PIN       15u   /* eks JTDI */
#define PIN_SPARE_PB3_PORT       GPIOB
#define PIN_SPARE_PB3_PIN        3u    /* eks JTDO/SWO */
#define PIN_SPARE_PB4_PORT       GPIOB
#define PIN_SPARE_PB4_PIN        4u    /* eks NJTRST */

/* ---------------------------------------------------------------------------
 * TIM3 - SERVO0/1 Aileron  [BARU]
 * ------------------------------------------------------------------------- */
#define PIN_SERVO0_AIL_L_PORT    GPIOB
#define PIN_SERVO0_AIL_L_PIN     0u    /* TIM3_CH3, AF2 */
#define PIN_SERVO1_AIL_R_PORT    GPIOB
#define PIN_SERVO1_AIL_R_PIN     1u    /* TIM3_CH4, AF2 */

/* ---------------------------------------------------------------------------
 * Spare - BOOT1 (aman dipakai GPIO biasa selama BOOT0 low / boot normal)
 * ------------------------------------------------------------------------- */
#define PIN_SPARE_PB2_PORT       GPIOB
#define PIN_SPARE_PB2_PIN        2u
#define PIN_SPARE_PB5_PORT       GPIOB
#define PIN_SPARE_PB5_PIN        5u

/* ---------------------------------------------------------------------------
 * I2C1 - 4 device: MPU6050, BMP280, HMC5883/QMC5883, OLED SSD1306
 * [SUDAH ADA DI KODE]
 * ------------------------------------------------------------------------- */
#define PIN_I2C1_SCL_PORT        GPIOB
#define PIN_I2C1_SCL_PIN         6u    /* AF4 */
#define PIN_I2C1_SDA_PORT        GPIOB
#define PIN_I2C1_SDA_PIN         7u    /* AF4 */

/* ---------------------------------------------------------------------------
 * TIM4 - SERVO2/3 Ruddervator  [BARU]
 * ------------------------------------------------------------------------- */
#define PIN_SERVO2_VTAIL_L_PORT  GPIOB
#define PIN_SERVO2_VTAIL_L_PIN   8u    /* TIM4_CH3, AF2 */
#define PIN_SERVO3_VTAIL_R_PORT  GPIOB
#define PIN_SERVO3_VTAIL_R_PIN   9u    /* TIM4_CH4, AF2 */

/* ---------------------------------------------------------------------------
 * Spare - slot ekspansi TIM2 (motor kedua / RSSI capture, dll)
 * ------------------------------------------------------------------------- */
#define PIN_SPARE_PB10_PORT      GPIOB
#define PIN_SPARE_PB10_PIN       10u   /* bisa jadi TIM2_CH3 */
#define PIN_SPARE_PB11_PORT      GPIOB
#define PIN_SPARE_PB11_PIN       11u   /* bisa jadi TIM2_CH4 */

/* ---------------------------------------------------------------------------
 * SPI2 - Flash W25Q64 (Blackbox)  [BARU]
 * ------------------------------------------------------------------------- */
#define PIN_FLASH_CS_PORT        GPIOB
#define PIN_FLASH_CS_PIN         12u   /* GPIO output manual */
#define PIN_SPI2_SCK_PORT        GPIOB
#define PIN_SPI2_SCK_PIN         13u   /* AF5 */
#define PIN_SPI2_MISO_PORT       GPIOB
#define PIN_SPI2_MISO_PIN        14u   /* AF5 */
#define PIN_SPI2_MOSI_PORT       GPIOB
#define PIN_SPI2_MOSI_PIN        15u   /* AF5 */

/* ---------------------------------------------------------------------------
 * PC13 - LED status onboard (WeAct Blackpill: LED biru, active-low)
 * ------------------------------------------------------------------------- */
#define PIN_LED_STATUS_PORT      GPIOC
#define PIN_LED_STATUS_PIN       13u
#define LED_STATUS_ACTIVE_LOW    1

/* ---------------------------------------------------------------------------
 * PC14/PC15 - OSC32_IN/OUT untuk LSE 32.768kHz.
 * Board WeAct Blackpill CEU6: LSE terpasang -> JANGAN dikonfigurasi sebagai
 * GPIO oleh BSP_PinMap_Init(). Dicadangkan untuk RCC (LSE) / RTC.
 * ------------------------------------------------------------------------- */
#define PIN_OSC32_IN_PORT        GPIOC
#define PIN_OSC32_IN_PIN         14u
#define PIN_OSC32_OUT_PORT       GPIOC
#define PIN_OSC32_OUT_PIN        15u

/* ---------------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------------- */

/**
 * @brief Enable clock AHB1 untuk GPIOA/B/C. Panggil paling awal di boot,
 *        sebelum modul BSP lain (spi/i2c/uart/adc/timer) melakukan init pin.
 */
void BSP_PinMap_EnableGPIOClocks(void);

/**
 * @brief Konfigurasi satu pin secara generik (bare-metal, tanpa HAL).
 *        Dipakai oleh bsp_pinmap.c sendiri maupun modul BSP lain yang perlu
 *        konfigurasi pin ad-hoc (mis. GPIO chip-select SPI).
 */
void BSP_GPIO_ConfigPin(GPIO_TypeDef *port, uint8_t pin,
                         pinmap_mode_t mode, uint8_t af,
                         pinmap_otype_t otype, pinmap_speed_t speed,
                         pinmap_pupd_t pupd);

/**
 * @brief Inisialisasi seluruh pin yang statusnya SUDAH ADA/BARU (bukan spare)
 *        sesuai tabel di pinout-fc-stm32f411.md. Pin spare & pin debug SWD
 *        TIDAK disentuh oleh fungsi ini.
 *
 *        Catatan urutan: fungsi ini TIDAK menginisialisasi peripheral clock
 *        (RCC APB/AHB untuk SPI/I2C/USART/TIM) - itu tanggung jawab masing2
 *        modul BSP (bsp_spi, bsp_i2c, dst). Fungsi ini hanya menyiapkan
 *        GPIO mode/AF/speed/pupd.
 */
void BSP_PinMap_Init(void);

/**
 * @brief Bebaskan salah satu pin eks-JTAG (PA15 / PB3 / PB4) dari fungsi
 *        debug JTAG bawaan, supaya bisa dipakai GPIO/AF biasa. WAJIB
 *        dipanggil sebelum pin terkait dikonfigurasi untuk fungsi lain.
 *
 *        Fungsi ini HANYA melepas AF0 (set input floating sementara) -
 *        caller tetap harus memanggil BSP_GPIO_ConfigPin() sesudahnya untuk
 *        konfigurasi mode/AF final sesuai kebutuhan drivernya.
 *
 *        Pin PA13 (SWDIO) & PA14 (SWCLK) TIDAK BISA dilepas lewat fungsi
 *        ini secara sengaja - keduanya harus tetap SWD aktif.
 *
 * @return 0 jika berhasil, -1 jika pin yang diminta bukan PA15/PB3/PB4.
 */
int BSP_PinMap_ReleaseJTAGPin(GPIO_TypeDef *port, uint8_t pin);

#endif /* BSP_PINMAP_H */
