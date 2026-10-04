#include "bsp_pinmap.h"

/* ============================================================================
 * bsp_pinmap.c
 * Implementasi bare-metal (register langsung, tanpa HAL) untuk inisialisasi
 * GPIO sesuai bsp_pinmap.h / pinout-fc-stm32f411.md.
 * ============================================================================ */

/* Tabel deskriptor pin - hanya untuk pin yang statusnya SUDAH ADA/BARU
 * (bukan spare, bukan debug SWD, bukan OSC32). Pin spare sengaja tidak
 * dikonfigurasi di sini supaya tidak "mengunci" mode tertentu sebelum
 * modul pemiliknya jelas. */
typedef struct {
    GPIO_TypeDef  *port;
    uint8_t        pin;
    pinmap_mode_t  mode;
    uint8_t        af;
    pinmap_otype_t otype;
    pinmap_speed_t speed;
    pinmap_pupd_t  pupd;
} pinmap_entry_t;

static const pinmap_entry_t s_pinmap_table[] = {
    /* ADC1 - power monitoring */
    { PIN_VBAT_SENSE_PORT,    PIN_VBAT_SENSE_PIN,    PINMAP_MODE_ANALOG, 0,           PINMAP_OTYPE_PP, PINMAP_SPEED_LOW,    PINMAP_PUPD_NONE },
    { PIN_CURRENT_SENSE_PORT, PIN_CURRENT_SENSE_PIN, PINMAP_MODE_ANALOG, 0,           PINMAP_OTYPE_PP, PINMAP_SPEED_LOW,    PINMAP_PUPD_NONE },

    /* USART2 - GPS */
    { PIN_GPS_TX_PORT,        PIN_GPS_TX_PIN,        PINMAP_MODE_AF,     AF7_USART1_2, PINMAP_OTYPE_PP, PINMAP_SPEED_HIGH,  PINMAP_PUPD_UP },
    { PIN_GPS_RX_PORT,        PIN_GPS_RX_PIN,        PINMAP_MODE_AF,     AF7_USART1_2, PINMAP_OTYPE_PP, PINMAP_SPEED_HIGH,  PINMAP_PUPD_UP },

    /* SPI1 - IMU MPU6500 */
    { PIN_IMU_CS_PORT,        PIN_IMU_CS_PIN,        PINMAP_MODE_OUTPUT, 0,           PINMAP_OTYPE_PP, PINMAP_SPEED_VHIGH, PINMAP_PUPD_UP },
    { PIN_SPI1_SCK_PORT,      PIN_SPI1_SCK_PIN,      PINMAP_MODE_AF,     AF5_SPI1_2,  PINMAP_OTYPE_PP, PINMAP_SPEED_VHIGH, PINMAP_PUPD_NONE },
    { PIN_SPI1_MISO_PORT,     PIN_SPI1_MISO_PIN,     PINMAP_MODE_AF,     AF5_SPI1_2,  PINMAP_OTYPE_PP, PINMAP_SPEED_VHIGH, PINMAP_PUPD_NONE },
    { PIN_SPI1_MOSI_PORT,     PIN_SPI1_MOSI_PIN,     PINMAP_MODE_AF,     AF5_SPI1_2,  PINMAP_OTYPE_PP, PINMAP_SPEED_VHIGH, PINMAP_PUPD_NONE },

    /* TIM1 - MOTOR0 DShot */
    { PIN_MOTOR0_PORT,        PIN_MOTOR0_PIN,        PINMAP_MODE_AF,     AF1_TIM1,    PINMAP_OTYPE_PP, PINMAP_SPEED_VHIGH, PINMAP_PUPD_NONE },

    /* USART1 - iBus RX-only (PA9/TX sengaja tidak dikonfigurasi, lihat header) */
    { PIN_IBUS_RX_PORT,       PIN_IBUS_RX_PIN,       PINMAP_MODE_AF,     AF7_USART1_2, PINMAP_OTYPE_PP, PINMAP_SPEED_HIGH, PINMAP_PUPD_UP },

    /* USART6 - Command/Config link cadangan (USB-to-TTL) */
    { PIN_CMD_UART_TX_PORT,   PIN_CMD_UART_TX_PIN,   PINMAP_MODE_AF,     AF8_USART6,  PINMAP_OTYPE_PP, PINMAP_SPEED_HIGH,  PINMAP_PUPD_UP },
    { PIN_CMD_UART_RX_PORT,   PIN_CMD_UART_RX_PIN,   PINMAP_MODE_AF,     AF8_USART6,  PINMAP_OTYPE_PP, PINMAP_SPEED_HIGH,  PINMAP_PUPD_UP },

    /* USB OTG FS */
    { PIN_USB_DM_PORT,        PIN_USB_DM_PIN,        PINMAP_MODE_AF,     AF10_OTG_FS, PINMAP_OTYPE_PP, PINMAP_SPEED_VHIGH, PINMAP_PUPD_NONE },
    { PIN_USB_DP_PORT,        PIN_USB_DP_PIN,        PINMAP_MODE_AF,     AF10_OTG_FS, PINMAP_OTYPE_PP, PINMAP_SPEED_VHIGH, PINMAP_PUPD_NONE },

    /* TIM3 - Servo aileron */
    { PIN_SERVO0_AIL_L_PORT,  PIN_SERVO0_AIL_L_PIN,  PINMAP_MODE_AF,     AF2_TIM3_4,  PINMAP_OTYPE_PP, PINMAP_SPEED_HIGH,  PINMAP_PUPD_NONE },
    { PIN_SERVO1_AIL_R_PORT,  PIN_SERVO1_AIL_R_PIN,  PINMAP_MODE_AF,     AF2_TIM3_4,  PINMAP_OTYPE_PP, PINMAP_SPEED_HIGH,  PINMAP_PUPD_NONE },

    /* I2C1 - 4 device (open-drain wajib untuk I2C) */
    { PIN_I2C1_SCL_PORT,      PIN_I2C1_SCL_PIN,      PINMAP_MODE_AF,     AF4_I2C1,    PINMAP_OTYPE_OD, PINMAP_SPEED_HIGH,  PINMAP_PUPD_UP },
    { PIN_I2C1_SDA_PORT,      PIN_I2C1_SDA_PIN,      PINMAP_MODE_AF,     AF4_I2C1,    PINMAP_OTYPE_OD, PINMAP_SPEED_HIGH,  PINMAP_PUPD_UP },

    /* TIM4 - Servo ruddervator */
    { PIN_SERVO2_VTAIL_L_PORT, PIN_SERVO2_VTAIL_L_PIN, PINMAP_MODE_AF,   AF2_TIM3_4,  PINMAP_OTYPE_PP, PINMAP_SPEED_HIGH,  PINMAP_PUPD_NONE },
    { PIN_SERVO3_VTAIL_R_PORT, PIN_SERVO3_VTAIL_R_PIN, PINMAP_MODE_AF,   AF2_TIM3_4,  PINMAP_OTYPE_PP, PINMAP_SPEED_HIGH,  PINMAP_PUPD_NONE },

    /* SPI2 - Flash blackbox */
    { PIN_FLASH_CS_PORT,      PIN_FLASH_CS_PIN,      PINMAP_MODE_OUTPUT, 0,           PINMAP_OTYPE_PP, PINMAP_SPEED_VHIGH, PINMAP_PUPD_UP },
    { PIN_SPI2_SCK_PORT,      PIN_SPI2_SCK_PIN,      PINMAP_MODE_AF,     AF5_SPI1_2,  PINMAP_OTYPE_PP, PINMAP_SPEED_VHIGH, PINMAP_PUPD_NONE },
    { PIN_SPI2_MISO_PORT,     PIN_SPI2_MISO_PIN,     PINMAP_MODE_AF,     AF5_SPI1_2,  PINMAP_OTYPE_PP, PINMAP_SPEED_VHIGH, PINMAP_PUPD_NONE },
    { PIN_SPI2_MOSI_PORT,     PIN_SPI2_MOSI_PIN,     PINMAP_MODE_AF,     AF5_SPI1_2,  PINMAP_OTYPE_PP, PINMAP_SPEED_VHIGH, PINMAP_PUPD_NONE },

    /* PC13 - LED status */
    { PIN_LED_STATUS_PORT,    PIN_LED_STATUS_PIN,    PINMAP_MODE_OUTPUT, 0,           PINMAP_OTYPE_PP, PINMAP_SPEED_LOW,   PINMAP_PUPD_NONE },
};

#define PINMAP_TABLE_COUNT (sizeof(s_pinmap_table) / sizeof(s_pinmap_table[0]))

void BSP_PinMap_EnableGPIOClocks(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN
                  | RCC_AHB1ENR_GPIOBEN
                  | RCC_AHB1ENR_GPIOCEN;
    /* Delay singkat setelah enable clock peripheral (errata RCC) */
    __asm volatile ("nop");
    __asm volatile ("nop");
}

void BSP_GPIO_ConfigPin(GPIO_TypeDef *port, uint8_t pin,
                         pinmap_mode_t mode, uint8_t af,
                         pinmap_otype_t otype, pinmap_speed_t speed,
                         pinmap_pupd_t pupd)
{
    uint32_t pos2  = (uint32_t)pin * 2u;   /* untuk register 2-bit/pin */
    uint32_t pos4  = ((uint32_t)pin % 8u) * 4u; /* untuk AFR 4-bit/pin */

    /* MODER */
    port->MODER &= ~(0x3u << pos2);
    port->MODER |=  ((uint32_t)mode << pos2);

    /* OTYPER (hanya relevan utk output/AF) */
    if (mode == PINMAP_MODE_OUTPUT || mode == PINMAP_MODE_AF) {
        port->OTYPER &= ~(0x1u << pin);
        port->OTYPER |=  ((uint32_t)otype << pin);

        port->OSPEEDR &= ~(0x3u << pos2);
        port->OSPEEDR |=  ((uint32_t)speed << pos2);
    }

    /* PUPDR */
    port->PUPDR &= ~(0x3u << pos2);
    port->PUPDR |=  ((uint32_t)pupd << pos2);

    /* AFR[0] utk pin 0-7, AFR[1] utk pin 8-15 */
    if (mode == PINMAP_MODE_AF) {
        uint8_t afr_index = (pin < 8u) ? 0u : 1u;
        port->AFR[afr_index] &= ~(0xFu << pos4);
        port->AFR[afr_index] |=  ((uint32_t)af << pos4);
    }
}

void BSP_PinMap_Init(void)
{
    BSP_PinMap_EnableGPIOClocks();

    for (uint32_t i = 0; i < PINMAP_TABLE_COUNT; i++) {
        const pinmap_entry_t *e = &s_pinmap_table[i];
        BSP_GPIO_ConfigPin(e->port, e->pin, e->mode, e->af,
                            e->otype, e->speed, e->pupd);
    }

    /* LED status default: mati.
     * WeAct Blackpill LED aktif-low -> "mati" berarti pin HIGH. */
#if LED_STATUS_ACTIVE_LOW
    PIN_LED_STATUS_PORT->BSRR = (1u << PIN_LED_STATUS_PIN);
#else
    PIN_LED_STATUS_PORT->BSRR = (1u << (PIN_LED_STATUS_PIN + 16u));
#endif

    /* Chip-select default: idle HIGH (SPI CS aktif-low) */
    PIN_IMU_CS_PORT->BSRR   = (1u << PIN_IMU_CS_PIN);
    PIN_FLASH_CS_PORT->BSRR = (1u << PIN_FLASH_CS_PIN);

    /* Catatan sengaja TIDAK dikonfigurasi di sini:
     *  - PA9 (spare, eks USART1_TX)         -> RX-only, lihat bsp_pinmap.h
     *  - PA13/PA14 (SWDIO/SWCLK)             -> jangan diganggu, tetap debug
     *  - PA15/PB3/PB4 (eks JTAG)             -> panggil ReleaseJTAGPin dulu
     *  - PB2, PB5, PB10, PB11 (spare)        -> belum ada pemilik modul
     *  - PC14/PC15 (OSC32_IN/OUT untuk LSE)  -> dicadangkan RCC/RTC
     */
}

int BSP_PinMap_ReleaseJTAGPin(GPIO_TypeDef *port, uint8_t pin)
{
    int is_valid_jtag_pin =
        (port == GPIOA && pin == PIN_SPARE_PA15_PIN) ||
        (port == GPIOB && pin == PIN_SPARE_PB3_PIN)  ||
        (port == GPIOB && pin == PIN_SPARE_PB4_PIN);

    if (!is_valid_jtag_pin) {
        return -1;
    }

    /* Lepas dari AF0 (fungsi debug JTAG) -> input floating sementara.
     * Caller WAJIB memanggil BSP_GPIO_ConfigPin() sesudah ini untuk
     * konfigurasi mode final (output/AF lain) sesuai kebutuhan driver. */
    BSP_GPIO_ConfigPin(port, pin, PINMAP_MODE_INPUT, 0,
                        PINMAP_OTYPE_PP, PINMAP_SPEED_LOW, PINMAP_PUPD_NONE);

    return 0;
}
