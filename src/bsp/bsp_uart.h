#ifndef BSP_UART_H
#define BSP_UART_H

#include "stm32f4xx.h"   /* CMSIS device header, target STM32F411xE */
#include <stdint.h>

/* ============================================================================
 * bsp_uart.h
 * USART1 -> iBus RX-only (PA10=RX, AF7). PA9 (eks USART1_TX) SENGAJA tidak
 *           diinisialisasi di modul ini -- lihat bsp_pinmap.h, keputusan tim:
 *           RX-only untuk v1 (awalnya diputuskan untuk CRSF, tetap berlaku
 *           sama untuk iBus karena iBus servo/channel frame juga satu arah,
 *           tidak butuh jalur balik -- lihat rx_ibus.h untuk cakupan).
 * USART2 -> GPS (PA2=TX, PA3=RX, AF7), dua arah -- TX dipakai untuk kirim
 *           command konfigurasi UBX ke modul GPS, RX untuk terima NMEA/UBX.
 * USART6 -> Command/Config link cadangan (PC6=TX, PC7=RX, AF8), dua arah --
 *           dipakai comms/protocol.c sebagai transport tambahan selain USB
 *           CDC, untuk kasus device disambung lewat adapter USB-to-TTL ke
 *           pin UART, bukan lewat konektor USB native. Lihat bsp_pinmap.h.
 *
 * GPIO mode/AF pin-pin di atas SUDAH dikonfigurasi oleh BSP_PinMap_Init()
 * (lihat bsp_pinmap.c) -- modul ini HANYA menyentuh register peripheral
 * USART1/USART2 + NVIC.
 *
 * Clock tree acuan (final, lihat bsp_spi.h / firmware-architecture Bagian F):
 *   USART1 (APB2) = 96 MHz
 *   USART2 (APB1) = 48 MHz
 *
 * Desain RX: interrupt-driven ring buffer per USART (bukan DMA) -- cukup
 * untuk throughput iBus (115200bps, frame 32 byte tetap) dan GPS NMEA/UBX
 * (jauh lebih lambat). Kalau nanti scheduler (Bagian C checklist) sudah ada
 * dan beban CPU jadi masalah, DMA circular bisa jadi upgrade tanpa mengubah
 * API publik di header ini (BSP_UARTx_Available/ReadByte tetap sama).
 *
 * BAUD RATE:
 *   USART1_BAUD_IBUS -- 115200, baud tetap standar protokol iBus (bukan
 *   pilihan yang bisa-beda seperti CRSF dulu -- receiver FlySky/kompatibel
 *   iBus semuanya pakai baud ini, tidak perlu di-tuning per merk radio).
 *   USART2_BAUD_GPS -- default 9600 (baud default pabrik hampir semua
 *   modul GPS NMEA). Kalau modul sudah dikonfigurasi baud lain (mis.
 *   u-blox sering dinaikkan ke 57600/115200 untuk update rate tinggi),
 *   panggil BSP_UART2_Init() ulang dengan baud baru.
 * ============================================================================ */

#define USART1_BAUD_IBUS   115200u
#define USART2_BAUD_GPS    9600u
#define USART6_BAUD_CMD    115200u  /* samakan dengan DEFAULT_BAUD_RATE di faas-web/src/core/transport/constants.ts */

#define USART1_RX_BUF_SIZE 256u   /* harus power-of-2 untuk wrap mask murah */
#define USART2_RX_BUF_SIZE 256u
#define USART6_RX_BUF_SIZE 256u

/* ---------------------------------------------------------------------------
 * USART1 - iBus (RX-only)
 * ------------------------------------------------------------------------- */

/**
 * @brief Init USART1 RX-only di USART1_BAUD_IBUS, 8N1, RXNE interrupt
 *        aktif (ISR mengisi ring buffer internal). TX peripheral maupun
 *        pin TIDAK diaktifkan sama sekali (RX-only per keputusan tim).
 *        Panggil sekali saat boot, setelah BSP_PinMap_Init().
 */
void BSP_UART1_Init(void);

/** @brief Jumlah byte yang siap dibaca dari ring buffer RX USART1. */
uint16_t BSP_UART1_Available(void);

/**
 * @brief Ambil 1 byte dari ring buffer RX USART1 (non-blocking).
 * @param out tujuan byte yang diambil
 * @return 1 kalau ada byte diambil, 0 kalau buffer kosong
 */
int BSP_UART1_ReadByte(uint8_t *out);

/* ---------------------------------------------------------------------------
 * USART2 - GPS
 * ------------------------------------------------------------------------- */

/**
 * @brief Init USART2 (TX+RX) di baud yang diminta, 8N1, RXNE interrupt
 *        aktif. Bisa dipanggil ulang kapan saja untuk ganti baud rate
 *        (mis. setelah reconfigure modul GPS ke baud lebih tinggi) --
 *        ring buffer RX di-reset setiap kali fungsi ini dipanggil.
 * @param baud baud rate yang diinginkan, mis. USART2_BAUD_GPS
 */
void BSP_UART2_Init(uint32_t baud);

/** @brief Jumlah byte yang siap dibaca dari ring buffer RX USART2. */
uint16_t BSP_UART2_Available(void);

/**
 * @brief Ambil 1 byte dari ring buffer RX USART2 (non-blocking).
 * @param out tujuan byte yang diambil
 * @return 1 kalau ada byte diambil, 0 kalau buffer kosong
 */
int BSP_UART2_ReadByte(uint8_t *out);

/**
 * @brief Kirim 1 byte lewat USART2 (blocking sampai TXE set).
 *        Dipakai untuk kirim command konfigurasi UBX ke modul GPS.
 */
void BSP_UART2_WriteByte(uint8_t data);

/**
 * @brief Kirim N byte lewat USART2 (blocking).
 * @param data buffer yang dikirim
 * @param len jumlah byte
 */
void BSP_UART2_WriteBuf(const uint8_t *data, uint16_t len);

/* ---------------------------------------------------------------------------
 * USART6 - Command/Config link cadangan (TX+RX)
 * ------------------------------------------------------------------------- */

/**
 * @brief Init USART6 (TX+RX) di baud yang diminta, 8N1, RXNE interrupt
 *        aktif. Panggil sekali saat boot, setelah BSP_PinMap_Init().
 * @param baud baud rate yang diinginkan, mis. USART6_BAUD_CMD
 */
void BSP_UART6_Init(uint32_t baud);

/** @brief Jumlah byte yang siap dibaca dari ring buffer RX USART6. */
uint16_t BSP_UART6_Available(void);

/**
 * @brief Ambil 1 byte dari ring buffer RX USART6 (non-blocking).
 * @param out tujuan byte yang diambil
 * @return 1 kalau ada byte diambil, 0 kalau buffer kosong
 */
int BSP_UART6_ReadByte(uint8_t *out);

/**
 * @brief Kirim N byte lewat USART6 (blocking), dipakai protocol.c untuk
 *        membalas frame yang datang dari transport ini.
 * @param data buffer yang dikirim
 * @param len jumlah byte
 */
void BSP_UART6_WriteBuf(const uint8_t *data, uint16_t len);

#endif /* BSP_UART_H */
