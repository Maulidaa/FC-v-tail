/**
 * @file    flash_w25q64.h
 * @brief   Driver low-level flash W25Q64 (8MB, SPI) via SPI2 —
 *          PB12=CS, PB13=SCK, PB14=MISO, PB15=MOSI.
 *          Lihat pinout-fc-stm32f411.md Bagian 1 & 3, dan
 *          firmware-architecture-stm32f411.md Bagian 3.13 (blackbox).
 *
 * Lapisan ini HANYA soal command-set W25Q64 (JEDEC ID, read, page program,
 * sector erase, status register) — tidak tahu apa-apa soal format record
 * log, itu tanggung jawab blackbox.h/c di atasnya.
 *
 * === Dependency kontrak ke bsp_spi.c/h (Orang 1) ===
 * SPI2 ditandai [BARU] di pinout-fc-stm32f411.md (beda dari SPI1 yang sudah
 * dipakai imu_mpu6500.c). Modul ini TIDAK menyentuh register SPI/GPIO
 * langsung — supaya driver flash bisa dites logic-nya (mock SPI) tanpa
 * hardware, dan supaya inisialisasi SPI2 level-bus tetap satu tempat di
 * bsp_spi.c, konsisten dengan bsp_uart.c/bsp_i2c.c yang sudah ada.
 * Fungsi-fungsi berikut diasumsikan disediakan modul lain (bsp_spi.h) dan
 * WAJIB dikoordinasikan ke Orang 1 sebelum driver ini dipakai:
 *
 *   bool     BSP_SPI2_Init(void);
 *   bool     BSP_SPI2_TransferBuffer(const uint8_t *tx, uint8_t *rx, uint16_t len);
 *   void     BSP_SPI2_CS_Assert(void);   // tarik PB12 low
 *   void     BSP_SPI2_CS_Release(void);  // lepas PB12 high
 *   uint32_t BSP_GetTickMs(void);        // tick milidetik dari scheduler/systick
 *
 * Kalau nama/tanda tangan fungsi ini beda di bsp_spi.c final, tinggal
 * sesuaikan bagian #include & pemanggilan di flash_w25q64.c — logic command
 * W25Q64 di file ini tidak perlu berubah.
 */

#ifndef FLASH_W25Q64_H
#define FLASH_W25Q64_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define W25Q64_PAGE_SIZE_BYTES     (256u)
#define W25Q64_SECTOR_SIZE_BYTES   (4096u)
#define W25Q64_CAPACITY_BYTES      (8u * 1024u * 1024u) /* 8MB / 64Mbit */

/* JEDEC ID standar Winbond W25Q64: manufacturer 0xEF, device 0x4017
 * (memory type 0x40, capacity 0x17). Dipakai W25Q64_Init() untuk
 * memverifikasi chip yang terpasang benar sebelum dipakai untuk logging
 * yang dipercaya. */
#define W25Q64_EXPECTED_MANUFACTURER_ID (0xEFu)
#define W25Q64_EXPECTED_DEVICE_ID       (0x4017u)

/* Timeout wajar untuk operasi erase/program — sector erase W25Q64 tipikal
 * <400ms per datasheet, diberi margin. Page program jauh lebih cepat
 * (<3ms tipikal) tapi dipakai timeout yang sama untuk kesederhanaan. */
#define W25Q64_OP_TIMEOUT_MS (500u)

typedef enum {
    W25Q64_OK = 0,
    W25Q64_ERR_SPI,        /* transfer SPI gagal (lapisan bsp) */
    W25Q64_ERR_TIMEOUT,    /* BUSY tidak turun dalam W25Q64_OP_TIMEOUT_MS */
    W25Q64_ERR_ID_MISMATCH,/* JEDEC ID tidak cocok W25Q64 — chip salah/tidak terpasang */
    W25Q64_ERR_BAD_PARAM,  /* address/length di luar kapasitas atau melewati batas page */
} W25Q64_Status_t;

/**
 * @brief Inisialisasi bus SPI2 (lewat BSP_SPI2_Init()) dan verifikasi JEDEC
 *        ID chip yang terpasang. WAJIB dipanggil sebelum fungsi lain di
 *        modul ini, dan WAJIB dicek return value-nya sebelum blackbox.c
 *        mempercayai flash untuk logging — flash yang gagal deteksi tidak
 *        boleh diam-diam dianggap "logging jalan".
 */
W25Q64_Status_t W25Q64_Init(void);

/**
 * @brief Baca data mentah, panjang & alamat bebas (tidak perlu align page/sector).
 */
W25Q64_Status_t W25Q64_ReadData(uint32_t address, uint8_t *buffer, uint32_t length);

/**
 * @brief Tulis data ke satu page (maks W25Q64_PAGE_SIZE_BYTES byte, TIDAK
 *        boleh melewati batas page — kalau address+length melewati batas
 *        page yang sama, fungsi ini mengembalikan W25Q64_ERR_BAD_PARAM
 *        alih-alih diam-diam wrap-around di dalam page seperti perilaku
 *        chip aslinya, supaya bug pemanggil ketahuan segera).
 *        Sel flash tujuan HARUS sudah dalam kondisi ter-erase (0xFF)
 *        sebelum dipanggil — program tidak melakukan erase implisit.
 */
W25Q64_Status_t W25Q64_PageProgram(uint32_t address, const uint8_t *data, uint16_t length);

/**
 * @brief Erase satu sector (4KB) yang mengandung `address` (address
 *        di-floor ke kelipatan W25Q64_SECTOR_SIZE_BYTES secara internal).
 *        Blocking sampai BUSY turun atau timeout.
 */
W25Q64_Status_t W25Q64_SectorErase(uint32_t address);

/**
 * @brief Baca ulang JEDEC ID (dipakai internal oleh W25Q64_Init(), diekspos
 *        juga untuk keperluan diagnostik/CMD_GET_STATUS kalau diperlukan).
 */
W25Q64_Status_t W25Q64_ReadJedecId(uint8_t *manufacturer_id, uint16_t *device_id);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_W25Q64_H */
