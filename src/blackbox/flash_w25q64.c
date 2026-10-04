/**
 * @file    flash_w25q64.c
 * @brief   Implementasi driver W25Q64. Lihat flash_w25q64.h untuk
 *          dokumentasi API dan kontrak dependency ke bsp_spi.c.
 */

#include <stddef.h>
#include "flash_w25q64.h"

/* --- Dependency dari bsp_spi.c (lihat kontrak di flash_w25q64.h) --- */
extern bool     BSP_SPI2_Init(void);
extern bool     BSP_SPI2_TransferBuffer(const uint8_t *tx, uint8_t *rx, uint16_t len);
extern void     BSP_SPI2_CS_Assert(void);
extern void     BSP_SPI2_CS_Release(void);
extern uint32_t BSP_GetTickMs(void);

/* --- Command set W25Q64 (datasheet Winbond) --- */
#define W25Q64_CMD_WRITE_ENABLE     (0x06u)
#define W25Q64_CMD_WRITE_DISABLE    (0x04u)
#define W25Q64_CMD_READ_STATUS1     (0x05u)
#define W25Q64_CMD_PAGE_PROGRAM     (0x02u)
#define W25Q64_CMD_READ_DATA        (0x03u)
#define W25Q64_CMD_SECTOR_ERASE     (0x20u)
#define W25Q64_CMD_JEDEC_ID         (0x9Fu)

#define W25Q64_STATUS1_BUSY_BIT     (0x01u)

static W25Q64_Status_t w25q64_wait_busy(void)
{
    uint32_t start_ms = BSP_GetTickMs();

    for (;;) {
        uint8_t tx[2] = { W25Q64_CMD_READ_STATUS1, 0x00 };
        uint8_t rx[2] = { 0 };

        BSP_SPI2_CS_Assert();
        bool ok = BSP_SPI2_TransferBuffer(tx, rx, sizeof(tx));
        BSP_SPI2_CS_Release();

        if (!ok) {
            return W25Q64_ERR_SPI;
        }

        if ((rx[1] & W25Q64_STATUS1_BUSY_BIT) == 0u) {
            return W25Q64_OK;
        }

        if ((BSP_GetTickMs() - start_ms) > W25Q64_OP_TIMEOUT_MS) {
            return W25Q64_ERR_TIMEOUT;
        }
    }
}

static W25Q64_Status_t w25q64_write_enable(void)
{
    uint8_t cmd = W25Q64_CMD_WRITE_ENABLE;

    BSP_SPI2_CS_Assert();
    bool ok = BSP_SPI2_TransferBuffer(&cmd, NULL, 1u);
    BSP_SPI2_CS_Release();

    return ok ? W25Q64_OK : W25Q64_ERR_SPI;
}

W25Q64_Status_t W25Q64_ReadJedecId(uint8_t *manufacturer_id, uint16_t *device_id)
{
    if (manufacturer_id == NULL || device_id == NULL) {
        return W25Q64_ERR_BAD_PARAM;
    }

    uint8_t tx[4] = { W25Q64_CMD_JEDEC_ID, 0x00, 0x00, 0x00 };
    uint8_t rx[4] = { 0 };

    BSP_SPI2_CS_Assert();
    bool ok = BSP_SPI2_TransferBuffer(tx, rx, sizeof(tx));
    BSP_SPI2_CS_Release();

    if (!ok) {
        return W25Q64_ERR_SPI;
    }

    *manufacturer_id = rx[1];
    *device_id = ((uint16_t)rx[2] << 8) | (uint16_t)rx[3];

    return W25Q64_OK;
}

W25Q64_Status_t W25Q64_Init(void)
{
    if (!BSP_SPI2_Init()) {
        return W25Q64_ERR_SPI;
    }

    uint8_t manufacturer_id = 0;
    uint16_t device_id = 0;

    W25Q64_Status_t status = W25Q64_ReadJedecId(&manufacturer_id, &device_id);
    if (status != W25Q64_OK) {
        return status;
    }

    if (manufacturer_id != W25Q64_EXPECTED_MANUFACTURER_ID ||
        device_id != W25Q64_EXPECTED_DEVICE_ID) {
        return W25Q64_ERR_ID_MISMATCH;
    }

    return W25Q64_OK;
}

W25Q64_Status_t W25Q64_ReadData(uint32_t address, uint8_t *buffer, uint32_t length)
{
    if (buffer == NULL || length == 0u) {
        return W25Q64_ERR_BAD_PARAM;
    }
    if ((address + length) > W25Q64_CAPACITY_BYTES) {
        return W25Q64_ERR_BAD_PARAM;
    }

    uint8_t header[4] = {
        W25Q64_CMD_READ_DATA,
        (uint8_t)((address >> 16) & 0xFFu),
        (uint8_t)((address >> 8) & 0xFFu),
        (uint8_t)(address & 0xFFu),
    };

    BSP_SPI2_CS_Assert();

    bool ok = BSP_SPI2_TransferBuffer(header, NULL, sizeof(header));
    if (ok) {
        /* Dummy TX bytes (0x00) selagi menerima data — banyak implementasi
         * BSP_SPI2_TransferBuffer mengasumsikan tx non-NULL; kalau bsp_spi.c
         * final kalian sudah mendukung tx=NULL sebagai "kirim 0x00 otomatis",
         * baris di bawah bisa disederhanakan. Untuk keamanan v1, kirim
         * buffer dummy eksplisit. */
        for (uint32_t i = 0; i < length && ok; ) {
            uint8_t dummy_tx[32] = { 0 };
            uint16_t chunk = (uint16_t)((length - i) > sizeof(dummy_tx) ? sizeof(dummy_tx) : (length - i));
            ok = BSP_SPI2_TransferBuffer(dummy_tx, &buffer[i], chunk);
            i += chunk;
        }
    }

    BSP_SPI2_CS_Release();

    return ok ? W25Q64_OK : W25Q64_ERR_SPI;
}

W25Q64_Status_t W25Q64_PageProgram(uint32_t address, const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0u || length > W25Q64_PAGE_SIZE_BYTES) {
        return W25Q64_ERR_BAD_PARAM;
    }
    if ((address + length) > W25Q64_CAPACITY_BYTES) {
        return W25Q64_ERR_BAD_PARAM;
    }

    /* Tolak eksplisit kalau write akan melewati batas page — lihat catatan
     * di flash_w25q64.h, sengaja tidak meniru perilaku wrap-around chip asli. */
    uint32_t page_start = address & ~((uint32_t)W25Q64_PAGE_SIZE_BYTES - 1u);
    if ((address + length) > (page_start + W25Q64_PAGE_SIZE_BYTES)) {
        return W25Q64_ERR_BAD_PARAM;
    }

    W25Q64_Status_t status = w25q64_write_enable();
    if (status != W25Q64_OK) {
        return status;
    }

    uint8_t header[4] = {
        W25Q64_CMD_PAGE_PROGRAM,
        (uint8_t)((address >> 16) & 0xFFu),
        (uint8_t)((address >> 8) & 0xFFu),
        (uint8_t)(address & 0xFFu),
    };

    BSP_SPI2_CS_Assert();
    bool ok = BSP_SPI2_TransferBuffer(header, NULL, sizeof(header));
    if (ok) {
        ok = BSP_SPI2_TransferBuffer(data, NULL, length);
    }
    BSP_SPI2_CS_Release();

    if (!ok) {
        return W25Q64_ERR_SPI;
    }

    return w25q64_wait_busy();
}

W25Q64_Status_t W25Q64_SectorErase(uint32_t address)
{
    if (address >= W25Q64_CAPACITY_BYTES) {
        return W25Q64_ERR_BAD_PARAM;
    }

    uint32_t sector_start = address & ~((uint32_t)W25Q64_SECTOR_SIZE_BYTES - 1u);

    W25Q64_Status_t status = w25q64_write_enable();
    if (status != W25Q64_OK) {
        return status;
    }

    uint8_t header[4] = {
        W25Q64_CMD_SECTOR_ERASE,
        (uint8_t)((sector_start >> 16) & 0xFFu),
        (uint8_t)((sector_start >> 8) & 0xFFu),
        (uint8_t)(sector_start & 0xFFu),
    };

    BSP_SPI2_CS_Assert();
    bool ok = BSP_SPI2_TransferBuffer(header, NULL, sizeof(header));
    BSP_SPI2_CS_Release();

    if (!ok) {
        return W25Q64_ERR_SPI;
    }

    return w25q64_wait_busy();
}
