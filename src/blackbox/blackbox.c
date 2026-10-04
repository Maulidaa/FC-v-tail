/**
 * @file    blackbox.c
 * @brief   Implementasi blackbox. Lihat blackbox.h untuk dokumentasi API.
 */

#include <stddef.h>
#include "blackbox.h"
#include "flash_w25q64.h"

/* Reuse fungsi CRC dari modul protokol (comms/), sesuai keputusan desain —
 * lihat catatan di blackbox.h. Signature disamakan dengan kontrak yang
 * dibekukan protocol.md Bagian 4: "fungsi murni tanpa state,
 * crc8Dvbs2(bytes, initial?)". Kalau signature final di comms/protocol.c
 * sedikit beda (mis. urutan parameter), sesuaikan baris extern ini saja. */
extern uint8_t Protocol_CRC8_DVBS2(const uint8_t *bytes, uint16_t length, uint8_t initial);

#define BLACKBOX_CRC_INITIAL (0x00u)

typedef struct {
    bool initialized;
    uint32_t area_start_address;
    uint32_t area_size_bytes;
    uint32_t write_offset; /* relatif ke area_start_address, sudah termasuk header sesi */
    /* Index sector (relatif ke area_start_address, sector 0 = sector pertama
     * area) tertinggi yang sudah pernah di-erase pada sesi ini. Dipakai
     * untuk erase-on-demand: sector baru cukup di-erase sekali, tepat
     * sebelum byte pertamanya ditulis. */
    uint32_t highest_erased_sector_index;
} BlackboxState_t;

static BlackboxState_t s_blackbox = {0};

static void blackbox_build_header(uint8_t out[BLACKBOX_HEADER_SIZE_BYTES])
{
    out[0] = (uint8_t)BLACKBOX_MAGIC_BYTE0;
    out[1] = (uint8_t)BLACKBOX_MAGIC_BYTE1;
    out[2] = (uint8_t)BLACKBOX_MAGIC_BYTE2;
    out[3] = (uint8_t)BLACKBOX_MAGIC_BYTE3;
    out[4] = BLACKBOX_FORMAT_VERSION;
    out[5] = 0u; /* reserved */
    out[6] = 0u; /* reserved */
    out[7] = 0u; /* reserved */
}

Blackbox_Status_t Blackbox_Init(uint32_t start_address, uint32_t area_size_bytes)
{
    if ((start_address % W25Q64_SECTOR_SIZE_BYTES) != 0u ||
        (area_size_bytes % W25Q64_SECTOR_SIZE_BYTES) != 0u ||
        area_size_bytes < W25Q64_SECTOR_SIZE_BYTES) {
        return BLACKBOX_ERR_BAD_PARAM;
    }

    if (W25Q64_Init() != W25Q64_OK) {
        return BLACKBOX_ERR_FLASH;
    }

    if (W25Q64_SectorErase(start_address) != W25Q64_OK) {
        return BLACKBOX_ERR_FLASH;
    }

    uint8_t header[BLACKBOX_HEADER_SIZE_BYTES];
    blackbox_build_header(header);

    if (W25Q64_PageProgram(start_address, header, sizeof(header)) != W25Q64_OK) {
        return BLACKBOX_ERR_FLASH;
    }

    s_blackbox.area_start_address = start_address;
    s_blackbox.area_size_bytes = area_size_bytes;
    s_blackbox.write_offset = BLACKBOX_HEADER_SIZE_BYTES;
    s_blackbox.highest_erased_sector_index = 0u; /* sector pertama baru saja di-erase di atas */
    s_blackbox.initialized = true;

    return BLACKBOX_OK;
}

Blackbox_Status_t Blackbox_WriteRecord(uint8_t record_type, uint32_t timestamp_ms,
                                        const uint8_t *payload, uint8_t payload_len)
{
    if (!s_blackbox.initialized) {
        return BLACKBOX_ERR_NOT_INITIALIZED;
    }
    /* payload_len bertipe uint8_t, jadi selama BLACKBOX_MAX_PAYLOAD_BYTES
     * masih 255 perbandingan di bawah SELALU false (gcc memperingatkannya
     * lewat -Wtype-limits). Cek-nya tidak dihapus -- ia jadi benar-benar
     * bekerja begitu batas itu diturunkan -- tapi dikawal preprocessor
     * supaya tidak ada peringatan palsu yang menutupi peringatan asli.
     * Ukuran buffer record[] di bawah ikut batas yang sama, jadi keduanya
     * tidak bisa lepas sinkron. */
#if BLACKBOX_MAX_PAYLOAD_BYTES < 255u
    if (payload_len > BLACKBOX_MAX_PAYLOAD_BYTES) {
        return BLACKBOX_ERR_BAD_PARAM;
    }
#endif
    if (payload_len > 0u && payload == NULL) {
        return BLACKBOX_ERR_BAD_PARAM;
    }

    uint8_t record[BLACKBOX_MAX_RECORD_BYTES];
    uint16_t record_len = 0;

    record[record_len++] = record_type;
    record[record_len++] = (uint8_t)(timestamp_ms & 0xFFu);
    record[record_len++] = (uint8_t)((timestamp_ms >> 8) & 0xFFu);
    record[record_len++] = (uint8_t)((timestamp_ms >> 16) & 0xFFu);
    record[record_len++] = (uint8_t)((timestamp_ms >> 24) & 0xFFu);
    record[record_len++] = payload_len;

    for (uint8_t i = 0; i < payload_len; i++) {
        record[record_len++] = payload[i];
    }

    /* CRC dihitung atas seluruh byte di atas (record_type..payload),
     * TIDAK termasuk byte crc8 itu sendiri — sama pola dengan protocol.md
     * Bagian 4. */
    uint8_t crc = Protocol_CRC8_DVBS2(record, record_len, BLACKBOX_CRC_INITIAL);
    record[record_len++] = crc;

    if ((s_blackbox.write_offset + record_len) > s_blackbox.area_size_bytes) {
        /* Area penuh — lihat catatan kebijakan "tidak wrap-around" di
         * blackbox.h. */
        return BLACKBOX_ERR_FULL;
    }

    uint32_t absolute_address = s_blackbox.area_start_address + s_blackbox.write_offset;

    /* Record tidak boleh dipecah program call melewati batas page W25Q64
     * (maks 256 byte per W25Q64_PageProgram) meskipun sudah dalam sector
     * yang sama & sudah ter-erase — jadi program per-page di sini. Sector
     * baru di-erase "on demand" tepat sebelum byte pertamanya ditulis
     * (flash hanya bisa program bit 1->0; sel yang belum di-erase akan
     * korup) — pendekatan ini menghindari erase seluruh area_size_bytes
     * di depan (lambat kalau area besar) tanpa mengorbankan korupsi data. */
    uint16_t bytes_written = 0;
    while (bytes_written < record_len) {
        uint32_t chunk_address = absolute_address + bytes_written;
        uint32_t chunk_offset_in_area = s_blackbox.write_offset + bytes_written;
        uint32_t chunk_sector_index = chunk_offset_in_area / W25Q64_SECTOR_SIZE_BYTES;

        if (chunk_sector_index > s_blackbox.highest_erased_sector_index) {
            uint32_t sector_absolute_address =
                s_blackbox.area_start_address + (chunk_sector_index * W25Q64_SECTOR_SIZE_BYTES);
            if (W25Q64_SectorErase(sector_absolute_address) != W25Q64_OK) {
                return BLACKBOX_ERR_FLASH;
            }
            s_blackbox.highest_erased_sector_index = chunk_sector_index;
        }

        uint16_t page_offset = (uint16_t)(chunk_address % W25Q64_PAGE_SIZE_BYTES);
        uint16_t max_chunk_in_page = (uint16_t)(W25Q64_PAGE_SIZE_BYTES - page_offset);
        uint16_t remaining = (uint16_t)(record_len - bytes_written);
        uint16_t chunk_len = (remaining < max_chunk_in_page) ? remaining : max_chunk_in_page;

        if (W25Q64_PageProgram(chunk_address, &record[bytes_written], chunk_len) != W25Q64_OK) {
            return BLACKBOX_ERR_FLASH;
        }

        bytes_written += chunk_len;
    }

    s_blackbox.write_offset += record_len;

    return BLACKBOX_OK;
}

uint32_t Blackbox_GetBytesUsed(void)
{
    return s_blackbox.initialized ? s_blackbox.write_offset : 0u;
}
