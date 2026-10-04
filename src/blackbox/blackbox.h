/**
 * @file    blackbox.h
 * @brief   Format & manajemen logging blackbox di atas flash_w25q64.h.
 *          Lihat pembagian-tugas-firmware-4-orang.md Bagian: keputusan
 *          desain blackbox, dan protocol.md Bagian 12 (item terbuka
 *          "Format record blackbox" — dokumen ini yang menuntaskannya
 *          dengan format tersendiri, BUKAN reuse framing protocol.md,
 *          supaya overhead per-record kecil untuk kecepatan tulis flash).
 *
 * Format record (per keputusan desain):
 *   [record_type: u8][timestamp_ms: u32][payload_len: u8][payload...][crc8: u8]
 *
 * Format header sesi (ditulis di byte pertama tiap kali sesi logging baru
 * dimulai, yaitu setelah sector pertama di-erase):
 *   [magic: 4 byte "BBLK"][format_version: u8][reserved: 3 byte]
 *
 * CRC8: fungsi crc8Dvbs2() DIPAKAI ULANG dari modul protokol (comms/), sesuai
 * keputusan desain eksplisit di pembagian-tugas-firmware-4-orang.md — bukan
 * diimplementasikan ulang di sini. Domain checksum per record mengikuti pola
 * yang sama seperti protocol.md Bagian 4 (checksum atas seluruh byte record
 * KECUALI byte crc8 itu sendiri): [record_type, timestamp_ms(4 byte LE),
 * payload_len, payload...].
 */

#ifndef BLACKBOX_H
#define BLACKBOX_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLACKBOX_MAGIC_BYTE0 ('B')
#define BLACKBOX_MAGIC_BYTE1 ('B')
#define BLACKBOX_MAGIC_BYTE2 ('L')
#define BLACKBOX_MAGIC_BYTE3 ('K')
#define BLACKBOX_FORMAT_VERSION (1u)
#define BLACKBOX_HEADER_SIZE_BYTES (8u) /* magic(4) + version(1) + reserved(3) */

/* Payload maksimum per record dibatasi field payload_len (u8) -> 255 byte.
 * Record maksimum total = 1(type)+4(ts)+1(len)+255(payload)+1(crc) = 262 byte. */
#define BLACKBOX_MAX_PAYLOAD_BYTES (255u)
#define BLACKBOX_MAX_RECORD_BYTES (1u + 4u + 1u + BLACKBOX_MAX_PAYLOAD_BYTES + 1u)

/**
 * @brief Jenis record blackbox. Rentang dijaga longgar (bukan enum ketat di
 *        header ini) supaya penambahan record_type baru tidak perlu
 *        recompile pembaca/penulis lain — nilai konkret didefinisikan di
 *        modul yang memanggil Blackbox_WriteRecord() (mis. main.c loop
 *        logging, sesuai jenis data apa yang mau direkam per siklus).
 *        Beberapa nilai awal disediakan sebagai starting point:
 */
typedef enum {
    BLACKBOX_RECORD_ATTITUDE = 0x01,   /* mirror payload mirip CMD_ATTITUDE */
    BLACKBOX_RECORD_GPS = 0x02,        /* mirip CMD_GPS_DATA */
    BLACKBOX_RECORD_BATTERY = 0x03,    /* mirip CMD_BATTERY */
    BLACKBOX_RECORD_MIXER_OUTPUT = 0x04, /* nilai channel dari mixer.c, untuk debug tuning */
    BLACKBOX_RECORD_EVENT = 0x05,      /* event diskrit: arm/disarm, mode switch, failsafe trigger */
} BlackboxRecordType_t;

typedef enum {
    BLACKBOX_OK = 0,
    BLACKBOX_ERR_FLASH,        /* operasi flash_w25q64.h gagal */
    BLACKBOX_ERR_BAD_PARAM,    /* payload_len > BLACKBOX_MAX_PAYLOAD_BYTES, dll */
    BLACKBOX_ERR_FULL,         /* kapasitas flash habis, lihat catatan di Blackbox_WriteRecord() */
    BLACKBOX_ERR_NOT_INITIALIZED,
} Blackbox_Status_t;

/**
 * @brief State sesi logging saat ini. Instance ini harus persist selama
 *        firmware hidup (biasanya satu instance global/static di
 *        blackbox.c, diakses lewat fungsi-fungsi di bawah — header ini
 *        sengaja tidak mengekspos struct-nya supaya pemanggil tidak
 *        bergantung ke layout internal).
 */

/**
 * @brief Inisialisasi flash (lewat W25Q64_Init()) dan mulai sesi logging
 *        baru: erase sector pertama, tulis header sesi di awalnya, set
 *        write pointer tepat setelah header.
 *
 * WAJIB dipanggil sebelum Blackbox_WriteRecord() dan HARUS dicek return
 * value-nya — kalau flash gagal terdeteksi (chip salah/tidak terpasang),
 * panggilan Blackbox_WriteRecord() berikutnya akan langsung gagal dengan
 * BLACKBOX_ERR_NOT_INITIALIZED, bukan diam-diam kehilangan data.
 *
 * @param start_address Alamat awal area flash yang dipakai untuk sesi ini
 *                       (harus kelipatan W25Q64_SECTOR_SIZE_BYTES). Dengan
 *                       ini, blackbox bisa berbagi chip flash yang sama
 *                       dengan area lain (mis. cadangan setting) tanpa
 *                       tabrakan, cukup beri offset berbeda.
 * @param area_size_bytes Ukuran area yang dialokasikan untuk blackbox
 *                         (harus kelipatan sector). Dipakai
 *                         Blackbox_WriteRecord() untuk tahu kapan area
 *                         penuh (lihat BLACKBOX_ERR_FULL).
 */
Blackbox_Status_t Blackbox_Init(uint32_t start_address, uint32_t area_size_bytes);

/**
 * @brief Tulis satu record ke flash.
 *
 * Perilaku saat area penuh (v1, disengaja sederhana): berhenti menulis dan
 * mengembalikan BLACKBOX_ERR_FULL terus-menerus untuk panggilan berikutnya,
 * TIDAK wrap-around menimpa data lama. Circular buffer dengan penomoran
 * sesi (supaya data lama boleh ditimpa tapi tetap bisa dibedakan sesi mana
 * saat dibaca) ditandai sebagai perbaikan v2 — untuk v1, kapasitas 8MB
 * dianggap cukup untuk durasi terbang wajar; kalau ternyata tidak, ini
 * yang perlu direvisi duluan.
 *
 * @param record_type   Lihat BlackboxRecordType_t (atau nilai custom lain).
 * @param timestamp_ms  Timestamp record, biasanya dari BSP_GetTickMs().
 * @param payload        Data payload, boleh NULL kalau payload_len=0.
 * @param payload_len    Maks BLACKBOX_MAX_PAYLOAD_BYTES.
 */
Blackbox_Status_t Blackbox_WriteRecord(uint8_t record_type, uint32_t timestamp_ms,
                                        const uint8_t *payload, uint8_t payload_len);

/**
 * @brief Berapa byte area logging yang sudah terpakai di sesi berjalan.
 *        Berguna untuk telemetry/status (mis. tambahan field di
 *        CMD_GET_STATUS) supaya web configurator bisa tampilkan kapasitas
 *        blackbox tersisa.
 */
uint32_t Blackbox_GetBytesUsed(void);

#ifdef __cplusplus
}
#endif

#endif /* BLACKBOX_H */
