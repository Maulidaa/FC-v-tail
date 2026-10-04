/**
 * gps_ubx.h
 *
 * Parser UBX untuk u-blox NEO-6M (protokol u-blox 6), dikonsumsi
 * byte-per-byte dari USART2 (PA2=TX, PA3=RX).
 *
 * KENAPA BUKAN NAV-PVT: NAV-PVT baru ada di protokol u-blox 7 ke atas.
 * NEO-6M TIDAK mendukungnya. Sebagai gantinya modul ini menggabungkan
 * tiga message dari epoch navigasi yang sama (iTOW sama):
 *   NAV-POSLLH (0x01 0x02, 28 B) -> lat, lon, height, hMSL, hAcc
 *   NAV-VELNED (0x01 0x12, 36 B) -> ground speed, heading
 *   NAV-SOL    (0x01 0x06, 52 B) -> fix type, flags(gpsFixOK), pDOP, numSV
 * GPS_UBX_ProcessByte() baru return true kalau ketiganya lengkap.
 *
 * NEO-6M default: NMEA @9600 baud, 1 Hz. Modul ini menyediakan urutan
 * konfigurasi (GPS_UBX_ConfigStep) untuk mengaktifkan UBX-only, 3
 * message di atas, dan 5 Hz (maks NEO-6M). Baud TETAP 9600.
 *
 * Modul ini murni parsing: tidak tahu soal waktu (tick) dan tidak
 * menyentuh UART langsung. Pengiriman byte lewat callback GPS_UBX_TxFn,
 * pencatatan waktu frame dilakukan pemanggil (lihat task_gps di main.c).
 */

#ifndef GPS_UBX_H
#define GPS_UBX_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Model dinamik receiver (CFG-NAV5). 6 = Airborne <1g (pesawat sayap
 * tetap). Multirotor: 7 (<2g) atau 3 (pedestrian). 0 = portable (default
 * pabrik, membatasi kecepatan/altitude — kurang cocok untuk pesawat). */
#ifndef GPS_UBX_DYN_MODEL
#define GPS_UBX_DYN_MODEL  6u
#endif

/* Laju navigasi. NEO-6M maksimal 5 Hz. */
#ifndef GPS_UBX_MEAS_RATE_MS
#define GPS_UBX_MEAS_RATE_MS  200u
#endif

typedef enum {
    GPS_FIX_NONE       = 0,
    GPS_FIX_DEAD_RECK  = 1,
    GPS_FIX_2D         = 2,
    GPS_FIX_3D         = 3,
    GPS_FIX_GNSS_DR    = 4,
    GPS_FIX_TIME_ONLY  = 5
} GPS_FixType_t;

typedef struct {
    int32_t  lat_e7;             /* 1e-7 derajat */
    int32_t  lon_e7;             /* 1e-7 derajat */
    int32_t  height_mm;          /* di atas ellipsoid */
    int32_t  height_msl_mm;      /* di atas mean sea level */
    int32_t  ground_speed_mm_s;  /* kecepatan horizontal */
    int32_t  heading_e5;         /* course over ground, 1e-5 derajat */
    GPS_FixType_t fix_type;
    bool     fix_ok;             /* flag gpsFixOK dari NAV-SOL — WAJIB true
                                  * sebelum posisi dipercaya */
    uint8_t  num_sv;
    uint16_t pdop_x100;          /* pDOP * 100 (skala 0.01 sesuai UBX) */
    uint32_t h_acc_mm;           /* estimasi akurasi horizontal */
    uint32_t itow_ms;
} GPS_Data_t;

/** Callback kirim byte ke UART GPS (mis. pembungkus BSP_UART2 write). */
typedef void (*GPS_UBX_TxFn)(const uint8_t *data, uint16_t len);

/** Reset parser + state konfigurasi. Panggil sekali di boot. */
void GPS_UBX_Init(void);

/**
 * Umpankan satu byte. Return true HANYA saat satu epoch lengkap
 * (POSLLH + VELNED + SOL dengan iTOW sama, semua checksum valid) baru
 * saja digabung — data baru siap diambil lewat GPS_UBX_GetData().
 */
bool GPS_UBX_ProcessByte(uint8_t byte);

/** Salin epoch lengkap terakhir. False kalau belum pernah ada. */
bool GPS_UBX_GetData(GPS_Data_t *out);

/** True kalau epoch terakhir punya fix 2D/3D DAN gpsFixOK. Ini status
 *  fix saat epoch terakhir, bukan jaminan data masih baru — cek umur
 *  frame di pemanggil. */
bool GPS_UBX_HasFix(void);

/**
 * Konfigurasi modul, satu message per panggilan (supaya TX yang
 * mungkin blocking tidak menahan scheduler lama). Return true kalau
 * seluruh urutan sudah terkirim. Urutan: CFG-RATE, CFG-NAV5,
 * CFG-MSG x3, CFG-PRT (UBX-only @9600, terakhir).
 */
bool GPS_UBX_ConfigStep(GPS_UBX_TxFn tx);

/** Ulangi urutan konfigurasi dari awal (mis. modul telat boot). */
void GPS_UBX_ConfigRestart(void);

/** True kalau urutan konfigurasi sudah selesai terkirim. */
bool GPS_UBX_ConfigDone(void);

#ifdef __cplusplus
}
#endif

#endif /* GPS_UBX_H */
