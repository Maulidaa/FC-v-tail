/**
 * calib_mag.h
 *
 * Kalibrasi hard-iron & soft-iron sederhana untuk magnetometer, dengan
 * gerakan figure-8 (protocol.md Bagian 10 & command
 * CMD_CALIB_MAG_START/STOP, ID 0x0403/0x0404).
 *
 * Sama seperti calib_accel_gyro, modul ini tidak membaca sensor
 * sendiri — caller memanggil CalibMag_FeedSample() tiap ada sample
 * baru dari mag_compass.c selama state IN_PROGRESS.
 *
 * METODE (v1, disederhanakan — bukan full ellipsoid fit):
 *   - Hard-iron offset per axis = (min + max) / 2 selama sesi.
 *   - Soft-iron scale per axis = average_range / axis_range, supaya
 *     ketiga axis dinormalisasi ke skala yang sama (mendekati bentuk
 *     bola, bukan elips) — cukup untuk kompas hobi, bukan presisi
 *     survey-grade.
 *   - Progress dihitung dari cakupan orientasi: raw vector dibagi ke
 *     8 oktan berdasarkan tanda (x,y,z), progress = oktan_terkunjungi/8.
 *     Figure-8 yang benar akan menyentuh sebagian besar oktan; kalau
 *     device cuma diputar di satu bidang datar, progress akan mentok
 *     di bawah 100% — ini disengaja, jadi user tahu perlu variasi
 *     orientasi lebih (miring, bukan cuma diputar rata).
 */

#ifndef CALIB_MAG_H
#define CALIB_MAG_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Selaras dengan field `state` CMD_CALIB_STATUS, sama seperti
 * calib_accel_gyro.h: 0=idle,1=in_progress,2=done,3=failed. */
typedef enum {
    CALIB_MAG_STATE_IDLE        = 0,
    CALIB_MAG_STATE_IN_PROGRESS = 1,
    CALIB_MAG_STATE_DONE        = 2,
    CALIB_MAG_STATE_FAILED      = 3
} CalibMagState_t;

typedef struct {
    int16_t offset_x; /* hard-iron offset, satuan raw LSB */
    int16_t offset_y;
    int16_t offset_z;
    float   scale_x;  /* soft-iron scale, dikalikan ke (raw - offset) */
    float   scale_y;
    float   scale_z;
} MagCalibResult_t;

/**
 * Mulai kalibrasi: reset min/max & bitmask oktan, set state
 * IN_PROGRESS. Pengecekan "kalibrasi lain sedang jalan" tetap
 * tanggung jawab caller (calib_dispatcher).
 */
void CalibMag_Start(void);

/** Batalkan kalibrasi yang sedang berjalan, kembali ke IDLE. */
void CalibMag_Stop(void);

/**
 * Umpankan satu sample raw mag. No-op kalau state bukan IN_PROGRESS.
 * Kalibrasi otomatis pindah ke DONE begitu seluruh 8 oktan tersentuh
 * (dicek internal tiap panggilan ini, tidak perlu polling terpisah).
 */
void CalibMag_FeedSample(int16_t x, int16_t y, int16_t z);

CalibMagState_t CalibMag_GetState(void);

/** 0-100, dari jumlah oktan yang sudah tersentuh minimal sekali. */
uint8_t CalibMag_GetProgressPercent(void);

/**
 * Salin hasil ke *out. Return false kalau state bukan DONE.
 * Kalibrasi ini tidak punya kondisi FAILED otomatis (beda dari
 * accel/gyro) karena tidak ada cara murah mendeteksi "figure-8 salah"
 * dari data mentah saja — kalau progress macet di bawah 100% user
 * cukup diminta ulangi gerakan, bukan digagalkan paksa oleh firmware.
 */
bool CalibMag_GetResult(MagCalibResult_t *out);

#ifdef __cplusplus
}
#endif

#endif /* CALIB_MAG_H */
