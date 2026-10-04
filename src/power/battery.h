/**
 * @file    battery.h
 * @brief   Power monitoring — VBAT (PA0/ADC1_IN0) & current sense
 *          (PA1/ADC1_IN1). Lihat firmware-architecture-stm32f411.md
 *          Bagian 3.10 dan pinout-fc-stm32f411.md Bagian 1.
 *
 * Modul ini TIDAK menyentuh register ADC langsung — itu tanggung jawab
 * `bsp_adc.c/h` (Orang 1). battery.c murni menerima nilai ADC mentah lewat
 * Battery_Update() dan mengubahnya jadi tegangan/arus terkalibrasi.
 * Pemisahan ini disengaja: driver ADC low-level bisa dipakai modul lain
 * tanpa terikat ke logic kalibrasi baterai, dan battery.c bisa diuji
 * host-side (unit test) tanpa hardware nyata (lihat test/ di struktur
 * folder firmware-architecture-stm32f411.md Bagian 2).
 *
 * Kalibrasi:
 * VBAT lewat resistor divider, current sense dari amplifier analog
 * (INA139/INA180/INA199 atau ACS712/ACS758 — tipe pasti belum dikonfirmasi,
 * lihat item terbuka firmware-architecture-stm32f411.md Bagian 5). Karena
 * faktor skala berbeda antar tipe amplifier dan antar board (toleransi
 * komponen), modul ini TIDAK hardcode satu skala tetap — skala/offset
 * disimpan di BatteryConfig_t, diisi nilai default konservatif dulu, lalu
 * dikalibrasi ulang per board dan (idealnya) diekspos ke SETTING_GET/SET
 * sama seperti parameter mixer.
 */

#ifndef BATTERY_H
#define BATTERY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Resolusi ADC1 STM32F411 (12-bit, 0..4095), dipakai sebagai
 *        referensi konversi raw->tegangan sebelum kalibrasi. Kalau bsp_adc.c
 *        dikonfigurasi oversampling/resolusi berbeda, sesuaikan nilai ini
 *        atau lakukan penyesuaian di sisi bsp_adc sebelum raw diteruskan
 *        ke Battery_Update().
 */
#define BATTERY_ADC_RESOLUTION_COUNTS (4095u)

/**
 * @brief VREF ADC dalam volt. STM32F411 Blackpill umumnya VDD=3.3V dipakai
 *        sebagai VREF+ (tidak ada VREF eksternal terpisah di board ini).
 */
#define BATTERY_ADC_VREF_VOLTS (3.3f)

/**
 * @brief Parameter kalibrasi linear: nilai_fisik = (raw_volt_at_pin * scale) + offset.
 *
 * Untuk VBAT lewat resistor divider R1/R2 (VBAT di titik tengah dibaca ADC):
 *   scale default = (R1+R2)/R2 (rasio divider), offset default = 0.0
 * Nilai R1/R2 pasti di board kalian belum dikonfirmasi — lihat item
 * terbuka pinout-fc-stm32f411.md, jadi default di bawah adalah PLACEHOLDER
 * yang HARUS diverifikasi/dikalibrasi ulang sebelum dipakai untuk keputusan
 * safety (mis. low-voltage failsafe), bukan nilai final siap pakai.
 *
 * Untuk current sense, scale/offset tergantung tipe amplifier terpasang
 * (INA139/180/199 pakai Rsense+gain tetap, ACS712/758 pakai mV/A linear
 * dari datasheet) — juga masih item terbuka, lihat catatan di atas.
 */
typedef struct {
    float voltage_scale;   /* rasio divider VBAT, tanpa satuan */
    float voltage_offset;  /* volt, biasanya 0.0 */

    float current_scale;   /* ampere per volt keluaran amplifier current sense */
    float current_offset;  /* ampere, offset bias amplifier (banyak chip current
                               sense punya offset di tengah rentang, bukan 0V=0A) */

    /* Low-pass filter (EMA) untuk meredam noise ADC. Rentang [0.0, 1.0),
     * 0.0 = tanpa filter (raw tiap sampel dipakai langsung). */
    float filter_alpha;

    /* Ambang batas failsafe low-voltage, dalam volt per sel (agar tidak
     * tergantung jumlah sel baterai) dan jumlah sel — dipakai
     * Battery_IsLowVoltage(). Nilai default konservatif untuk LiPo. */
    float low_voltage_threshold_per_cell;
    uint8_t cell_count;
} BatteryConfig_t;

/**
 * @brief Hasil pembacaan baterai terkini, sudah difilter & terkalibrasi.
 */
typedef struct {
    float voltage_volts;
    float current_amps;

    /* Akumulasi muatan terpakai sejak boot/reset, dalam miliampere-hour.
     * Diperbarui tiap Battery_Update() lewat integrasi current*dt —
     * berguna untuk estimasi kapasitas tersisa di telemetry/blackbox. */
    float consumed_mah;

    bool valid; /* false kalau belum ada sampel sejak Battery_Init() */
} BatteryReading_t;

/**
 * @brief Isi config dengan nilai default PLACEHOLDER (lihat catatan di
 *        BatteryConfig_t) dan reset state internal.
 *
 * @param config     Config yang akan diinisialisasi.
 * @param cell_count Jumlah sel baterai (mis. 3 untuk 3S LiPo) — dipakai
 *                    untuk ambang low-voltage total. Kalau 0, dipaksa ke 1
 *                    supaya tidak divide-by-zero di kalkulasi terkait.
 */
void Battery_Init(BatteryConfig_t *config, uint8_t cell_count);

/**
 * @brief Set parameter kalibrasi tegangan (voltage divider).
 */
void Battery_SetVoltageCalibration(BatteryConfig_t *config, float scale, float offset);

/**
 * @brief Set parameter kalibrasi current sense.
 */
void Battery_SetCurrentCalibration(BatteryConfig_t *config, float scale, float offset);

/**
 * @brief Set ambang failsafe low-voltage per sel (volt) dan koefisien filter.
 */
void Battery_SetThresholds(BatteryConfig_t *config, float low_voltage_threshold_per_cell);

/**
 * @brief Update pembacaan baterai dari nilai ADC mentah.
 *
 * @param config       Parameter kalibrasi saat ini (read-only di sini).
 * @param reading      State pembacaan yang di-update in-place (termasuk
 *                      consumed_mah, jadi instance ini harus persist antar
 *                      panggilan, bukan dibuat ulang tiap kali).
 * @param raw_vbat      Nilai ADC1_IN0 mentah (0..BATTERY_ADC_RESOLUTION_COUNTS).
 * @param raw_current    Nilai ADC1_IN1 mentah (0..BATTERY_ADC_RESOLUTION_COUNTS).
 * @param dt_seconds     Delta waktu sejak update terakhir, untuk integrasi
 *                        consumed_mah. Kalau <= 0 (first-call/jitter),
 *                        consumed_mah tidak diakumulasi pada panggilan ini,
 *                        tapi voltage/current tetap diperbarui.
 */
void Battery_Update(const BatteryConfig_t *config, BatteryReading_t *reading,
                     uint16_t raw_vbat, uint16_t raw_current, float dt_seconds);

/**
 * @brief Cek apakah pembacaan terkini di bawah ambang low-voltage total
 *        (threshold_per_cell * cell_count). Mengembalikan false kalau
 *        reading belum valid (belum ada sampel) — supaya tidak memicu
 *        failsafe palsu sebelum ADC pertama kali terbaca.
 */
bool Battery_IsLowVoltage(const BatteryConfig_t *config, const BatteryReading_t *reading);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_H */
