/**
 * @file    battery.c
 * @brief   Implementasi power monitoring. Lihat battery.h untuk dokumentasi API.
 */

#include "battery.h"
#include <math.h>
#include <stddef.h>   /* NULL -- lihat catatan yang sama di pid.c */

/* Placeholder kalibrasi awal — WAJIB diverifikasi ulang per board, lihat
 * catatan panjang di battery.h. Rasio divider contoh: R1=10k (atas),
 * R2=3.3k (bawah) -> scale = (10+3.3)/3.3 ~= 4.03, cocok untuk baterai
 * sampai ~4S (16.8V) dengan VREF 3.3V. GANTI sesuai nilai resistor riil
 * di board kalian sebelum dipakai untuk keputusan safety apa pun. */
#define BATTERY_DEFAULT_VOLTAGE_SCALE   (4.03f)
#define BATTERY_DEFAULT_VOLTAGE_OFFSET  (0.0f)

/* Placeholder current sense — angka netral (gain 1, offset 0) SENGAJA
 * bukan nilai datasheet chip tertentu, supaya siapa pun yang belum
 * mengkalibrasi ulang langsung sadar dari hasil baca yang jelas salah
 * (bukan angka masuk akal yang menipu), bukan pura-pura sudah benar. */
#define BATTERY_DEFAULT_CURRENT_SCALE   (1.0f)
#define BATTERY_DEFAULT_CURRENT_OFFSET  (0.0f)

#define BATTERY_DEFAULT_FILTER_ALPHA               (0.9f)
#define BATTERY_DEFAULT_LOW_VOLTAGE_PER_CELL_VOLTS  (3.3f) /* LiPo, konservatif */

static inline float battery_clampf(float value, float min, float max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

static inline float battery_raw_to_pin_volts(uint16_t raw)
{
    if (raw > BATTERY_ADC_RESOLUTION_COUNTS) {
        raw = BATTERY_ADC_RESOLUTION_COUNTS;
    }
    return ((float)raw / (float)BATTERY_ADC_RESOLUTION_COUNTS) * BATTERY_ADC_VREF_VOLTS;
}

void Battery_Init(BatteryConfig_t *config, uint8_t cell_count)
{
    if (config == NULL) {
        return;
    }

    config->voltage_scale = BATTERY_DEFAULT_VOLTAGE_SCALE;
    config->voltage_offset = BATTERY_DEFAULT_VOLTAGE_OFFSET;

    config->current_scale = BATTERY_DEFAULT_CURRENT_SCALE;
    config->current_offset = BATTERY_DEFAULT_CURRENT_OFFSET;

    config->filter_alpha = BATTERY_DEFAULT_FILTER_ALPHA;

    config->low_voltage_threshold_per_cell = BATTERY_DEFAULT_LOW_VOLTAGE_PER_CELL_VOLTS;
    config->cell_count = (cell_count == 0u) ? 1u : cell_count;
}

void Battery_SetVoltageCalibration(BatteryConfig_t *config, float scale, float offset)
{
    if (config == NULL || isnan(scale) || isinf(scale) || isnan(offset) || isinf(offset)) {
        return;
    }
    config->voltage_scale = scale;
    config->voltage_offset = offset;
}

void Battery_SetCurrentCalibration(BatteryConfig_t *config, float scale, float offset)
{
    if (config == NULL || isnan(scale) || isinf(scale) || isnan(offset) || isinf(offset)) {
        return;
    }
    config->current_scale = scale;
    config->current_offset = offset;
}

void Battery_SetThresholds(BatteryConfig_t *config, float low_voltage_threshold_per_cell)
{
    if (config == NULL || isnan(low_voltage_threshold_per_cell) ||
        isinf(low_voltage_threshold_per_cell)) {
        return;
    }
    config->low_voltage_threshold_per_cell = low_voltage_threshold_per_cell;
}

void Battery_Update(const BatteryConfig_t *config, BatteryReading_t *reading,
                     uint16_t raw_vbat, uint16_t raw_current, float dt_seconds)
{
    if (config == NULL || reading == NULL) {
        return;
    }

    float vbat_pin_volts = battery_raw_to_pin_volts(raw_vbat);
    float current_pin_volts = battery_raw_to_pin_volts(raw_current);

    float voltage_sample = (vbat_pin_volts * config->voltage_scale) + config->voltage_offset;
    float current_sample = (current_pin_volts * config->current_scale) + config->current_offset;

    /* Current sense secara fisik tidak boleh negatif untuk topologi unidirectional
     * (ESC brushless v1 hanya menarik arus, tidak regen) — clamp ke 0 supaya noise
     * di sekitar titik nol tidak muncul sebagai arus negatif yang membingungkan
     * di telemetry/blackbox. */
    if (current_sample < 0.0f) {
        current_sample = 0.0f;
    }

    if (!reading->valid) {
        /* Sampel pertama: langsung pakai nilai mentah, jangan filter dari 0.0
         * yang akan bikin voltage_volts kelihatan drop palsu di awal boot. */
        reading->voltage_volts = voltage_sample;
        reading->current_amps = current_sample;
        reading->consumed_mah = 0.0f;
        reading->valid = true;
    } else {
        float alpha = battery_clampf(config->filter_alpha, 0.0f, 0.999f);
        reading->voltage_volts = (alpha * reading->voltage_volts) + ((1.0f - alpha) * voltage_sample);
        reading->current_amps = (alpha * reading->current_amps) + ((1.0f - alpha) * current_sample);
    }

    if (dt_seconds > 0.0f) {
        /* mAh += A * h = A * (dt_seconds / 3600) * 1000 */
        reading->consumed_mah += reading->current_amps * (dt_seconds / 3600.0f) * 1000.0f;
    }
}

bool Battery_IsLowVoltage(const BatteryConfig_t *config, const BatteryReading_t *reading)
{
    if (config == NULL || reading == NULL || !reading->valid) {
        return false;
    }

    float threshold_total = config->low_voltage_threshold_per_cell * (float)config->cell_count;
    return reading->voltage_volts < threshold_total;
}
