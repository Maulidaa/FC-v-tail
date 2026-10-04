/**
 * @file    settings.h  (STUB HOST-ONLY -- host_tests/)
 * @brief   Tiruan minimal src/settings/settings.h, HANYA untuk
 *          kebutuhan kompilasi host (host_tests/test_compass.c).
 *
 * compass.c meng-include "settings.h" untuk Compass_RegisterSettings().
 * settings.h ASLI meng-include protocol.h -> stm32f4xx.h (register MCU
 * via CMSIS), jadi tidak bisa dikompilasi native di host (gcc biasa,
 * bukan arm-none-eabi-gcc). File ini mendefinisikan ULANG tipe/fungsi
 * yang benar-benar dipakai compass.c (SettingValue_t, SettingType_t,
 * SettingFieldDef_t, Settings_RegisterField) SAMA PERSIS field demi
 * field dengan src/settings/settings.h -- kalau signature/struct di
 * sana berubah, file ini WAJIB disamakan lagi atau test_compass akan
 * diam-diam menguji interface yang sudah usang.
 *
 * Karena `-I.` (direktori host_tests/) sudah lebih dulu di CFLAGS
 * daripada path manapun ke src/settings/, `#include "settings.h"` di
 * compass.c otomatis resolve ke file INI saat dikompilasi lewat
 * Makefile host_tests -- TIDAK memengaruhi build firmware sungguhan
 * (PlatformIO tidak menyentuh folder host_tests/ sama sekali).
 *
 * Settings_RegisterField() di sini cukup mencatat jumlah panggilan
 * (tidak perlu logic penuh settings.c) -- test_compass.c hanya perlu
 * memverifikasi Compass_ComputeHeadingDeg()/ApplyCalibration(), bukan
 * pipeline CMD_SETTING_* (itu di luar cakupan test ini).
 */

#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SETTING_TYPE_NUMBER  = 0,
    SETTING_TYPE_BOOL    = 1,
    SETTING_TYPE_ENUM    = 2,
    SETTING_TYPE_STRING  = 3,
    SETTING_TYPE_BITMASK = 4,
    SETTING_TYPE_GROUP   = 5,
} SettingType_t;

typedef union {
    float   number;
    int32_t enum_val;
    bool    boolean;
} SettingValue_t;

typedef struct {
    int32_t     value;
    const char *label;
} SettingEnumOption_t;

typedef SettingValue_t (*SettingGetFn_t)(void);
typedef bool (*SettingSetFn_t)(SettingValue_t value);

typedef struct {
    const char *group;
    const char *key;
    const char *label;
    SettingType_t type;
    bool has_min_max_step;
    float min;
    float max;
    float step;
    const char *unit;
    bool readonly_when_armed;
    const SettingEnumOption_t *options;
    uint8_t option_count;
    SettingGetFn_t get_fn;
    SettingSetFn_t set_fn;
} SettingFieldDef_t;

bool Settings_RegisterField(const SettingFieldDef_t *def);

#ifdef __cplusplus
}
#endif

#endif /* SETTINGS_H */
