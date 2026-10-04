/**
 * @file    settings_stub.c  (STUB HOST-ONLY -- host_tests/)
 * @brief   Implementasi kosong Settings_RegisterField() untuk keperluan
 *          link test_compass.c. Lihat settings.h (stub) di folder ini
 *          untuk alasan lengkap kenapa settings.h ASLI (src/settings/)
 *          tidak bisa dipakai langsung di host test.
 *
 * Tidak perlu logic apa pun di sini -- test_compass.c hanya memanggil
 * Compass_RegisterSettings() untuk memastikan TIDAK crash saat
 * dipanggil (lihat komentar Compass_RegisterSettings() di compass.h
 * soal Settings_Init()/Settings_FlashLoad() yang belum disambungkan di
 * main.c) -- bukan untuk menguji pipeline CMD_SETTING_* itu sendiri.
 */
#include "settings.h"

bool Settings_RegisterField(const SettingFieldDef_t *def)
{
    (void)def;
    return true;
}
