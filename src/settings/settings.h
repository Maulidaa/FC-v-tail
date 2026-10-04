#ifndef SETTINGS_H
#define SETTINGS_H

#include "protocol.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * settings.h
 * Registry generik untuk grup setting yang bisa dibaca/ditulis dari web
 * lewat CMD_SETTING_SCHEMA_LIST / CMD_SETTING_GET / CMD_SETTING_SET /
 * CMD_SETTING_COMMIT (protocol.md Bagian 5, 7, 8, 9). Tanggung jawab:
 * Orang 1 (Sistem & Komunikasi) -- lihat pembagian-tugas-firmware-4-orang.md
 * Bagian 1.
 *
 * DESAIN: registration-table, POLA YANG SAMA dengan command_handler.h
 * (dog-food API sendiri, modul lain daftar diri, tabel terpusat tidak
 * perlu tahu isi modul pendaftar). Modul pemilik satu grup setting (mis.
 * `control/mixer.c` untuk grup "mixer", `output/output_map.c` untuk grup
 * "output_mapping") TIDAK perlu edit file ini -- cukup panggil
 * Settings_RegisterField() untuk tiap field leaf-nya dari fungsi init
 * modul masing-masing, dipanggil SETELAH Settings_Init(). Contoh (nanti
 * dikerjakan Orang 4 di mixer.c, sesuai keputusan
 * pembagian-tugas-firmware-4-orang.md Bagian 2 #1 -- "mixer" adalah
 * tunable setting, bukan konstanta tetap):
 *
 *   static SettingValue_t get_ruddervator_gain(void) {
 *       SettingValue_t v; v.number = s_mixer_config.ruddervator_gain; return v;
 *   }
 *   static bool set_ruddervator_gain(SettingValue_t v) {
 *       Mixer_SetRuddervatorGain(&s_mixer_config, v.number);
 *       return true; // Mixer_SetRuddervatorGain menolak diam-diam utk NaN/Inf,
 *                     // tidak ada sinyal reject eksplisit -- lihat mixer.h
 *   }
 *   static const SettingFieldDef_t kMixerGainField = {
 *       .group = "mixer", .key = "mixer.vtail_ruddervator_gain",
 *       .label = "Ruddervator Gain", .type = SETTING_TYPE_NUMBER,
 *       .has_min_max_step = true, .min = 0.0f, .max = 2.0f, .step = 0.05f,
 *       .unit = NULL, .readonly_when_armed = true,
 *       .get_fn = get_ruddervator_gain, .set_fn = set_ruddervator_gain,
 *   };
 *   Settings_RegisterField(&kMixerGainField);
 *
 * SCHEMA TREE -- KETERBATASAN SENGAJA (v1): protocol.md Bagian 7
 * mendefinisikan `type=group` sebagai rekursif (grup di dalam grup).
 * Implementasi ini HANYA mendukung DUA LEVEL: top-level berupa daftar
 * grup (dikumpulkan otomatis dari field `group` di SettingFieldDef_t,
 * urutan kemunculan pertama) + field ungrouped (group==NULL/"") sebagai
 * leaf langsung di top-level, dan tiap grup berisi leaf field secara
 * FLAT (tidak ada sub-grup di dalam grup). Ini cukup untuk kebutuhan
 * v1 yang diketahui (protocol.md Bagian 8 `output_mapping` dan Bagian 9
 * `mixer`, keduanya flat). Kalau nanti ada kebutuhan nested-group
 * sungguhan, `group` perlu diubah dari `const char *` tunggal jadi path
 * bertingkat dan build_top_nodes()/encode_group() di settings.c perlu
 * digeneralisasi -- BELUM dikerjakan di sini, dicatat sebagai TODO.
 *
 * BACKEND PENYIMPANAN PERMANEN -- ITEM TERBUKA: Settings_Commit() (dipicu
 * CMD_SETTING_COMMIT) perlu menulis nilai saat ini ke storage permanen
 * supaya bertahan lintas reboot. Modul ini TIDAK memutuskan media
 * penyimpanannya (sektor internal flash STM32F411 vs reuse external
 * SPI flash W25Q64 milik blackbox Orang 4 dengan partisi terpisah --
 * belum dibahas tim di dokumen manapun, BUKAN bagian dari
 * firmware-architecture-stm32f411.md Bagian 5 yang hanya membahas format
 * record blackbox, bukan config storage). Sebagai jembatan, modul ini
 * menyediakan hook __attribute__((weak)) Settings_FlashSave()/
 * Settings_FlashLoad() (lihat settings.c) dengan default yang GAGAL
 * eksplisit (bukan pura-pura sukses) -- WAJIB di-override modul storage
 * sungguhan begitu keputusan medianya diambil, supaya CMD_SETTING_COMMIT
 * tidak diam-diam berbohong ke web bahwa nilai sudah permanen padahal
 * cuma di RAM.
 * ============================================================================ */

#define SETTINGS_MAX_KEY_LEN     31u   /* panjang string key maksimum (tanpa null terminator, key_len wire tetap u8) */
#define SETTINGS_MAX_LABEL_LEN   31u
#define SETTINGS_MAX_UNIT_LEN    7u
#define SETTINGS_MAX_FIELDS      32u   /* naikkan kalau field terdaftar makin banyak (mixer:2 + output_mapping diperkirakan <=10 utk v1) */
#define SETTINGS_MAX_OPTIONS     8u    /* per field enum/bitmask */

/**
 * @brief Tipe wire sesuai protocol.md Bagian 7 (`type: u8`).
 *        SETTING_TYPE_GROUP TIDAK pernah dipakai modul pendaftar --
 *        node grup dibentuk OTOMATIS oleh settings.c dari field `group`
 *        di SettingFieldDef_t (lihat catatan header di atas).
 */
typedef enum {
    SETTING_TYPE_NUMBER  = 0,
    SETTING_TYPE_BOOL    = 1,
    SETTING_TYPE_ENUM    = 2,
    SETTING_TYPE_STRING  = 3, /* BELUM didukung get/set (lihat SettingValue_t) -- tidak ada kebutuhan v1 (output_mapping/mixer semua number/enum) */
    SETTING_TYPE_BITMASK = 4,
    SETTING_TYPE_GROUP   = 5, /* internal settings.c saja */
} SettingType_t;

/**
 * @brief Nilai generik satu field. String SENGAJA tidak disediakan di
 *        union ini (lihat SETTING_TYPE_STRING di atas) -- kalau nanti
 *        ada field string sungguhan, union ini perlu union tambahan
 *        {char buf[N]; uint8_t len;} dan seluruh jalur encode/decode di
 *        settings.c perlu disesuaikan. TIDAK dikerjakan di v1 ini.
 */
typedef union {
    float   number;     /* SETTING_TYPE_NUMBER */
    int32_t enum_val;   /* SETTING_TYPE_ENUM & SETTING_TYPE_BITMASK -- bitmask disimpan sebagai bit pattern di representasi dua's-complement yang sama */
    bool    boolean;    /* SETTING_TYPE_BOOL */
} SettingValue_t;

/**
 * @brief Satu opsi enum/bitmask (protocol.md Bagian 7:
 *        `(value: i32)(label_len: u8)(label: bytes)`).
 */
typedef struct {
    int32_t     value;
    const char *label;
} SettingEnumOption_t;

/** @brief Baca nilai TERKINI field dari modul pemilik (mis. baca langsung
 *         dari MixerConfig_t yang sedang dipakai scheduler). Dipanggil
 *         settings.c untuk CMD_SETTING_GET dan untuk readback setelah
 *         CMD_SETTING_SET berhasil (nilai bisa saja di-clamp modul
 *         pemilik, mis. Mixer_SetAileronDifferential -- readback
 *         memastikan web tahu nilai SEBENARNYA yang berlaku, bukan
 *         nilai mentah yang dikirim). */
typedef SettingValue_t (*SettingGetFn_t)(void);

/**
 * @brief Terapkan nilai baru ke modul pemilik. Validasi range
 *        (min/max/step) dan armed-gating (readonly_when_armed) SUDAH
 *        dilakukan settings.c SEBELUM fungsi ini dipanggil -- fungsi
 *        ini hanya perlu menolak kondisi yang tidak bisa diekspresikan
 *        lewat schema (mis. NaN/Inf pada float, lihat
 *        Mixer_SetRuddervatorGain).
 * @return true kalau diterima & diterapkan, false kalau ditolak modul
 *         pemilik -- settings.c akan membalas ERROR_SETTING_VALUE_OUT_OF_RANGE
 *         ke web pada kasus ini (reuse kode error yang ada, karena
 *         error.h belum punya kode khusus utk "ditolak modul pemilik").
 */
typedef bool (*SettingSetFn_t)(SettingValue_t value);

/**
 * @brief Definisi satu field leaf. Modul pendaftar (mixer.c,
 *        output_map.c, dst) membuat instance `static const` dari struct
 *        ini per field dan mendaftarkannya lewat Settings_RegisterField().
 *        Pointer yang disimpan (group/key/label/unit/options) HARUS
 *        tetap hidup selama program berjalan (pakai string literal atau
 *        `static const`, JANGAN buffer stack sementara) -- settings.c
 *        menyimpan pointer-nya langsung, tidak menyalin string.
 */
typedef struct {
    const char *group;              /**< nama grup top-level (mis. "mixer", "output_mapping"), atau NULL/"" untuk field ungrouped langsung di top-level */
    const char *key;                /**< key penuh unik dipakai GET/SET, mis. "mixer.vtail_ruddervator_gain" (protocol.md Bagian 8 & 9) */
    const char *label;              /**< label tampilan utk web */
    SettingType_t type;             /**< salah satu dari NUMBER/BOOL/ENUM/BITMASK (bukan STRING/GROUP, lihat catatan tipe) */
    bool has_min_max_step;          /**< kalau true, min/max/step dikirim di schema DAN dipakai settings.c utk validasi SETTING_SET (hanya relevan utk NUMBER) */
    float min;
    float max;
    float step;
    const char *unit;               /**< boleh NULL (dikirim sbg unit_len=0) */
    bool readonly_when_armed;       /**< wajib true utk seluruh grup output_mapping & mixer (protocol.md Bagian 7 & 8) */
    const SettingEnumOption_t *options; /**< hanya dipakai type ENUM/BITMASK, NULL kalau tidak relevan */
    uint8_t option_count;
    SettingGetFn_t get_fn;          /**< tidak boleh NULL */
    SettingSetFn_t set_fn;          /**< tidak boleh NULL */
} SettingFieldDef_t;

/**
 * @brief Init registry (reset ke kosong) dan daftarkan handler
 *        CMD_SETTING_SCHEMA_LIST / CMD_SETTING_GET / CMD_SETTING_SET /
 *        CMD_SETTING_COMMIT ke command_handler.c. WAJIB dipanggil
 *        SETELAH CommandHandler_Init(), SEBELUM modul lain memanggil
 *        Settings_RegisterField() (pola sama seperti
 *        CommandHandler_Init()/CommandHandler_Register()).
 *
 * Berbeda dengan command_handler.c, urutan Settings_RegisterField() dari
 * modul lain TIDAK harus selesai sebelum command pertama diterima --
 * schema tree (grouping, pagination) dibangun ULANG setiap kali
 * CMD_SETTING_SCHEMA_LIST diminta (bukan di-cache saat Init), jadi
 * field yang terdaftar belakangan tetap muncul di request berikutnya.
 * Field yang didaftarkan SETELAH suatu CMD_SETTING_SCHEMA_LIST/GET/SET
 * sempat diproses TIDAK retroaktif muncul di respons yang sudah
 * terkirim, tapi ini bukan masalah nyata selama semua modul mendaftar
 * di fase init boot sebelum scheduler mulai memproses command dari USB.
 */
void Settings_Init(void);

/**
 * @brief Daftarkan satu field leaf. Menyalin ISI struct (bukan hanya
 *        pointer ke `def`) ke slot internal -- `def` boleh berupa
 *        variabel stack sementara, TAPI string yang DITUNJUK field-field
 *        di dalamnya (key/label/unit/group/options) tetap harus statis
 *        (lihat catatan di SettingFieldDef_t).
 * @return true kalau berhasil, false kalau `def`/get_fn/set_fn/key NULL,
 *         key sudah terdaftar sebelumnya (cegah dua modul rebutan key
 *         yang sama -- gagal keras saat init, konsisten dengan
 *         CommandHandler_Register()), atau tabel penuh
 *         (SETTINGS_MAX_FIELDS).
 */
bool Settings_RegisterField(const SettingFieldDef_t *def);

#ifdef __cplusplus
}
#endif

#endif /* SETTINGS_H */
