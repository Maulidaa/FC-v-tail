#include "command_handler.h"
#include "error.h"
#include <stdio.h>
#include <string.h>

/* ============================================================================
 * command_handler.c
 * Lihat command_handler.h untuk filosofi desain (registration-table,
 * armed-gating terpusat, hook __attribute__((weak) untuk integrasi
 * modul lain yang belum tentu sudah ada).
 * ============================================================================ */

#define COMMAND_HANDLER_MAX_ENTRIES   24u

typedef struct {
    uint16_t             command_id;
    command_handler_fn_t fn;
} command_handler_entry_t;

static command_handler_entry_t s_handlers[COMMAND_HANDLER_MAX_ENTRIES];
static uint8_t                 s_handler_count;

/* ---------------------------------------------------------------------------
 * Daftar terpusat command armed-gated (protocol.md Bagian 5,
 * pembagian-tugas-firmware-4-orang.md Bag.2 keputusan #2, checklist
 * Bagian D). SATU-SATUNYA tempat kebijakan ini didefinisikan -- kalau
 * suatu command perlu ditambah/dihapus dari daftar gated, ubah di sini
 * saja, tidak perlu menyentuh modul registrar mana pun.
 * ------------------------------------------------------------------------- */
static const uint16_t s_armed_gated_commands[] = {
    CMD_MOTOR_TEST,
    CMD_SERVO_TEST,
    CMD_REBOOT_DFU,
};
#define ARMED_GATED_COUNT (sizeof(s_armed_gated_commands) / sizeof(s_armed_gated_commands[0]))

static int is_armed_gated(uint16_t command_id)
{
    for (uint32_t i = 0; i < ARMED_GATED_COUNT; i++) {
        if (s_armed_gated_commands[i] == command_id) {
            return 1;
        }
    }
    return 0;
}

/* ---------------------------------------------------------------------------
 * Hook integrasi (weak default) -- lihat command_handler.h.
 *
 * Armed_IsArmed() default RETURN 1 (dianggap armed) SENGAJA, bukan 0.
 * Alasan: ini gerbang keselamatan untuk command yang menggerakkan
 * motor/servo fisik atau reboot ke DFU. Kalau modul flight-state asli
 * (Orang 3/4) belum ada/belum di-link, fail-safe yang benar adalah
 * MENOLAK command berbahaya ini secara default, bukan mengizinkannya
 * begitu saja karena "belum ada yang bilang armed". Konsekuensinya:
 * MOTOR_TEST/SERVO_TEST/REBOOT_DFU tidak akan bisa dites lewat protokol
 * sampai modul flight-state sungguhan meng-override fungsi ini dengan
 * definisi non-weak bernama sama (linker otomatis pakai versi kuat).
 * ------------------------------------------------------------------------- */
__attribute__((weak)) int Armed_IsArmed(void)
{
    return 1;
}

__attribute__((weak)) uint16_t Battery_GetVoltageMillivolts(void)
{
    return 0u;
}

__attribute__((weak)) uint8_t GPS_GetFixType(void)
{
    return 0u;
}

__attribute__((weak)) uint8_t ControlMode_GetActive(void)
{
    /* Default 0 (stabilize) -- fail-safe yang masuk akal: kalau modul
     * yang benar-benar melacak passthrough belum di-link/di-override,
     * lebih baik CMD_GET_STATUS melaporkan "stabilize" (nilai lama,
     * sebelum field ini ada) daripada mengarang status passthrough yang
     * tidak pernah dikonfirmasi.
     *
     * STATUS (Comms #1 -- lihat catatan Opsi A di dokumentasi
     * handle_get_status() di bawah): hook ini SEMENTARA TIDAK dipakai
     * oleh command mana pun -- payload CMD_GET_STATUS diperbaiki supaya
     * cocok dengan parser web yang sudah berjalan (faas-configurator),
     * dan parser itu tidak punya slot untuk byte control_mode. Hook ini
     * SENGAJA dibiarkan ada (bukan dihapus) supaya begitu tim menambah
     * command/field khusus untuk mode kontrol, sumber nilainya sudah
     * siap tinggal disambungkan -- lihat handle_get_status() untuk
     * detail keputusannya. */
    return 0u;
}

/* ---------------------------------------------------------------------------
 * Handler bawaan: CMD_GET_STATUS
 *
 * BUG AKTIF DITUTUP (Comms #1): layout payload di bawah SEBELUMNYA
 * (9 byte, ditandai "PROPOSAL") tidak cocok sama sekali dengan parser
 * yang SUDAH BERJALAN di sisi web (`faas-configurator`,
 * `src/core/commands/statusCommands.ts`, sudah diverifikasi langsung
 * dari repo tersebut) -- field kedua saja sudah beda arti total
 * (firmware kirim `protocol_version` sebagai 1 byte, web membaca posisi
 * itu sebagai panjang-string `firmwareVersion`), jadi seluruh field
 * setelahnya ikut salah baca. Ini BUKAN proposal lagi -- layout di
 * bawah adalah hasil PENYESUAIAN firmware ke kontrak yang sudah
 * benar-benar dipakai web (lebih murah mengubah satu fungsi di
 * firmware daripada mengubah parser + semua pemanggilnya di web).
 * `statusCommands.ts` sisi web adalah sumber kebenaran; protocol.md
 * BELUM diperbarui untuk mencerminkan wire-format ini secara eksplisit
 * (masih item terbuka, sama sifatnya dengan catatan lama "PROPOSAL"
 * yang digantikan catatan ini).
 *
 * Payload (final, disinkronkan ke statusCommands.ts -- panjang variabel,
 * min 8 byte / maks 8+15=23 byte tergantung panjang string versi):
 *   [0]       armed             u8   (0/1)
 *   [1]       fw_version_len    u8   (panjang string di bawah, TANPA null
 *                                     terminator -- encoding "lstring"
 *                                     yang sama dipakai grup setting,
 *                                     lihat protocol.md Bagian 7)
 *   [2..2+N)  fw_version        char[N] ASCII "MAJOR.MINOR.PATCH", N =
 *                                     fw_version_len byte sebelumnya
 *   [2+N]     protocol_version  u8
 *   [3+N..6+N] battery_voltage  f32 little-endian, VOLT (bukan mV --
 *                                     beda dari layout lama; konversi
 *                                     dari Battery_GetVoltageMillivolts()
 *                                     dilakukan di sini)
 *   [7+N]     gps_fix_type      u8
 *
 * f32 di-serialize lewat memcpy langsung (bukan helper portable) --
 * konsisten dengan pola yang sudah dipakai settings.c untuk field
 * min/max/step (lihat settings.c, sekitar encode_field()): proyek ini
 * sudah build untuk Cortex-M4F little-endian, dan host build (kalau
 * ada, mis. utk host test) yang menjalankan test di bawah juga
 * diasumsikan little-endian -- tidak ada dua cara berbeda menulis float
 * ke wire di file yang berbeda.
 *
 * KEPUTUSAN field `control_mode` yang HILANG dari layout lama (dulu
 * byte [8], stabilize/passthrough): diambil OPSI A (bukan Opsi B) --
 * byte ini DIHAPUS dari CMD_GET_STATUS, TIDAK dipindah ke byte
 * tambahan. Alasan: Opsi B butuh PR di DUA repo sekaligus (firmware +
 * faas-configurator) supaya `src/shared/types/device.ts` dan
 * `statusCommands.ts` sisi web tahu cara membaca byte tambahan itu --
 * di luar cakupan/akses task ini (hanya repo firmware yang tersedia di
 * sini). Informasi `control_mode` TIDAK hilang diam-diam: sumbernya
 * (`ControlMode_GetActive()`, lihat definisi weak di atas) sengaja
 * dibiarkan tetap ada, sekadar untuk sementara TIDAK dipanggil dari
 * mana pun -- sampai tim menambah command/field khusus untuk mode
 * kontrol dan memperbarui kontrak `DeviceStatus` di kedua sisi untuk
 * menampungnya (baru saat itu Opsi B relevan lagi). Dicatat di sini
 * supaya keputusan ini tidak hilang tanpa jejak, sesuai permintaan
 * task Comms #1.
 * ------------------------------------------------------------------------- */
#define PROTOCOL_VERSION   1u
#define FW_VERSION_MAJOR   0u
#define FW_VERSION_MINOR   1u
#define FW_VERSION_PATCH   0u  /* TODO: sinkronkan ke tag git / build system begitu ada */

static void handle_get_status(const protocol_frame_t *frame)
{
    uint16_t battery_mv    = Battery_GetVoltageMillivolts();
    float    battery_volts = (float)battery_mv / 1000.0f;

    /* firmwareVersion sebagai string "MAJOR.MINOR.PATCH" -- lstring
     * (u8 length prefix + isi, tanpa null terminator), pola encoding
     * yang sama dipakai grup setting (settings.c). Buffer 16 byte lebih
     * dari cukup untuk "255.255.255" (11 char) + sedikit slack. */
    char fw_str[16];
    int  fw_len = snprintf(fw_str, sizeof(fw_str), "%u.%u.%u",
                            FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH);
    if (fw_len < 0) {
        fw_len = 0;
    }
    if ((size_t)fw_len > sizeof(fw_str) - 1) {
        fw_len = (int)sizeof(fw_str) - 1;
    }

    uint8_t payload[1 + 1 + 16 + 1 + 4 + 1]; /* armed + len + str(maks) + protoVer + f32 + fix */
    size_t  idx = 0;
    payload[idx++] = (uint8_t)Armed_IsArmed();
    payload[idx++] = (uint8_t)fw_len;
    for (int i = 0; i < fw_len; i++) {
        payload[idx++] = (uint8_t)fw_str[i];
    }
    payload[idx++] = PROTOCOL_VERSION;
    memcpy(&payload[idx], &battery_volts, sizeof(float));
    idx += sizeof(float);
    payload[idx++] = GPS_GetFixType();

    Protocol_SendFrame(CMD_GET_STATUS, frame->request_id, payload, (uint8_t)idx);
}

/* ---------------------------------------------------------------------------
 * Dispatcher
 * ------------------------------------------------------------------------- */

static command_handler_entry_t *find_entry(uint16_t command_id)
{
    for (uint8_t i = 0; i < s_handler_count; i++) {
        if (s_handlers[i].command_id == command_id) {
            return &s_handlers[i];
        }
    }
    return 0;
}

static void dispatch(const protocol_frame_t *frame)
{
    command_handler_entry_t *entry = find_entry(frame->command_id);

    if (entry == 0) {
        Error_Send(frame->request_id, ERROR_UNKNOWN_COMMAND, frame->command_id);
        return;
    }

    if (is_armed_gated(frame->command_id) && Armed_IsArmed()) {
        Error_Send(frame->request_id, ERROR_ARMED_REJECTED, frame->command_id);
        return;
    }

    entry->fn(frame);
}

/* ---------------------------------------------------------------------------
 * API publik
 * ------------------------------------------------------------------------- */

void CommandHandler_Init(void)
{
    s_handler_count = 0;

    /* dog-food registration API sendiri untuk handler bawaan modul ini */
    CommandHandler_Register(CMD_GET_STATUS, handle_get_status);

    Protocol_SetFrameHandler(dispatch);
}

int CommandHandler_Register(uint16_t command_id, command_handler_fn_t fn)
{
    if (fn == 0) {
        return 0;
    }
    if (find_entry(command_id) != 0) {
        return 0; /* sudah terdaftar -- cegah dua modul rebutan command_id sama */
    }
    if (s_handler_count >= COMMAND_HANDLER_MAX_ENTRIES) {
        return 0; /* tabel penuh -- naikkan COMMAND_HANDLER_MAX_ENTRIES kalau perlu */
    }

    s_handlers[s_handler_count].command_id = command_id;
    s_handlers[s_handler_count].fn = fn;
    s_handler_count++;
    return 1;
}
