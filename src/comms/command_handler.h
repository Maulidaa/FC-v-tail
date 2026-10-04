#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include "protocol.h"
#include <stdint.h>

/* ============================================================================
 * command_handler.h
 * Dispatcher command_id -> handler, plus kebijakan armed-state gating
 * TERPUSAT (checklist Bagian D: "audit ulang: satu kebijakan konsisten
 * untuk semua command *_TEST").
 *
 * DESAIN: registration-table, bukan switch-case monolitik. Modul lain
 * (Orang 2/3/4) TIDAK perlu edit file ini untuk menambah command mereka
 * sendiri -- cukup panggil CommandHandler_Register() dari init module
 * masing-masing, mis.:
 *
 *   // di output_map.c (Orang 4), dipanggil dari main() setelah
 *   // CommandHandler_Init():
 *   CommandHandler_Register(CMD_MOTOR_TEST, OutputMap_HandleMotorTest);
 *   CommandHandler_Register(CMD_SERVO_TEST, OutputMap_HandleServoTest);
 *
 * Status armed/tidak-armed TIDAK dicatat/di-set oleh command_handler.c --
 * itu tanggung jawab Orang 3/4 (state penerbangan, biasanya dari posisi
 * switch RC lewat parser CRSF + failsafe). command_handler.c hanya
 * MEMBACA status itu lewat Armed_IsArmed() untuk keperluan gating.
 * command_handler.c menyediakan definisi __attribute__((weak)) untuk
 * Armed_IsArmed() supaya firmware tetap link sebelum modul flight-state
 * yang sebenarnya ada -- WAJIB di-override modul asli sebelum command
 * armed-gated bisa diuji dengan makna yang benar (lihat command_handler.c).
 *
 * Kebijakan "command mana yang armed-gated" (protocol.md Bagian 5,
 * pembagian-tugas Bag.2 #2, checklist Bagian D) TIDAK diserahkan ke
 * masing-masing registrar (rawan lupa/salah tandai) -- daftarnya
 * di-hardcode sebagai satu sumber kebenaran di command_handler.c:
 * CMD_MOTOR_TEST, CMD_SERVO_TEST, CMD_REBOOT_DFU.
 * ============================================================================ */

/**
 * @brief Handler satu command. Dipanggil dispatcher SETELAH command_id
 *        dikenali (terdaftar) DAN lolos pengecekan armed-gating (kalau
 *        command_id ybs termasuk daftar gated). Handler bertanggung
 *        jawab membalas lewat Protocol_SendFrame() sendiri (echo
 *        frame->request_id) kalau command butuh balasan.
 */
typedef void (*command_handler_fn_t)(const protocol_frame_t *frame);

/**
 * @brief Init command_handler: reset tabel registrasi, daftarkan handler
 *        bawaan modul ini sendiri (CMD_GET_STATUS), lalu daftarkan diri
 *        sebagai frame handler ke protocol.c lewat
 *        Protocol_SetFrameHandler(). WAJIB dipanggil SETELAH
 *        Protocol_Init(), SEBELUM modul lain memanggil
 *        CommandHandler_Register().
 */
void CommandHandler_Init(void);

/**
 * @brief Daftarkan handler untuk satu command_id. Armed-gating TIDAK
 *        ditentukan di sini -- dispatcher otomatis cek command_id
 *        terhadap daftar gated terpusat (lihat catatan di atas).
 * @param command_id salah satu CMD_* dari protocol.h
 * @param fn fungsi yang dipanggil saat frame dengan command_id ini
 *        diterima (dan lolos gating kalau relevan)
 * @return 1 kalau berhasil, 0 kalau tabel penuh ATAU command_id ini
 *         sudah terdaftar sebelumnya (mencegah dua modul diam-diam
 *         rebutan satu command_id yang sama -- kesalahan integrasi
 *         yang lebih baik gagal keras saat init daripada silent bug).
 */
int CommandHandler_Register(uint16_t command_id, command_handler_fn_t fn);

/**
 * @brief Hook status armed, DIBACA dispatcher untuk gating, DI-ISI
 *        modul flight-state (Orang 3/4). command_handler.c menyediakan
 *        definisi __attribute__((weak)) default -- lihat command_handler.c
 *        untuk detail nilai default & alasan pemilihannya.
 * @return 1 kalau armed, 0 kalau tidak
 */
int Armed_IsArmed(void);

/**
 * @brief Hook tegangan baterai (mV), dipakai handler bawaan
 *        CMD_GET_STATUS. Weak default 0 -- DI-ISI modul power_monitor.c
 *        (Orang 4).
 */
uint16_t Battery_GetVoltageMillivolts(void);

/**
 * @brief Hook GPS fix type (0=no fix, sesuai skala yang dipakai parser
 *        UBX Orang 2 -- BELUM ada enum resmi di dokumen manapun yang
 *        saya lihat, jadi nilai persis selain 0 perlu disepakati saat
 *        modul GPS ada). Weak default 0 -- DI-ISI modul gps_ubx.c
 *        (Orang 2).
 */
uint8_t GPS_GetFixType(void);

/**
 * @brief Hook mode kontrol aktif (0=stabilize, 1=passthrough/full-manual
 *        — lihat "full manual / passthrough" di main.c/stabilize.c).
 *        Weak default 0 (stabilize) -- DI-ISI modul yang menyimpan state
 *        passthrough (saat ini `s_fc.passthrough_active` di main.c;
 *        BELUM di-override di commit ini, sama seperti Armed_IsArmed()
 *        yang juga belum di-migrasi main.c-nya -- lihat TODO INTEGRASI
 *        di armed_state.h untuk pola yang sama persis).
 *
 *        STATUS (Comms #1): hook ini SEMENTARA TIDAK dipanggil oleh
 *        command_handler.c mana pun -- CMD_GET_STATUS diperbaiki supaya
 *        cocok dengan parser web yang sudah berjalan (faas-configurator),
 *        yang tidak punya slot untuk field ini (Opsi A, lihat catatan
 *        lengkap di handle_get_status() dalam command_handler.c). Hook
 *        ini tetap dipertahankan (bukan dihapus) sebagai sumber nilai
 *        yang siap dipakai begitu ada command/field khusus untuk mode
 *        kontrol.
 * @return 0 = stabilize, 1 = passthrough/full-manual.
 */
uint8_t ControlMode_GetActive(void);

#endif /* COMMAND_HANDLER_H */
