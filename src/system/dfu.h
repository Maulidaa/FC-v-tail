/**
 * @file    dfu.h
 * @brief   Handler CMD_REBOOT_DFU & CMD_FLASH_HASH (protocol.md Bagian 11)
 *          + mekanisme lompat ke ST System Bootloader bawaan chip. Tanggung
 *          jawab: Orang 1 (Sistem & Komunikasi).
 *
 * === Alur lengkap ===
 * 1. Web kirim `CMD_REBOOT_DFU` (0x0601). Dispatcher command_handler.c
 *    SUDAH menolaknya duluan kalau armed=true (daftar gated terpusat di
 *    command_handler.c, protocol.md Bagian 11) -- handler di modul ini
 *    tidak perlu cek armed lagi.
 * 2. `Dfu_HandleRebootRequest()` (di dfu.c) membalas ack (frame kosong,
 *    request_id di-echo) SUPAYA web tahu request diterima SEBELUM USB CDC
 *    hilang, lalu menulis magic value ke RAM (lihat "Kenapa RAM, bukan
 *    variabel biasa" di bawah) dan memanggil `NVIC_SystemReset()`.
 * 3. MCU reset penuh (bukan hanya reboot software biasa -- semua
 *    peripheral kembali ke default, watchdog.c mendeteksi ini via
 *    `Watchdog_GetResetCause() == WATCHDOG_RESET_CAUSE_SOFTWARE`).
 * 4. `Dfu_CheckAndJumpIfRequested()` WAJIB jadi baris PERTAMA di `main()`
 *    (lihat kontrak lengkap di bawah). Kalau magic ditemukan, fungsi ini
 *    TIDAK PERNAH RETURN -- ia langsung mengambil alih eksekusi menuju
 *    ST System Bootloader (ROM, ditulis pabrik, tidak bisa diubah) di
 *    0x1FFF0000 (AN2606, tabel STM32F411xx). Dari titik itu, proses
 *    flashing sepenuhnya standar DfuSe/USB DFU (di luar protokol custom
 *    ini, lihat protocol.md Bagian 11) -- kode kita sudah tidak
 *    terlibat sama sekali.
 * 5. Kalau magic TIDAK ditemukan (boot normal), fungsi return seperti
 *    biasa dan `main()` lanjut ke `SystemClock_Config()` dst seperti
 *    biasa.
 *
 * === Kenapa RAM mentah, bukan variabel `static` biasa ===
 * `NVIC_SystemReset()` memicu reset MCU PENUH. Startup assembly
 * (`Startup/startup_stm32f411ceux.s`, `Reset_Handler`) menyalin ulang
 * `.data` dan MENOLKAN SELURUH `.bss` SEBELUM `main()` dipanggil -- kalau
 * flag disimpan sebagai variabel global/static biasa, nilainya sudah
 * hilang (di-nolkan) tepat sebelum sempat dibaca. Area RAM di ATAS
 * `.bss`/heap (yaitu ruang stack yang belum terpakai) TIDAK disentuh
 * proses itu -- ini teknik yang sudah dipakai luas di komunitas STM32
 * (referensi umum: stm32world.com "Jump to System Memory Bootloader",
 * AN2606). Modul ini menaruh flag beberapa word di bawah puncak stack
 * (`_estack`, simbol dari `STM32F411CEUX_FLASH.ld`), BUKAN dengan
 * hardcode alamat absolut -- supaya tetap benar kalau linker script
 * berubah.
 *
 * PERSYARATAN KERAS (baca sebelum memindah pemanggilan fungsi ini):
 * `Dfu_CheckAndJumpIfRequested()` HARUS dipanggil SEBELUM baris lain
 * apa pun di `main()` -- sebelum `Watchdog_Init()`, `SystemClock_Config()`,
 * `SysTick_Init()`, `ArmedState_Init()`, dan SEBELUM peripheral apa pun
 * diinisialisasi. Fungsi ini TIDAK melakukan RCC/SysTick/interrupt
 * teardown sendiri (lihat dfu.c) -- ia hanya AMAN karena mengasumsikan
 * tidak ada satu pun yang sempat dinyalakan/diaktifkan sebelumnya
 * (`SystemInit()` yang dipanggil startup assembly sebelum `main()` hanya
 * menyentuh `CPACR` dan `SCB->VTOR`, keduanya di-reset ulang sendiri oleh
 * fungsi ini sebelum lompat -- lihat dfu.c). Kalau syarat ini dilanggar
 * (mis. dipanggil setelah SystemClock_Config() menyalakan PLL 96MHz, atau
 * setelah interrupt USART/SPI/I2C sudah aktif), USB DFU di sisi PC bisa
 * gagal enumerate atau MCU hang -- kegagalan ini SULIT didiagnosa dari
 * gejala di PC saja, jadi jangan pernah pindahkan urutan ini tanpa alasan
 * kuat.
 */

#ifndef DFU_H
#define DFU_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Alamat System Bootloader ROM untuk STM32F411xx (AN2606 Rev 61+,
 *  bagian "STM32F411xx devices bootloader"). Sama dengan hampir seluruh
 *  keluarga F4 mainstream (F401/F405/F407/F411/dst semua 0x1FFF0000) --
 *  TAPI kalau tim ganti varian chip ke keluarga lain (F0/F1/G0/dll),
 *  WAJIB dicek ulang ke AN2606, alamat ini TIDAK universal lintas
 *  keluarga STM32. */
#define DFU_SYSTEM_MEMORY_BASE   0x1FFF0000UL

/**
 * @brief Panggil SEKALI, PALING PERTAMA di main() -- lihat kontrak
 *        lengkap & alasan di komentar file ini. TIDAK PERNAH RETURN kalau
 *        MCU baru saja reset akibat `Dfu_HandleRebootRequest()` (magic
 *        flag ditemukan) -- eksekusi berpindah permanen ke ROM bootloader.
 *        Return normal kalau ini boot biasa (magic tidak ada/tidak cocok).
 */
void Dfu_CheckAndJumpIfRequested(void);

/**
 * @brief Daftarkan handler CMD_REBOOT_DFU & CMD_FLASH_HASH ke
 *        command_handler.c. Panggil setelah `CommandHandler_Init()`,
 *        sama seperti modul lain yang mendaftarkan command sendiri.
 *        CMD_REBOOT_DFU SUDAH masuk daftar armed-gated terpusat di
 *        command_handler.c -- TIDAK perlu (dan TIDAK boleh) didaftarkan
 *        ulang di sini, cukup pastikan nama command_id-nya tetap sama
 *        (`CMD_REBOOT_DFU` dari protocol.h).
 */
void Dfu_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* DFU_H */
