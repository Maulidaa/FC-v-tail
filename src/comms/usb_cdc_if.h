#ifndef USB_CDC_IF_H
#define USB_CDC_IF_H

#include "stm32f4xx.h"   /* CMSIS device header, target STM32F411xE */
#include <stdint.h>

/* ============================================================================
 * usb_cdc_if.h
 * USB OTG FS device mode, kelas CDC-ACM (virtual COM port) -- transport v1
 * FC<->web sesuai protocol.md Bagian 1 ("Transport v1 murni USB CDC").
 *
 * Pin PA11 (D-) / PA12 (D+) sudah dikonfigurasi AF10 oleh BSP_PinMap_Init().
 *
 * CATATAN PENTING soal PA9 (lihat juga bsp_pinmap.h):
 *   STM32F411 punya fungsi dedicated OTG_FS_VBUS di PA9 untuk VBUS SENSING
 *   hardware (deteksi USB benar2 tercolok via pembagi tegangan eksternal).
 *   Modul ini SENGAJA memakai mode NOVBUSSENS (asumsi VBUS selalu ada,
 *   tidak sensing hardware) -- pola umum di board dev seperti Blackpill
 *   yang tidak memasang pembagi tegangan VBUS ke PA9. Konsekuensinya:
 *   PA9 TETAP BEBAS dipakai GPIO/fungsi lain (tidak bentrok dengan
 *   keputusan RX-only CRSF yang sudah membebaskan PA9 sebagai spare).
 *   Kalau suatu saat tim memutuskan pakai VBUS sensing hardware asli,
 *   PA9 harus direalokasi ulang dan keputusan itu perlu dikoordinasikan
 *   ke Orang 3 (karena PA9 sudah dijanjikan spare untuk CRSF).
 *
 * VID:PID (checklist Bagian K, MASIH BELUM FINAL):
 *   Dipakai placeholder VID:PID milik STMicroelectronics yang lazim
 *   dipakai contoh CDC/VCP (0x0483:0x5740) supaya device BISA dites
 *   end-to-end sekarang (driver bawaan usbser.sys/CDC-ACM di Windows/
 *   Linux/macOS akan mengenali tanpa .inf custom). GANTI USB_VID/USB_PID
 *   di bawah begitu tim memutuskan nilai final -- lihat protocol.md
 *   Bagian 12 & checklist Bagian K.
 *
 * DESAIN: RX interrupt-driven ring buffer (sama pola dengan bsp_uart.c).
 * TX: BSP_USB_CDC_WriteBuf() blocking -- menunggu transfer bulk IN selesai
 * sebelum return, cukup untuk command/response protocol.c yang memang
 * request-response (bukan streaming volume tinggi).
 *
 * PERINGATAN KOMPLEKSITAS: modul ini implementasi USB device dari
 * register langsung (bukan ST USB Device Library/HAL). Enumerasi USB
 * terkenal sensitif terhadap timing & detail kecil -- WAJIB diuji di
 * hardware asli (cek Device Manager/dmesg saat pertama colok, idealnya
 * juga dengan USB protocol analyzer kalau enumerasi gagal) sebelum
 * dianggap selesai. Kalau debugging register-level ini terlalu makan
 * waktu tim, opsi realistis: ganti modul ini dengan ST USB Device
 * Middleware (STM32Cube, lisensi permisif) tanpa mengubah API publik
 * di header ini -- protocol.c/command_handler.c tidak perlu tahu bedanya.
 * ============================================================================ */

#define USB_VID   0x0483u  /* TODO: ganti sesuai keputusan final tim */
#define USB_PID   0x5740u  /* TODO: ganti sesuai keputusan final tim */

#define USB_CDC_RX_BUF_SIZE   256u  /* power-of-2, sama pola dengan bsp_uart.c */
#define USB_CDC_BULK_MPS      64u   /* max packet size endpoint bulk full-speed */

/**
 * @brief Init lengkap USB OTG FS device mode + kelas CDC-ACM: core reset,
 *        FIFO sizing, deskriptor device/config/string, endpoint 0/1/2,
 *        lalu present pull-up (soft-connect) supaya host mulai enumerasi.
 *        Panggil sekali saat boot, setelah BSP_PinMap_Init() dan
 *        SystemClock_Config() (USB butuh clock 48MHz presisi dari PLLQ).
 */
void BSP_USB_CDC_Init(void);

/**
 * @brief Status koneksi -- 1 kalau host sudah menuntaskan enumerasi
 *        (SET_CONFIGURATION diterima), 0 kalau belum (mis. kabel belum
 *        dicolok, atau masih proses enumerasi). Command_handler/protocol.c
 *        bisa memakai ini untuk tahu kapan mulai kirim data valid.
 */
int BSP_USB_CDC_IsConfigured(void);

/**
 * @brief 1 kalau enumerasi selesai DAN host meng-assert DTR (ada program
 *        yang membuka port COM), 0 selain itu. Dipakai push telemetri
 *        (Comms #4) sebagai gate: BSP_USB_CDC_WriteBuf() menunggu
 *        transfer IN selesai TANPA timeout, dan kalau tidak ada host yang
 *        membaca (port tidak dibuka), endpoint IN terus di-NAK -> loop
 *        tunggu tidak pernah selesai -> IWDG (~500 ms) mereset MCU
 *        berulang-ulang. Balasan atas request tidak terkena karena host
 *        yang meminta pasti sedang membaca; push tanpa request beda.
 *        BELUM diverifikasi di hardware/OS (Web Serial diharapkan
 *        meng-assert DTR saat open() -- cek saat uji terhubung).
 */
int BSP_USB_CDC_IsHostOpen(void);

/** @brief Jumlah byte yang siap dibaca dari ring buffer RX. */
uint16_t BSP_USB_CDC_Available(void);

/**
 * @brief Ambil 1 byte dari ring buffer RX (non-blocking).
 * @param out tujuan byte yang diambil
 * @return 1 kalau ada byte diambil, 0 kalau buffer kosong
 */
int BSP_USB_CDC_ReadByte(uint8_t *out);

/**
 * @brief Kirim N byte ke host lewat endpoint bulk IN, dipecah otomatis
 *        jadi beberapa paket kalau len > USB_CDC_BULK_MPS. Blocking --
 *        menunggu setiap paket selesai ditransfer (XFRC) sebelum
 *        mengirim paket berikutnya / return ke caller.
 * @param data buffer yang dikirim
 * @param len jumlah byte
 * @return 1 kalau berhasil terkirim semua, 0 kalau device belum
 *         configured (host belum enumerasi) -- caller (protocol.c)
 *         sebaiknya buang frame kalau ini terjadi, bukan retry paksa.
 */
int BSP_USB_CDC_WriteBuf(const uint8_t *data, uint16_t len);

#endif /* USB_CDC_IF_H */
