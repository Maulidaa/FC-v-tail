#include "dfu.h"
#include "protocol.h"
#include "command_handler.h"
#include "error.h"
#include "stm32f4xx.h"

/* ============================================================================
 * dfu.c
 * Lihat dfu.h untuk kontrak lengkap, alasan desain, dan PERSYARATAN KERAS
 * soal urutan pemanggilan Dfu_CheckAndJumpIfRequested() di main().
 * ============================================================================ */

/* Simbol dari linker script (STM32F411CEUX_FLASH.ld):
 *   _estack = ORIGIN(RAM) + LENGTH(RAM);
 * Ini BUKAN variabel sungguhan -- jangan pernah dereference `_estack`
 * langsung, hanya ambil ALAMATNYA (&_estack) yang bernilai "satu word
 * setelah RAM terakhir yang valid", sama dengan initial stack pointer.
 * Pola ini identik dengan yang dipakai Startup/startup_stm32f411ceux.s
 * (`ldr r0, =_estack`). */
extern uint32_t _estack;

/* Berapa word di bawah puncak stack tempat flag disimpan. 16 word (64
 * byte) -- jauh lebih dari cukup mengingat titik penulisan
 * (request_bootloader_reboot(), dipanggil dari command_handler dispatch,
 * kedalaman panggilan masih dangkal) maupun titik pembacaan
 * (Dfu_CheckAndJumpIfRequested(), baris pertama main(), SP masih persis
 * di _estack) sama-sama jauh dari memakai 64 byte stack di titik itu. */
#define DFU_MAGIC_OFFSET_WORDS   16U

/* Nilai sembarang yang cukup acak supaya kecil kemungkinan cocok dengan
 * sampah stack biasa saat boot dingin (kondisi RAM saat power-on TIDAK
 * dijamin nol oleh hardware). Tidak perlu kriptografis -- ini bukan
 * mekanisme keamanan, cuma penanda "reboot ini memang diminta". */
#define DFU_MAGIC_VALUE          0xB007DF11UL

static uint32_t *dfu_magic_ptr(void)
{
    return ((uint32_t *)&_estack) - DFU_MAGIC_OFFSET_WORDS;
}

/* ---------------------------------------------------------------------------
 * Lompat ke ST System Bootloader (ROM, 0x1FFF0000 untuk STM32F411xx)
 * ------------------------------------------------------------------------- */
static void jump_to_system_bootloader(void)
{
    __disable_irq();

    /* SystemInit() (dipanggil startup assembly SEBELUM main(), lihat
     * kontrak di dfu.h) sudah menyetel SCB->VTOR = FLASH_BASE. Sebelum
     * lompat, VTOR WAJIB ditunjuk ulang ke tabel vector ROM bootloader --
     * kalau tidak, interrupt yang dipakai ROM bootloader (mis. USB OTG FS
     * untuk mode DFU) akan vector balik ke handler APLIKASI KITA yang
     * masih ada di flash, bukan handler bootloader -- kegagalan yang
     * gejalanya di sisi PC cuma "USB DFU tidak terdeteksi/enumerate",
     * sulit ditelusuri kalau tidak tahu akar masalahnya di sini.
     *
     * SYSCFG_MEMRMP (trik lama: alihkan alamat 0x00000000 ke system
     * memory) TETAP disetel untuk kompatibilitas denganasumsi sebagian
     * referensi/tooling, TAPI di target Cortex-M4 dengan VTOR eksplisit
     * seperti proyek ini, SCB->VTOR di bawah adalah langkah yang BENAR-
     * BENAR menentukan -- MEMRMP saja TIDAK CUKUP karena VTOR sudah
     * eksplisit menunjuk ke FLASH_BASE, bukan lagi mengikuti alias
     * 0x00000000. */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR; /* dummy read -- jaminan clock gating efektif,
                          * pola sama dengan system_clock.c */
    SYSCFG->MEMRMP = (SYSCFG->MEMRMP & ~SYSCFG_MEMRMP_MEM_MODE) | SYSCFG_MEMRMP_MEM_MODE_0;

    SCB->VTOR = DFU_SYSTEM_MEMORY_BASE;

    /* Belum pernah di-start (fungsi ini dipanggil SEBELUM SysTick_Init(),
     * lihat kontrak di dfu.h) -- ditulis 0 murni jaga-jaga terhadap
     * pelanggaran urutan di masa depan, bukan mematikan sesuatu yang
     * memang sedang aktif sekarang. */
    SysTick->CTRL = 0;

    __set_MSP(*(volatile uint32_t *)DFU_SYSTEM_MEMORY_BASE);

    void (*system_bootloader_entry)(void) =
        (void (*)(void))(*(volatile uint32_t *)(DFU_SYSTEM_MEMORY_BASE + 4U));

    system_bootloader_entry();

    /* Tidak pernah sampai sini kalau ROM bootloader valid. Kalau entah
     * bagaimana tetap sampai sini (mis. RDP level tinggi memblokir akses
     * -- lihat catatan RDP di protocol.md Bagian 2 keputusan #3, proyek
     * ini pakai RDP level 0 jadi seharusnya tidak relevan), diam di sini
     * daripada melanjutkan eksekusi dengan VTOR/MSP yang sudah rusak. */
    for (;;) {
        /* intentionally empty */
    }
}

static void request_bootloader_reboot(void)
{
    *dfu_magic_ptr() = DFU_MAGIC_VALUE;
    NVIC_SystemReset(); /* tidak pernah return */
}

void Dfu_CheckAndJumpIfRequested(void)
{
    uint32_t *magic = dfu_magic_ptr();

    if (*magic != DFU_MAGIC_VALUE) {
        return; /* boot normal -- lanjutkan main() seperti biasa */
    }

    /* Bersihkan SEKARANG, sebelum lompat -- kalau proses di bawah tidak
     * pernah kembali (kasus normal), ini tidak relevan lagi. Tapi kalau
     * suatu saat modul ini direvisi dan lompatnya bisa gagal lalu
     * lanjut, boot BERIKUTNYA tidak boleh diam-diam masuk DFU lagi tanpa
     * diminta ulang lewat protokol. */
    *magic = 0;

    jump_to_system_bootloader(); /* tidak pernah return */
}

/* ---------------------------------------------------------------------------
 * CMD_REBOOT_DFU (0x0601) -- lihat protocol.md Bagian 11.
 * Armed-gating SUDAH ditangani command_handler.c sebelum handler ini
 * dipanggil (daftar terpusat di sana) -- fungsi ini tidak cek armed lagi.
 * ------------------------------------------------------------------------- */
static void handle_reboot_dfu(const protocol_frame_t *frame)
{
    /* Ack dulu SEBELUM USB hilang, supaya web tahu request diterima --
     * best-effort, tidak ada jaminan byte ini benar-benar sampai ke host
     * sebelum reset terjadi beberapa instruksi kemudian (Protocol_SendFrame
     * menyerahkan byte ke BSP_USB_CDC_WriteBuf(), bukan menunggu ACK
     * layer USB) -- tetap lebih baik dicoba daripada tidak sama sekali. */
    Protocol_SendFrame(CMD_REBOOT_DFU, frame->request_id, 0, 0);

    request_bootloader_reboot();
}

/* ---------------------------------------------------------------------------
 * CMD_FLASH_HASH (0x0602) -- lihat protocol.md Bagian 11.
 * Request : [start_address: u32 LE][length: u32 LE]
 * Response: [crc32: u32 LE]
 * CRC32 dihitung lewat peripheral CRC hardware bawaan STM32F4 (bukan
 * software) -- keputusan eksplisit protocol.md Bagian 11 (threat model:
 * deteksi korupsi/erase-gagal-sebagian, bukan proteksi pihak jahat, jadi
 * kecepatan hardware lebih diprioritaskan daripada bisa pilih variasi
 * polynomial). Peripheral CRC F4 FIXED ke polynomial 0x04C11DB7, initial
 * value 0xFFFFFFFF, TANPA reflect input/output -- ini variasi CRC-32
 * BERBEDA dari CRC-32 "zlib/PKZIP" yang umum dipakai library JS (yang
 * reflected). Sisi web (`core/protocol/frame.ts` atau util verifikasi
 * DFU-nya) WAJIB mereplikasi variasi non-reflected ini persis, BUKAN
 * pakai crc32() bawaan library umum begitu saja -- item ini PERLU
 * dikomunikasikan eksplisit ke tim web, belum tercatat di protocol.md.
 * ------------------------------------------------------------------------- */

#define DFU_FLASH_HASH_REQUEST_LEN   8u   /* 2x u32 */

static uint32_t read_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

/* Keputusan (belum tercatat di protocol.md, PERLU dikonfirmasi tim):
 * start_address & length WAJIB kelipatan 4 byte (word-aligned). Alasan:
 * peripheral CRC F4 hanya menerima input 32-bit per tulis ke CRC->DR --
 * menerima panjang non-kelipatan-4 berarti harus memutuskan cara pad
 * word terakhir (nol? byte flash asli setelahnya?), dan pilihan itu
 * mengubah hasil CRC. Daripada diam-diam memilih satu skema padding yang
 * bisa beda dari asumsi tooling flashing di sisi web, permintaan yang
 * tidak word-aligned DITOLAK eksplisit -- selama tooling flashing selalu
 * menulis flash per word (praktik umum, dan cocok dengan cara linker
 * menyusun .bin/.hex), ini seharusnya tidak pernah jadi masalah nyata. */
static bool is_valid_flash_hash_range(uint32_t start_address, uint32_t length)
{
    if (length == 0u) {
        return false;
    }
    if ((start_address % 4u) != 0u || (length % 4u) != 0u) {
        return false;
    }
    if (start_address < FLASH_BASE) {
        return false;
    }

    /* Aritmatika 64-bit sengaja dipakai supaya start_address+length yang
     * hampir meluap 32-bit tidak diam-diam wrap jadi kelihatan valid. */
    uint64_t end_inclusive = (uint64_t)start_address + (uint64_t)length - 1u;
    if (end_inclusive > (uint64_t)FLASH_END) {
        return false;
    }

    return true;
}

static uint32_t compute_flash_crc32(uint32_t start_address, uint32_t length)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
    (void)RCC->AHB1ENR; /* dummy read, sinkron clock gating (pola sama
                          * dengan system_clock.c) */

    CRC->CR = CRC_CR_RESET; /* reset DR ke 0xFFFFFFFF, mulai kalkulasi baru --
                              * peripheral ini SATU instance dipakai bersama
                              * seluruh firmware; aman selama tidak ada modul
                              * lain yang menulis CRC->DR di tengah loop ini
                              * (scheduler cooperative, jadi tidak ada
                              * preemption -- lihat scheduler.h) */

    const uint32_t *words = (const uint32_t *)(uintptr_t)start_address;
    uint32_t word_count = length / 4u;

    for (uint32_t i = 0; i < word_count; i++) {
        CRC->DR = words[i];
    }

    return CRC->DR;
}

static void handle_flash_hash(const protocol_frame_t *frame)
{
    if (frame->payload_len != DFU_FLASH_HASH_REQUEST_LEN) {
        Error_Send(frame->request_id, ERROR_INVALID_PAYLOAD_LENGTH, CMD_FLASH_HASH);
        return;
    }

    uint32_t start_address = read_u32_le(&frame->payload[0]);
    uint32_t length        = read_u32_le(&frame->payload[4]);

    if (!is_valid_flash_hash_range(start_address, length)) {
        /* Belum ada kode error khusus "rentang alamat tidak valid" di
         * error.h (lihat catatan "PROPOSAL" di error.h) -- ERROR_INTERNAL
         * dipakai sebagai pinjaman sementara paling dekat maknanya
         * daripada menambah enum baru sepihak tanpa konfirmasi tim. Tandai
         * ini kalau tim mau kode error yang lebih spesifik. */
        Error_Send(frame->request_id, ERROR_INTERNAL, CMD_FLASH_HASH);
        return;
    }

    uint32_t crc32 = compute_flash_crc32(start_address, length);

    uint8_t payload[4];
    payload[0] = (uint8_t)(crc32 & 0xFFu);
    payload[1] = (uint8_t)((crc32 >> 8) & 0xFFu);
    payload[2] = (uint8_t)((crc32 >> 16) & 0xFFu);
    payload[3] = (uint8_t)((crc32 >> 24) & 0xFFu);

    Protocol_SendFrame(CMD_FLASH_HASH, frame->request_id, payload, sizeof(payload));
}

/* ---------------------------------------------------------------------------
 * API publik
 * ------------------------------------------------------------------------- */

void Dfu_Init(void)
{
    CommandHandler_Register(CMD_REBOOT_DFU, handle_reboot_dfu);
    CommandHandler_Register(CMD_FLASH_HASH, handle_flash_hash);
}
