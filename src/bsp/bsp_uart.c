#include "bsp_uart.h"
#include "bsp_pinmap.h"

/* ============================================================================
 * bsp_uart.c
 * Implementasi bare-metal (register langsung, tanpa HAL) untuk USART1
 * (iBus RX-only) dan USART2 (GPS, TX+RX). RX interrupt-driven dengan ring
 * buffer sederhana per USART; TX blocking (cukup untuk command konfigurasi
 * GPS yang jarang dan tidak time-critical).
 * ============================================================================ */

#define APB2_CLOCK_HZ_LOCAL   96000000UL  /* USART1 di APB2 -- lihat bsp_spi.h utk sumber kebenaran clock tree */
#define APB1_CLOCK_HZ_LOCAL   48000000UL  /* USART2 di APB1 */

/* ---------------------------------------------------------------------------
 * Ring buffer generik (dipakai internal untuk RX USART1 & USART2)
 * ------------------------------------------------------------------------- */
typedef struct {
    volatile uint8_t buf[USART1_RX_BUF_SIZE]; /* ukuran sama dipakai keduanya, lihat static assert di bawah */
    volatile uint16_t head; /* index tulis, diisi ISR */
    volatile uint16_t tail; /* index baca, diisi caller */
} uart_ring_t;

/* USART1_RX_BUF_SIZE dan USART2_RX_BUF_SIZE didefinisikan sama besar di
 * header saat ini (256). Kalau nanti dibedakan, ubah struct ini jadi
 * pakai ukuran masing2 (mis. via macro terpisah) -- ditulis eksplisit di
 * sini sebagai pengingat, bukan diasumsikan diam2. */
#if USART1_RX_BUF_SIZE != USART2_RX_BUF_SIZE || USART1_RX_BUF_SIZE != USART6_RX_BUF_SIZE
#error "bsp_uart.c mengasumsikan USART1/2/6_RX_BUF_SIZE sama besar, sesuaikan uart_ring_t kalau beda"
#endif

static uart_ring_t s_uart1_rx;
static uart_ring_t s_uart2_rx;
static uart_ring_t s_uart6_rx;

static void ring_reset(uart_ring_t *r)
{
    r->head = 0u;
    r->tail = 0u;
}

static void ring_push(uart_ring_t *r, uint8_t byte)
{
    uint16_t next = (uint16_t)((r->head + 1u) % USART1_RX_BUF_SIZE);
    if (next == r->tail) {
        /* Buffer penuh -- byte terlama di-drop (overwrite tidak dilakukan,
         * byte baru ini yang di-drop) supaya data lama yang belum sempat
         * diproses tidak korup di tengah. Caller yang polling terlalu
         * jarang akan kehilangan byte terbaru, bukan byte tertua. */
        return;
    }
    r->buf[r->head] = byte;
    r->head = next;
}

static uint16_t ring_available(const uart_ring_t *r)
{
    return (uint16_t)((r->head - r->tail) % USART1_RX_BUF_SIZE);
}

static int ring_pop(uart_ring_t *r, uint8_t *out)
{
    if (r->head == r->tail) {
        return 0; /* kosong */
    }
    *out = r->buf[r->tail];
    r->tail = (uint16_t)((r->tail + 1u) % USART1_RX_BUF_SIZE);
    return 1;
}

/* ---------------------------------------------------------------------------
 * Helper: hitung nilai BRR dari clock peripheral + baud target.
 * over8=1 dipakai di sini untuk presisi kuantisasi yang lebih baik secara
 * umum -- dipertahankan dari implementasi CRSF sebelumnya (yang butuh
 * OVER8=1 karena baud tinggi 420000), TIDAK diubah ke OVER8=0 untuk iBus
 * walau baud-nya sekarang cuma 115200 (jauh lebih rendah), karena task
 * migrasi ini tidak minta tuning ulang oversampling -- OVER8=1 tetap
 * valid/aman dipakai di baud berapa pun, cuma beda presisi kuantisasi
 * (yang di 115200 malah lebih dari cukup).
 * ------------------------------------------------------------------------- */
static uint32_t compute_brr(uint32_t pclk_hz, uint32_t baud, int over8)
{
    uint32_t div_val   = over8 ? 8u : 16u;
    uint32_t denom      = div_val * baud;
    uint32_t mantissa   = pclk_hz / denom;
    uint32_t remainder  = pclk_hz % denom;

    /* fraction = round(remainder / denom * div_val) */
    uint32_t fraction = (remainder * div_val + (denom / 2u)) / denom;

    uint32_t frac_max = over8 ? 8u : 16u;
    if (fraction >= frac_max) {
        mantissa += 1u;
        fraction = 0u;
    }

    if (over8) {
        /* OVER8=1: DIV_Fraction cuma 3 bit valid (bit3 harus 0) */
        return (mantissa << 4) | (fraction & 0x7u);
    }
    return (mantissa << 4) | (fraction & 0xFu);
}

/* ---------------------------------------------------------------------------
 * USART1 - iBus RX-only
 * ------------------------------------------------------------------------- */

void BSP_UART1_Init(void)
{
    ring_reset(&s_uart1_rx);

    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    __asm volatile ("nop");
    __asm volatile ("nop");

    USART1->CR1 = 0; /* disable dulu, reset konfigurasi */

    /* OVER8=1 dipertahankan dari implementasi CRSF sebelumnya (lihat
     * catatan di compute_brr() di atas) -- masih valid untuk baud iBus
     * 115200, tidak ada alasan mengubahnya. */
    USART1->CR1 |= USART_CR1_OVER8;
    USART1->BRR  = compute_brr(APB2_CLOCK_HZ_LOCAL, USART1_BAUD_IBUS, 1);

    /* 8N1 (M=0, PCE=0, STOP default 1 di CR2=0), RX-only: RE tanpa TE */
    USART1->CR2 = 0;
    USART1->CR3 = 0;
    USART1->CR1 |= USART_CR1_RE | USART_CR1_RXNEIE;

    NVIC_SetPriority(USART1_IRQn, 5);
    NVIC_EnableIRQ(USART1_IRQn);

    USART1->CR1 |= USART_CR1_UE; /* enable peripheral */
}

uint16_t BSP_UART1_Available(void)
{
    return ring_available(&s_uart1_rx);
}

int BSP_UART1_ReadByte(uint8_t *out)
{
    return ring_pop(&s_uart1_rx, out);
}

void USART1_IRQHandler(void)
{
    if (USART1->SR & USART_SR_RXNE) {
        uint8_t byte = (uint8_t)USART1->DR; /* baca DR otomatis clear RXNE */
        ring_push(&s_uart1_rx, byte);
    }
    /* ORE (overrun) bisa muncul kalau ISR ini kalah cepat vs beban
     * interrupt tinggi -- baca SR lalu DR meng-clear ORE juga, jadi tidak
     * perlu handling terpisah, tapi byte yang overrun tetap hilang. Di
     * baud iBus 115200 (jauh lebih rendah dari CRSF 420000 sebelumnya)
     * risiko ORE ini juga jauh lebih longgar. Kalau ORE sering terjadi
     * di pengujian nyata, ini sinyal scheduler/ISR lain di sistem
     * terlalu lama menahan interrupt, bukan bug di ring buffer ini. */
}

/* ---------------------------------------------------------------------------
 * USART2 - GPS (TX+RX)
 * ------------------------------------------------------------------------- */

void BSP_UART2_Init(uint32_t baud)
{
    ring_reset(&s_uart2_rx);

    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    __asm volatile ("nop");
    __asm volatile ("nop");

    USART2->CR1 = 0; /* disable dulu, reset konfigurasi (termasuk kalau ganti baud) */

    /* Baud GPS umumnya rendah-menengah (9600-115200) -> OVER8=0 cukup presisi */
    USART2->BRR = compute_brr(APB1_CLOCK_HZ_LOCAL, baud, 0);

    USART2->CR2 = 0;
    USART2->CR3 = 0;
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

    NVIC_SetPriority(USART2_IRQn, 6);
    NVIC_EnableIRQ(USART2_IRQn);

    USART2->CR1 |= USART_CR1_UE; /* enable peripheral */
}

uint16_t BSP_UART2_Available(void)
{
    return ring_available(&s_uart2_rx);
}

int BSP_UART2_ReadByte(uint8_t *out)
{
    return ring_pop(&s_uart2_rx, out);
}

void BSP_UART2_WriteByte(uint8_t data)
{
    while (!(USART2->SR & USART_SR_TXE)) {
        /* tunggu TX buffer kosong */
    }
    USART2->DR = data;
}

void BSP_UART2_WriteBuf(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        BSP_UART2_WriteByte(data[i]);
    }
    /* Pastikan byte terakhir benar2 keluar dari shift register sebelum
     * caller menganggap transmisi selesai (penting kalau caller langsung
     * mematikan peripheral atau ganti baud setelah ini). */
    while (!(USART2->SR & USART_SR_TC)) {
        /* tunggu transmission complete */
    }
}

void USART2_IRQHandler(void)
{
    if (USART2->SR & USART_SR_RXNE) {
        uint8_t byte = (uint8_t)USART2->DR;
        ring_push(&s_uart2_rx, byte);
    }
}

/* ---------------------------------------------------------------------------
 * USART6 - Command/Config link cadangan (TX+RX), pola sama persis dengan
 * USART2 di atas. USART6 ada di bus APB2 (sama seperti USART1), makanya
 * pakai APB2_CLOCK_HZ_LOCAL, bukan APB1 seperti USART2.
 * ------------------------------------------------------------------------- */

void BSP_UART6_Init(uint32_t baud)
{
    ring_reset(&s_uart6_rx);

    RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
    __asm volatile ("nop");
    __asm volatile ("nop");

    USART6->CR1 = 0; /* disable dulu, reset konfigurasi */

    /* Baud command link 115200 (lihat USART6_BAUD_CMD) -> OVER8=0 cukup
     * presisi, sama pertimbangan seperti USART2. */
    USART6->BRR = compute_brr(APB2_CLOCK_HZ_LOCAL, baud, 0);

    USART6->CR2 = 0;
    USART6->CR3 = 0;
    USART6->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

    NVIC_SetPriority(USART6_IRQn, 6);
    NVIC_EnableIRQ(USART6_IRQn);

    USART6->CR1 |= USART_CR1_UE; /* enable peripheral */
}

uint16_t BSP_UART6_Available(void)
{
    return ring_available(&s_uart6_rx);
}

int BSP_UART6_ReadByte(uint8_t *out)
{
    return ring_pop(&s_uart6_rx, out);
}

static void uart6_write_byte(uint8_t data)
{
    while (!(USART6->SR & USART_SR_TXE)) {
        /* tunggu TX buffer kosong */
    }
    USART6->DR = data;
}

void BSP_UART6_WriteBuf(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uart6_write_byte(data[i]);
    }
    while (!(USART6->SR & USART_SR_TC)) {
        /* tunggu transmission complete, sama alasan seperti BSP_UART2_WriteBuf */
    }
}

void USART6_IRQHandler(void)
{
    if (USART6->SR & USART_SR_RXNE) {
        uint8_t byte = (uint8_t)USART6->DR;
        ring_push(&s_uart6_rx, byte);
    }
}
