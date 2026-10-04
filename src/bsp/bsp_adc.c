#include "bsp_adc.h"
#include "bsp_pinmap.h"

/* ============================================================================
 * bsp_adc.c
 * Implementasi bare-metal ADC1 (register langsung, tanpa HAL): scan mode
 * 2 channel (VBAT=IN0, current=IN1), continuous conversion, DMA2 Stream0
 * Channel0 circular ke buffer 2 elemen.
 * ============================================================================ */

#define APB2_CLOCK_HZ_LOCAL   96000000UL /* lihat bsp_spi.h utk sumber kebenaran clock tree */

/* Urutan buffer HARUS sinkron dengan urutan SQR3 (SQ1=channel0, SQ2=channel1)
 * di BSP_ADC_Init() -- index 0 = VBAT (IN0), index 1 = current (IN1). */
static volatile uint16_t s_adc_dma_buf[2];

void BSP_ADC_Init(void)
{
    /* --- Clock ADC1 --- */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    __asm volatile ("nop");
    __asm volatile ("nop");

    /* Prescaler ADC common: ADCCLK = APB2/4 = 96MHz/4 = 24MHz (<=36MHz spec) */
    ADC1_COMMON->CCR &= ~ADC_CCR_ADCPRE;
    ADC1_COMMON->CCR |= ADC_CCR_ADCPRE_0; /* 01 -> /4 */

    ADC1->CR1 = 0;
    ADC1->CR1 |= ADC_CR1_SCAN; /* scan mode, wajib untuk >1 channel di sequence */

    ADC1->CR2 = 0;
    ADC1->CR2 |= ADC_CR2_CONT;  /* continuous conversion */
    ADC1->CR2 |= ADC_CR2_DMA;   /* request DMA tiap EOC */
    ADC1->CR2 |= ADC_CR2_DDS;   /* tetap request DMA berikutnya (perlu untuk mode circular kontinu) */
    /* EXTEN=00 (default) -> software trigger only, sesuai kebutuhan (bukan
     * dipicu timer eksternal) */

    /* Sample time panjang (480 cycles, kode 0b111) untuk kedua channel --
     * VBAT/current tidak butuh sample rate tinggi, prioritaskan akurasi
     * (impedansi sumber voltage-divider biasanya cukup tinggi). */
    ADC1->SMPR2 &= ~(ADC_SMPR2_SMP0 | ADC_SMPR2_SMP1);
    ADC1->SMPR2 |= (0x7u << (0u * 3u)); /* channel0 (VBAT) */
    ADC1->SMPR2 |= (0x7u << (1u * 3u)); /* channel1 (current) */

    /* Sequence: 2 conversion (L = jumlah-1 = 1), SQ1=channel0, SQ2=channel1 */
    ADC1->SQR1 &= ~ADC_SQR1_L;
    ADC1->SQR1 |= (0x1u << ADC_SQR1_L_Pos);

    ADC1->SQR3 = (0u << 0)   /* SQ1 = channel 0 (VBAT) */
               | (1u << 5);  /* SQ2 = channel 1 (current) */

    /* --- DMA2 Stream0 Channel0 (peripheral->memory, circular) --- */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    __asm volatile ("nop");
    __asm volatile ("nop");

    DMA2_Stream0->CR &= ~DMA_SxCR_EN; /* pastikan stream mati sebelum reconfig */
    while (DMA2_Stream0->CR & DMA_SxCR_EN) {
        /* tunggu disable selesai sebelum ubah register lain (RM0383) */
    }

    DMA2_Stream0->PAR  = (uint32_t)&ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t)s_adc_dma_buf;
    DMA2_Stream0->NDTR = 2u; /* 2 elemen: VBAT, current */

    DMA2_Stream0->CR = 0;
    DMA2_Stream0->CR |= (0u << DMA_SxCR_CHSEL_Pos); /* channel 0 = ADC1 */
    DMA2_Stream0->CR |= DMA_SxCR_MINC;              /* memory increment */
    DMA2_Stream0->CR |= DMA_SxCR_CIRC;              /* circular -- otomatis wrap ke elemen 0 */
    DMA2_Stream0->CR |= (0x1u << DMA_SxCR_PSIZE_Pos); /* peripheral size 16-bit (ADC DR 12-bit dibaca sbg half-word) */
    DMA2_Stream0->CR |= (0x1u << DMA_SxCR_MSIZE_Pos); /* memory size 16-bit */
    DMA2_Stream0->CR |= (0x2u << DMA_SxCR_PL_Pos);    /* priority high */
    /* DIR = 00 (peripheral-to-memory), default sudah benar, tidak perlu di-set */

    DMA2_Stream0->CR |= DMA_SxCR_EN; /* enable stream */

    /* --- Nyalakan ADC + mulai konversi --- */
    ADC1->CR2 |= ADC_CR2_ADON;
    for (volatile int d = 0; d < 1000; d++) { /* tstab ADC (~beberapa us), delay longgar */ }

    ADC1->CR2 |= ADC_CR2_SWSTART; /* mulai scan pertama, CONT bit menjaga tetap jalan */
}

uint16_t BSP_ADC_GetVBatRaw(void)
{
    return s_adc_dma_buf[0];
}

uint16_t BSP_ADC_GetCurrentRaw(void)
{
    return s_adc_dma_buf[1];
}

float BSP_ADC_RawToPinVoltage(uint16_t raw)
{
    return ((float)raw * ADC_VREF_VOLTS) / (float)ADC_RESOLUTION;
}
