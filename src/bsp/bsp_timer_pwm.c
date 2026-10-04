#include "bsp_timer_pwm.h"
#include "bsp_pinmap.h"

/* ============================================================================
 * bsp_timer_pwm.c
 * Implementasi bare-metal (register langsung, tanpa HAL) untuk servo PWM
 * (TIM3/TIM4) dan DShot (TIM1 + DMA2 Stream1 burst ke CCR1).
 * ============================================================================ */

/* ---------------------------------------------------------------------------
 * Servo (TIM3/TIM4)
 * ------------------------------------------------------------------------- */

/* Tick 1us -> PSC = (clock/1MHz) - 1. ARR = periode_us - 1 (20000us @ 50Hz). */
#define SERVO_PSC   ((TIM1_CLOCK_HZ_LOCAL / 1000000u) - 1u)
#define SERVO_ARR   ((1000000u / SERVO_PWM_FREQ_HZ) - 1u)

static uint16_t clamp_pulse_us(uint16_t pulse_us)
{
    if (pulse_us < SERVO_PULSE_MIN_US) {
        return SERVO_PULSE_MIN_US;
    }
    if (pulse_us > SERVO_PULSE_MAX_US) {
        return SERVO_PULSE_MAX_US;
    }
    return pulse_us;
}

/* Konfigurasi generik CH3/CH4 (dipakai sama untuk TIM3 & TIM4 -- keduanya
 * punya layout register identik untuk CCMR2/CCER/CCR3/CCR4). */
static void servo_timer_configure(TIM_TypeDef *tim)
{
    tim->CR1 = 0;
    tim->PSC = (uint16_t)SERVO_PSC;
    tim->ARR = (uint16_t)SERVO_ARR;

    /* CH3: OC3M=110 (PWM mode1), OC3PE=1 (preload) -- CCMR2 bit[6:4]=OC3M, bit3=OC3PE */
    tim->CCMR2 &= ~(TIM_CCMR2_OC3M | TIM_CCMR2_OC4M);
    tim->CCMR2 |= (0x6u << 4) | TIM_CCMR2_OC3PE; /* CH3: mode1 + preload */
    tim->CCMR2 |= (0x6u << 12) | TIM_CCMR2_OC4PE; /* CH4: mode1 + preload */

    tim->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E; /* enable output CH3, CH4 */

    tim->CCR3 = SERVO_PULSE_CENTER_US; /* posisi netral saat boot, lihat header */
    tim->CCR4 = SERVO_PULSE_CENTER_US;

    tim->CR1 |= TIM_CR1_ARPE; /* auto-reload preload */
    tim->EGR |= TIM_EGR_UG;   /* force update supaya PSC/ARR/CCR langsung terpakai */

    tim->CR1 |= TIM_CR1_CEN;  /* start counter */
}

void BSP_TIM_Servo_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
    __asm volatile ("nop");
    __asm volatile ("nop");

    servo_timer_configure(TIM3); /* AIL_L (CH3) / AIL_R (CH4) */
    servo_timer_configure(TIM4); /* VTAIL_L (CH3) / VTAIL_R (CH4) */
}

void BSP_TIM_Servo_SetPulseUs(servo_channel_t ch, uint16_t pulse_us)
{
    uint16_t us = clamp_pulse_us(pulse_us);

    switch (ch) {
        case SERVO_CH_AIL_L:   TIM3->CCR3 = us; break;
        case SERVO_CH_AIL_R:   TIM3->CCR4 = us; break;
        case SERVO_CH_VTAIL_L: TIM4->CCR3 = us; break;
        case SERVO_CH_VTAIL_R: TIM4->CCR4 = us; break;
        default: break; /* nilai enum tidak valid -- sengaja diabaikan, bukan tugas BSP validasi caller */
    }
}

/* ---------------------------------------------------------------------------
 * DShot (TIM1 + DMA2 Stream1)
 * ------------------------------------------------------------------------- */

void BSP_TIM_DShot_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    __asm volatile ("nop");
    __asm volatile ("nop");

    TIM1->CR1 = 0;
    TIM1->PSC = 0; /* full 96MHz tick -- resolusi maksimum untuk timing bit DShot */
    TIM1->ARR = DSHOT_BIT_PERIOD_TICKS - 1u;

    /* CH1: PWM mode1, preload -- CCMR1 bit[6:4]=OC1M, bit3=OC1PE */
    TIM1->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM1->CCMR1 |= (0x6u << 4) | TIM_CCMR1_OC1PE;

    TIM1->CCER |= TIM_CCER_CC1E;

    /* TIM1 advanced-control timer: output tidak aktif tanpa MOE (main
     * output enable) di BDTR -- wajib di-set, tidak seperti TIM3/TIM4. */
    TIM1->BDTR |= TIM_BDTR_MOE;

    TIM1->CCR1 = 0; /* idle: line low, tidak ada frame valid */

    /* CCDS=1: DMA request "CC1" dipicu di UPDATE EVENT (bukan capture-
     * compare match) -- teknik standar DShot bit-banging, lihat header. */
    TIM1->CR2 |= TIM_CR2_CCDS;
    TIM1->DIER |= TIM_DIER_CC1DE;

    TIM1->CR1 |= TIM_CR1_ARPE;
    TIM1->EGR |= TIM_EGR_UG;

    TIM1->CR1 |= TIM_CR1_CEN; /* timer jalan terus, CCR1=0 -> idle low */

    /* --- Siapkan DMA2 Stream1 Channel6 (belum di-enable, menunggu frame pertama) --- */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    __asm volatile ("nop");
    __asm volatile ("nop");

    DMA2_Stream1->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream1->CR & DMA_SxCR_EN) {
        /* tunggu disable selesai sebelum ubah register lain (RM0383) */
    }

    DMA2_Stream1->PAR = (uint32_t)&TIM1->CCR1;
    /* M0AR & NDTR diisi per-frame di BSP_TIM_DShot_SendFrame() */

    DMA2_Stream1->CR = 0;
    DMA2_Stream1->CR |= (6u << DMA_SxCR_CHSEL_Pos);   /* channel 6 = TIM1_CH1 */
    DMA2_Stream1->CR |= (0x1u << DMA_SxCR_DIR_Pos);   /* memory-to-peripheral */
    DMA2_Stream1->CR |= DMA_SxCR_MINC;                /* memory increment (baca array duty berurutan) */
    DMA2_Stream1->CR |= (0x1u << DMA_SxCR_PSIZE_Pos); /* peripheral size 16-bit (CCR1) */
    DMA2_Stream1->CR |= (0x1u << DMA_SxCR_MSIZE_Pos); /* memory size 16-bit */
    DMA2_Stream1->CR |= (0x2u << DMA_SxCR_PL_Pos);    /* priority high */
    DMA2_Stream1->CR |= DMA_SxCR_TCIE;                /* interrupt saat transfer selesai */
    /* CIRC sengaja TIDAK di-set -- one-shot per frame, bukan circular */

    NVIC_SetPriority(DMA2_Stream1_IRQn, 3); /* prioritas tinggi -- timing-sensitive */
    NVIC_EnableIRQ(DMA2_Stream1_IRQn);
}

void BSP_TIM_EscPwm_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    __asm volatile ("nop");
    __asm volatile ("nop");

    TIM1->CR1 = 0;
    TIM1->PSC = (TIM1_CLOCK_HZ_LOCAL / 1000000u) - 1u;
    TIM1->ARR = 20000u - 1u;
    TIM1->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM1->CCMR1 |= (0x6u << 4) | TIM_CCMR1_OC1PE;
    TIM1->CCER |= TIM_CCER_CC1E;
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CCR1 = 1000u;
    TIM1->CR1 |= TIM_CR1_ARPE;
    TIM1->EGR |= TIM_EGR_UG;
    TIM1->CR1 |= TIM_CR1_CEN;
}

void BSP_TIM_EscPwm_SetPulseUs(uint16_t pulse_us)
{
    if (pulse_us < 1000u) pulse_us = 1000u;
    if (pulse_us > 2000u) pulse_us = 2000u;
    TIM1->CCR1 = pulse_us;
}

int BSP_TIM_DShot_SendFrame(const uint16_t *duty_ticks, uint16_t sample_count)
{
    if (sample_count != DSHOT_FRAME_BITS) {
        return 0;
    }
    if (DMA2_Stream1->CR & DMA_SxCR_EN) {
        /* Frame sebelumnya masih dalam transfer -- pada DSHOT300, satu
         * frame 16-bit hanya makan ~53us, jadi ini seharusnya jarang
         * terjadi kecuali caller memanggil SendFrame terlalu rapat.
         * Caller (output_map.c) sebaiknya skip siklus ini, bukan retry
         * paksa dalam loop sibuk. */
        return 0;
    }

    DMA2_Stream1->M0AR = (uint32_t)duty_ticks;
    DMA2_Stream1->NDTR = DSHOT_FRAME_BITS;

    /* Clear semua flag stream1 sebelum enable (LIFCR utk stream 0-3) */
    DMA2->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1
                | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;

    DMA2_Stream1->CR |= DMA_SxCR_EN;
    return 1;
}

void DMA2_Stream1_IRQHandler(void)
{
    if (DMA2->LISR & DMA_LISR_TCIF1) {
        DMA2->LIFCR = DMA_LIFCR_CTCIF1; /* clear transfer-complete flag */

        /* Kembalikan CCR1 ke 0 (idle low) supaya periode berikutnya tidak
         * "mewarisi" duty bit terakhir dari frame yang baru selesai --
         * lihat catatan desain di bsp_timer_pwm.h. Timer tetap jalan terus
         * (CEN tidak disentuh), hanya duty-nya yang direset ke idle. */
        TIM1->CCR1 = 0;
    }
}

/* ---------------------------------------------------------------------------
 * Wrapper kompatibilitas (lihat catatan di bsp_timer_pwm.h)
 * ------------------------------------------------------------------------- */

bool BSP_TimerPWM_ServoInit(void)
{
    BSP_TIM_Servo_Init(); /* void -- tidak ada kegagalan yang dilaporkan HW */
    return true;
}

void BSP_TimerPWM_ServoSetPulseUs(uint8_t channel_index, uint16_t pulse_us)
{
    BSP_TIM_Servo_SetPulseUs((servo_channel_t)channel_index, pulse_us);
}

bool BSP_DShotTimer_Init(void)
{
    BSP_TIM_DShot_Init(); /* void -- tidak ada kegagalan yang dilaporkan HW */
    return true;
}

bool BSP_DShotTimer_IsTransferBusy(void)
{
    /* Sama seperti pengecekan internal di BSP_TIM_DShot_SendFrame(): DMA
     * masih EN berarti frame sebelumnya belum selesai ditransfer. */
    return (DMA2_Stream1->CR & DMA_SxCR_EN) != 0u;
}

bool BSP_DShotTimer_SendFrameDMA(const uint16_t *duty_ticks, uint16_t length)
{
    return BSP_TIM_DShot_SendFrame(duty_ticks, length) != 0;
}
