#include "gimbals.h"
#include "gpio.h"
#include "internal.h"
#include "stm32_def.h"
#include "helpers.h"
#include "targets.h"
#include "1AUDfilter.h"
#include "debug_elrs.h"

#if defined(STM32F7xx)
#include "stm32f7xx_hal_adc.h"
#include "stm32f7xx_hal_adc_ex.h"

#include "stm32f7xx_ll_adc.h"
#include "stm32f7xx_ll_dma.h"
#include "stm32f7xx_ll_gpio.h"
#endif

#define TIMx TIM6
#define TIMx_IRQn TIM6_DAC_IRQn
#define TIMx_IRQx_FUNC TIM6_DAC_IRQHandler

#define TIMx_ISR_EN     1
#define TIM_INVERVAL_US 1000


struct gpio_out debug;

/* Contains:
 *  [ 0... 3] = channel1
 *  [ 4... 7] = channel2
 *  [ 8...11] = channel3
 *  [12...15] = channel4
*/
static uint16_t DRAM_ATTR RawVals[16];

/* STM32 ADC pins in channel order 0...n */
static uint32_t adc_pins[] = {
    GPIO('A', 0), GPIO('A', 1), GPIO('A', 2), GPIO('A', 3),
    GPIO('A', 4), GPIO('A', 5), GPIO('A', 6), GPIO('A', 7),
    GPIO('B', 0), GPIO('B', 1),
    GPIO('C', 0), GPIO('C', 1), GPIO('C', 2), GPIO('C', 3),
    GPIO('C', 4), GPIO('C', 5),
};

/* Create a filters */
static OneAUDfilter DRAM_ATTR filters[4] = {
    OneAUDfilter(100, 250, 0.01f, TIM_INVERVAL_US),
    OneAUDfilter(100, 250, 0.01f, TIM_INVERVAL_US),
    OneAUDfilter(100, 250, 0.01f, TIM_INVERVAL_US),
    OneAUDfilter(100, 250, 0.01f, TIM_INVERVAL_US),
};

void handle_dma_isr(void)
{
    //DEBUG_PRINTF("rdy %u! %u\n", LL_DMA_GetDataLength(DMA2, LL_DMA_STREAM_0), micros());
    uint32_t val;
    uint_fast8_t iter;
    for (iter = 0; iter < ARRAY_SIZE(RawVals); iter+=4) {
        val = RawVals[iter];
        val += RawVals[iter+1];
        val += RawVals[iter+2];
        val += RawVals[iter+3];
        filters[(iter / 4)].update((val / 4));
    }
}


extern "C" {
void ADC_IRQHandler(void)
{
    //DEBUG_PRINTF("ADC %u\n", micros());
    if (READ_BIT(ADC1->SR, ADC_SR_OVR)) {
        WRITE_REG(ADC1->SR , ADC_SR_OVR);
    }
}

void DMA2_Stream0_IRQHandler(void)
{
    // DMA transfer complete.
    if (READ_BIT(DMA2->LISR, DMA_LISR_TCIF0)) {
        WRITE_REG(DMA2->LIFCR , DMA_LIFCR_CTCIF0);
        handle_dma_isr();
        gpio_out_write(debug, 0);
    }

    // DMA transfer error.
    if (READ_BIT(DMA2->LISR, DMA_LISR_TEIF0)) {
        WRITE_REG(DMA2->LIFCR , DMA_LIFCR_CTEIF0);
        DEBUG_PRINTF("DMA ERROR\n");
    }
}

void TIMx_IRQx_FUNC(void)
{
    uint16_t SR = TIMx->SR;
    if (SR & TIM_SR_UIF) {
        TIMx->SR = SR & ~(TIM_SR_UIF);

        //DEBUG_PRINTF("TIM %u\n", micros());
        //gpio_out_toggle(debug);
        gpio_out_write(debug, 1);
    }
}
}


static void configure_dma(void) {
    // Enable DMA peripheral clock.
    enable_pclock((uint32_t)DMA2_Stream0);

    // Configure DMA transfer:
    //  - DMA transfer in circular mode to match with ADC configuration:
    //    DMA unlimited requests.
    //  - DMA transfer from ADC without address increment.
    //  - DMA transfer to memory with address increment.
    //  - DMA transfer from ADC by half-word to match with ADC configuration:
    //    ADC resolution 12 bits.
    //  - DMA transfer to memory by half-word to match with ADC conversion data
    //    buffer variable type: half-word.
    MODIFY_REG(DMA2_Stream0->CR, DMA_SxCR_CHSEL, 0x00);
    MODIFY_REG(DMA2_Stream0->CR,
                DMA_SxCR_DIR | DMA_SxCR_CIRC | DMA_SxCR_PINC | DMA_SxCR_MINC |
                DMA_SxCR_PSIZE | DMA_SxCR_MSIZE | DMA_SxCR_PL | DMA_SxCR_PFCTRL,
                0x00000000U |      // Direction: peripheral to memory
                DMA_SxCR_CIRC |    // Mode: circular
                0x00000000U |      // Peripheral: no increment
                DMA_SxCR_MINC |    // Memory: increment
                DMA_SxCR_PSIZE_0 | // Peripheral data align: halfword
                DMA_SxCR_MSIZE_0 | // Memory data align: halfword
                DMA_SxCR_PL_1);    // Priority: high

    // Set DMA transfer addresses.
    WRITE_REG(DMA2_Stream0->PAR, (uint32_t)&(ADC1->DR));
    WRITE_REG(DMA2_Stream0->M0AR, (uint32_t)&RawVals[0]);

    // Set DMA transfer size.
    MODIFY_REG(DMA2_Stream0->NDTR, DMA_SxNDT, ARRAY_SIZE(RawVals));

    // Enable DMA interrupts.
    NVIC_SetPriority(DMA2_Stream0_IRQn, 1); // DMA IRQ lower priority than ADC IRQ.
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    SET_BIT(DMA2_Stream0->CR, DMA_SxCR_TCIE);
    SET_BIT(DMA2_Stream0->CR, DMA_SxCR_TEIE);

    // Enable DMA transfer.
    SET_BIT(DMA2_Stream0->CR, DMA_SxCR_EN);
}


static void configure_timer(void)
{
    enable_pclock((uint32_t)TIMx);

    /* Config timer to trigger conversions */
    TIMx->CR1 = 0;
    TIMx->DIER = 0;
    TIMx->SR &= ~(TIM_SR_UIF);
    // Set clock prescaler to 1us
    TIMx->PSC = (2 * get_pclock_frequency((uint32_t)TIMx) / 1000000) - 1;
    TIMx->ARR = TIM_INVERVAL_US - 1;
    TIMx->CNT = 0;
    MODIFY_REG(TIMx->CR2, TIM_CR2_MMS, 0x2 << TIM_CR2_MMS_Pos); // TRGO update
#if TIMx_ISR_EN
    //TIMx->EGR = TIM_EGR_UG;
    TIMx->DIER = TIM_DIER_UIE;
    NVIC_SetPriority(TIMx_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
    NVIC_EnableIRQ(TIMx_IRQn);
#endif
    TIMx->CR1 = TIM_CR1_CEN | TIM_CR1_URS | TIM_CR1_DIR;
}

static void configure_adc_channel(uint32_t channel, uint8_t rank)
{
    //delay(100);
    //DEBUG_PRINTF("ADC cfg: ch %u, rank %u\n", channel, rank);

    /** Configure for the selected ADC regular channel its corresponding
     * rank in the sequencer and its sample time.
     */
    uint32_t mask;
    __IO uint32_t * REG;
    rank -= 1;

    /* Set channel time */
    if (rank < 6) {
        REG = &ADC1->SQR3;
    } else if (rank < 12) {
        rank -= 6;
        REG = &ADC1->SQR2;
    } else if (rank < 16) {
        rank -= 12;
        REG = &ADC1->SQR1;
    }
    rank *= 5;
    mask = ADC_SQR3_SQ1_Msk << rank;
    MODIFY_REG(*REG, mask, channel << rank);

    /* Set sampling time */
    if (channel < 10) {
        REG = &ADC1->SMPR2;
    } else {
        channel -= 10;
        REG = &ADC1->SMPR1;
    }
    channel *= 3;
    mask = ADC_SMPR1_SMP10_Msk << channel;
    MODIFY_REG(*REG, mask, (uint32_t)ADC_SAMPLETIME_480CYCLES << channel);
    /* 480T = ~300us */
    /* 144T = ~100us */
    /* 112T =  ~90us */
    /*  84T =  ~74us */
}

static void configure_adc(void)
{
    uint32_t channels[4] = {
        GIMBAL_L1, GIMBAL_L2,
        GIMBAL_R1, GIMBAL_R2,
    };
    uint32_t pin;
    uint8_t iter, chan, jter;

    configure_dma();
    configure_timer();

    /* Peripheral clock enable */
    enable_pclock((uint32_t)ADC1);

    //NVIC_SetPriority(ADC_IRQn, 0);
    //NVIC_EnableIRQ(ADC_IRQn);

    // Set ADC clock source: APB2 clock / 2.
    MODIFY_REG(ADC123_COMMON->CCR, ADC_CCR_ADCPRE, 0);


    for (iter = 0; iter < ARRAY_SIZE(channels); iter++) {
        for (chan = 0; chan < ARRAY_SIZE(adc_pins); chan++) {
            pin = channels[iter];
            if (adc_pins[chan] == pin) {
                gpio_peripheral(pin, GPIO_ANALOG, 0);
                for (jter = 0; jter < 4; jter++) {
                    configure_adc_channel(chan, ((iter * 4) + jter + 1));
                }
            }
        }
    }

    //uint32_t CR1 = ADC1->CR1 & ~0x03C0FFFF;
    uint32_t CR2 = ADC1->CR2 & ~0x7F7F0F03;

    // Set ADC group regular continuous mode: off.
    //MODIFY_REG(ADC1->CR2, ADC_CR2_CONT, 0);

    // Set ADC group regular trigger source: rising edge of TIM2 TRGO.
    //MODIFY_REG(ADC1->CR2, ADC_CR2_EXTSEL, 0x0B << ADC_CR2_EXTSEL_Pos);
    //MODIFY_REG(ADC1->CR2, ADC_CR2_EXTEN, 0x01 << ADC_CR2_EXTEN_Pos);
    //CR2 |= (ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_3); // TIM4
    CR2 |= (ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_3 |ADC_CR2_EXTSEL_0); // TIM6
    CR2 |= 0x01 << ADC_CR2_EXTEN_Pos;

    // Set ADC group regular sequencer length: four channels.
    MODIFY_REG(ADC1->SQR1, ADC_SQR1_L, 15LU << ADC_SQR1_L_Pos);

    // Enable multiple conversions.
    SET_BIT(ADC1->CR1, (ADC_CR1_SCAN | ADC_IT_OVR));
    // Enable DMA transfer for ADC. and start
    //MODIFY_REG(ADC1->CR2, ADC_CR2_DMA | ADC_CR2_DDS, ADC_CR2_DDS | ADC_CR2_DMA);
    CR2 |= (ADC_CR2_DDS | ADC_CR2_DMA | ADC_CR2_ADON);
    ADC1->SR &= ~(ADC_FLAG_EOC | ADC_FLAG_OVR);
    ADC1->CR2 = CR2;


    DEBUG_PRINTF("ADC started!\n");
}


void gimbals_init(void)
{
    debug = gpio_out_setup(PB12, 0);
    configure_adc();

    //uint32_t CFGR = RCC->DCKCFGR1;
    //DEBUG_PRINTF("RCC bit: %u\n", (CFGR & RCC_DCKCFGR1_TIMPRE_Msk));

    while(1) {
        DEBUG_PRINTF("hello! [0]=%u, [1]=%u, [2]=%u, [3]=%u\r\n",
            (uint32_t)filters[0].getCurrent(), (uint32_t)filters[1].getCurrent(),
            (uint32_t)filters[2].getCurrent(), (uint32_t)filters[3].getCurrent());
        delay(1000);
    }

}

void gimbals_timer_adjust(int32_t off)
{
    TIMx->ARR = TIM_INVERVAL_US - off;
}

void gimbals_get(uint16_t &l1, uint16_t &l2, uint16_t &r1, uint16_t &r2)
{
    l1 = filters[0].getCurrent();
    l2 = filters[1].getCurrent();
    r1 = filters[2].getCurrent();
    r2 = filters[3].getCurrent();
}



struct gpio_adc gpio_adc_setup(uint32_t pin)
{
    return {.adc = NULL, .chan = 0};
}

uint32_t gpio_adc_sample(struct gpio_adc g)
{
    return 0;
}

uint16_t gpio_adc_read(struct gpio_adc g)
{
    return 0;
}

void gpio_adc_cancel_sample(struct gpio_adc g)
{

}
