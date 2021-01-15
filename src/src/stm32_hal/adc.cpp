#if defined(TARGET_HANDSET_STM32F722)

#include "gpio.h"
#include "internal.h"
#include "hal_inc.h"

#if defined(STM32F7xx)
#include "stm32f7xx_hal_adc.h"
#include "stm32f7xx_hal_adc_ex.h"

#include "stm32f7xx_ll_adc.h"
#include "stm32f7xx_ll_dma.h"
#include "stm32f7xx_ll_gpio.h"
#endif

#define ADC_DMA DMA1
#define ADC_DMA_CH LL_DMA_CHANNEL_1


#define NUM_ADCS    8
uint16_t RawVals[NUM_ADCS];


/* ADC1 init function */
void MX_ADC1_Init(void)
{
    LL_ADC_CommonInitTypeDef common;
    LL_ADC_CommonStructInit(&common);
    LL_ADC_CommonInit(ADC, &common);

    LL_ADC_InitTypeDef init;
    LL_ADC_StructInit(&init);
    LL_ADC_Init(ADC1, &init); // ADC1..3

    LL_ADC_REG_InitTypeDef channel;
    LL_ADC_REG_StructInit(&channel);
    channel.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
    LL_ADC_REG_Init(ADC1, &channel);

    /* Peripheral clock enable */
    enable_pclock((uint32_t)ADC1);

}


void ADC_DMA_Start(void)
{
    //LL_ADC_Enable(ADC1);
    //LL_DMA_EnableChannel(ADC_DMA, ADC_DMA_CH);
    //LL_ADC_REG_StartConversionSWStart(ADC1);
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

#endif /* ELRS_HANDSET */
