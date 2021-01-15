#if defined(TARGET_HANDSET_STM32F722)

#include "gpio.h"
#include "internal.h"
#include "stm32_def.h"
#include "helpers.h"

#if defined(STM32F7xx)
#include "stm32f7xx_hal_adc.h"
#include "stm32f7xx_hal_adc_ex.h"

#include "stm32f7xx_ll_adc.h"
#include "stm32f7xx_ll_dma.h"
#include "stm32f7xx_ll_gpio.h"
#endif

#define ADC_DMA DMA1
#define ADC_DMA_CH LL_DMA_CHANNEL_1


#define NUM_READS    16
struct adc_read {
    uint16_t ch1;
    uint16_t ch2;
    uint16_t ch3;
    uint16_t ch4;
};
union adc_raw {
    struct {
        struct adc_read low[NUM_READS/2];
        struct adc_read high[NUM_READS/2];
    };
    struct adc_read values[NUM_READS];
};
union adc_raw RawVals;

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

struct adc_pins {
    uint32_t pin;
    ADC_TypeDef * adc;
    uint32_t adc_ch;
};

struct adc_pins adc_pins[] = {
    {GPIO('A', 0), ADC1, ADC_CHANNEL_0},
    {GPIO('A', 1), ADC1, ADC_CHANNEL_1},
    {GPIO('A', 2), ADC1, ADC_CHANNEL_2},
    {GPIO('A', 3), ADC1, ADC_CHANNEL_3},
    {GPIO('A', 4), ADC1, ADC_CHANNEL_4},
    {GPIO('A', 5), ADC1, ADC_CHANNEL_5},
    {GPIO('A', 6), ADC1, ADC_CHANNEL_6},
    {GPIO('A', 7), ADC1, ADC_CHANNEL_7},
    {GPIO('B', 0), ADC1, ADC_CHANNEL_8},
    {GPIO('B', 1), ADC1, ADC_CHANNEL_9},
    {GPIO('C', 0), ADC1, ADC_CHANNEL_10},
    {GPIO('C', 1), ADC1, ADC_CHANNEL_11},
    {GPIO('C', 2), ADC1, ADC_CHANNEL_12},
    {GPIO('C', 3), ADC1, ADC_CHANNEL_13},
    {GPIO('C', 4), ADC1, ADC_CHANNEL_14},
    {GPIO('C', 5), ADC1, ADC_CHANNEL_15},
};

extern "C" {
void ADC_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc1);
}

void DMA2_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_adc1);
}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    /* Full - handle upper part */
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    /* Half - handle lower part */

}


static void ADC1_Init(void)
{
    if (hadc1.Instance)
        return;

    /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* ADC1 DMA Init */
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
        Error_Handler();
    }
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    /** Configure the global features of the ADC
     *  (Clock, Resolution, Data Alignment and number of conversion)
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV; //ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /* ADC1 interrupt Init */
    //HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
    //HAL_NVIC_EnableIRQ(ADC_IRQn);

    //HAL_NVIC_DisableIRQ(ADC_IRQn);

    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

static void ADC1_ConfigChannel(uint32_t channel)
{
    uint32_t num = hadc1.Init.NbrOfConversion;

    /** Configure for the selected ADC regular channel its corresponding
     * rank in the sequencer and its sample time.
     */
    ADC_ChannelConfTypeDef sConfig;
    sConfig.Channel = channel;
    sConfig.Rank = num;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    /* Set ADC number of conversion */
    hadc1.Instance->SQR1 &= ~(ADC_SQR1_L);
    hadc1.Instance->SQR1 |=  ADC_SQR1(num);
    hadc1.Init.NbrOfConversion = num + 1;
}

struct gpio_adc gpio_adc_setup(uint32_t pin)
{
    uint8_t chan;
    if (GPIO('J', 0) <= pin)
        return {.adc = NULL, .chan = 0};

    for (chan = 0; chan < ARRAY_SIZE(adc_pins); chan++) {
        if (adc_pins[chan].pin == pin)
            break;
    }
    if (chan < ARRAY_SIZE(adc_pins)) {
        ADC1_Init();
        gpio_peripheral(pin, GPIO_ANALOG, 0);
        ADC1_ConfigChannel(adc_pins[chan].adc_ch);
        return {.adc = hadc1.Instance, .chan = 0};
    }
    return {.adc = NULL, .chan = 0};
}

void gpio_adc_start(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&RawVals, sizeof(RawVals)/sizeof(uint32_t));
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
