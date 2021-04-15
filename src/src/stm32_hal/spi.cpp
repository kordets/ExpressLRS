#include "internal.h"
#include "helpers.h"
#include "gpio.h"
#include "stm32_def.h"
#include "debug_elrs.h"
#include "stm32f3xx.h"
struct spi_info
{
    SPI_TypeDef *spi;
    uint8_t miso_pin, mosi_pin, sck_pin, function;
};

#if defined(STM32F0xx) || defined(STM32L0xx)
#define SPI_FUNCTION GPIO_FUNCTION(0)
#else
#define SPI_FUNCTION GPIO_FUNCTION(5)
#endif

static const struct spi_info spi_bus[] = {
#ifdef SPI2
    {SPI2, GPIO('B', 14), GPIO('B', 15), GPIO('B', 13), SPI_FUNCTION},
#endif
#ifdef SPI1
    {SPI1, GPIO('A', 6), GPIO('A', 7), GPIO('A', 5), SPI_FUNCTION},
#endif
#ifdef SPI3
    {SPI3, GPIO('B', 4), GPIO('B', 5), GPIO('B', 3), GPIO_FUNCTION(6)},
#if CONFIG_MACH_STM32F4
    {SPI3, GPIO('C', 11), GPIO('C', 12), GPIO('C', 10), GPIO_FUNCTION(6)},
#endif
#endif
};


#define USE_HAL_VERSION 0

struct spi_config
spi_setup(uint32_t speed, int sck, int miso, int mosi, uint8_t mode)
{
    uint8_t bus;
    for (bus = 0; bus < ARRAY_SIZE(spi_bus); bus++)
    {
        struct spi_info *spi = (struct spi_info *)&spi_bus[bus];
        if (spi->miso_pin == miso && spi->mosi_pin == mosi && spi->sck_pin == sck)
            break;
    }
    if (bus < ARRAY_SIZE(spi_bus))
    {
        // Enable SPI
        SPI_TypeDef *spi = spi_bus[bus].spi;
        if (!is_enabled_pclock((uint32_t)spi))
        {
            enable_pclock((uint32_t)spi);
            gpio_peripheral(spi_bus[bus].miso_pin, spi_bus[bus].function, 1);
            gpio_peripheral(spi_bus[bus].mosi_pin, spi_bus[bus].function, 0);
            gpio_peripheral(spi_bus[bus].sck_pin, spi_bus[bus].function, 0);
        }

        // Calculate CR1 register
        uint32_t pclk = get_pclock_frequency((uint32_t)spi);
        uint32_t div = 0;
        while ((pclk >> (div + 1)) > speed && div < 7)
            div++;
#if !USE_HAL_VERSION
        uint32_t cr1 = ((mode << SPI_CR1_CPHA_Pos) |
                        (div << SPI_CR1_BR_Pos) |
                        SPI_CR1_SPE | SPI_CR1_MSTR |
                        SPI_CR1_SSM | SPI_CR1_SSI);
#if defined STM32L4xx || defined STM32F7xx
        spi->CR2 = SPI_DATASIZE_8BIT | SPI_RXFIFO_THRESHOLD_QF;
#endif
        return (struct spi_config){.spi = spi, .spi_cr1 = cr1};
#else
        static SPI_HandleTypeDef handle;
        handle.Instance = spi;
        handle.Init.BaudRatePrescaler = (div << SPI_CR1_BR_Pos);
        handle.Init.Direction = SPI_DIRECTION_2LINES;
        switch (mode) {
            case 0:
                handle.Init.CLKPhase = SPI_PHASE_1EDGE;
                handle.Init.CLKPolarity = SPI_POLARITY_LOW;
                break;
            case 1:
                handle.Init.CLKPhase = SPI_PHASE_2EDGE;
                handle.Init.CLKPolarity = SPI_POLARITY_LOW;
                break;
            case 2:
                handle.Init.CLKPhase = SPI_PHASE_1EDGE;
                handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
                break;
            case 3:
                handle.Init.CLKPhase = SPI_PHASE_2EDGE;
                handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
                break;
        }
        handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
        handle.Init.DataSize = SPI_DATASIZE_8BIT;
        handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
        handle.Init.NSS = SPI_NSS_SOFT;
        handle.Init.TIMode = SPI_TIMODE_DISABLED;
        handle.Init.Mode = SPI_MODE_MASTER;
        if (HAL_SPI_Init(&handle) != HAL_OK) {
            DEBUG_PRINTF("SPI Init failed!\n");
            Error_Handler();
        }
        __HAL_SPI_ENABLE(&handle);
        return (struct spi_config){.spi = &handle, .spi_cr1 = 0};
#endif
    }
    return {.spi = NULL, .spi_cr1 = 0};
}

void FAST_CODE_1
spi_prepare(struct spi_config config)
{
    SPI_TypeDef *spi = (SPI_TypeDef *)config.spi;
    if (!spi) return;
#if !USE_HAL_VERSION
    spi->CR1 = config.spi_cr1;
#endif
}

void FAST_CODE_1
spi_transfer(struct spi_config config, uint8_t receive_data, uint8_t len, uint8_t *data)
{
#if !USE_HAL_VERSION
    SPI_TypeDef *spi = (SPI_TypeDef *)config.spi;
#else
    SPI_HandleTypeDef * handle = (SPI_HandleTypeDef*)config.spi;
    SPI_TypeDef *spi = (SPI_TypeDef *)handle->Instance;
#endif
    if (!spi) return;
    while (len--) {
        //while (!(spi->SR & (SPI_SR_TXE)));
        *(__IO uint8_t *)&spi->DR = *data;
        while (!(spi->SR & SPI_SR_RXNE));
        uint8_t rdata = *(__IO uint8_t *)&spi->DR;
        if (receive_data)
            *data = rdata;
        data++;
    }
    //while (spi->SR & (SPI_SR_BSY));
}
