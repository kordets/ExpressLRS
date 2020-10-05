#include "Arduino.h"
#include "stm32_def.h"
#include "priorities.h"

#ifdef STM32L0xx

#include <stm32l0xx_ll_dma.h>

#define APB2PERIPH_BASE (APBPERIPH_BASE + 0x00010000UL)

uint32_t dma_get(uint32_t periph, uint8_t type, uint8_t index)
{
    return DMA1_BASE;
}

uint32_t dma_channel_get(uint32_t periph, uint8_t type, uint8_t index)
{
    if (periph == USART1_BASE || periph == USART2_BASE)
        return (type == DMA_USART_RX) ? LL_DMA_CHANNEL_5 : LL_DMA_CHANNEL_4;
    return 0xff;
}

uint32_t dma_irq_get(uint32_t periph, uint8_t type, uint8_t index)
{
    if (periph == USART1_BASE || periph == USART2_BASE)
        return DMA1_Channel4_5_6_7_IRQn;
    return 0xff;
}

uint32_t uart_peripheral_get(uint32_t rx, uint32_t tx)
{
    if ((rx == GPIO('A', 10) && tx == GPIO('A', 9)) ||
        (rx == GPIO('B', 7) && tx == GPIO('B', 6)))
        return (uint32_t)USART1;
    else if (rx == GPIO('A', 3) && tx == GPIO('A', 2))
        return (uint32_t)USART2;
    return 0;
}

void uart_pins_get(uint32_t periph, uint32_t *rx_pin, uint32_t *tx_pin, uint8_t alt)
{
    switch (periph) {
        case USART1_BASE:
            *rx_pin = !alt ? GPIO('A', 10) : GPIO('B', 7);
            *tx_pin = !alt ? GPIO('A', 9) : GPIO('B', 6);
            break;
        case USART2_BASE:
            *rx_pin = GPIO('A', 3);
            *tx_pin = GPIO('A', 2);
            break;
    }
}

void uart_config_afio(uint32_t periph, uint32_t rx_pin, uint32_t tx_pin)
{
    // USART1 has alternative 0 config when pins are PB6 and PB7
    gpio_peripheral(rx_pin, GPIO_FUNCTION((rx_pin == GPIO('B', 7) ? 0 : 4)), 1);
    gpio_peripheral(tx_pin, GPIO_FUNCTION((tx_pin == GPIO('B', 6) ? 0 : 4)), 0);
}

// Enable a peripheral clock
void enable_pclock(uint32_t periph_base)
{
    if (is_enabled_pclock(periph_base))
        return;

    if (periph_base < APB2PERIPH_BASE) {
        uint32_t pos = (periph_base - APBPERIPH_BASE) / 0x400;
        RCC->APB1ENR |= (1 << pos);
        RCC->APB1ENR;
    } else if (periph_base < AHBPERIPH_BASE) {
        uint32_t pos = (periph_base - APB2PERIPH_BASE) / 0x400;
        RCC->APB2ENR |= (1 << pos);
        RCC->APB2ENR;
    } else {
        uint32_t pos = (periph_base - AHBPERIPH_BASE) / 0x400;
        RCC->AHBENR |= (1 << pos);
        RCC->AHBENR;
    }
}

// Check if a peripheral clock has been enabled
uint32_t is_enabled_pclock(uint32_t periph_base)
{
    if (periph_base < APB2PERIPH_BASE) {
        uint32_t pos = (periph_base - APBPERIPH_BASE) / 0x400;
        return RCC->APB1ENR & (1 << pos);
    } else if (periph_base < AHBPERIPH_BASE) {
        uint32_t pos = (periph_base - APB2PERIPH_BASE) / 0x400;
        return RCC->APB2ENR & (1 << pos);
    } else {
        uint32_t pos = (periph_base - AHBPERIPH_BASE) / 0x400;
        return RCC->AHBENR & (1 << pos);
    }
}

// Return the frequency of the given peripheral clock
uint32_t
get_pclock_frequency(uint32_t periph_base)
{
    return CONFIG_CLOCK_FREQ / 2;
}

// Enable a GPIO peripheral clock
void gpio_clock_enable(GPIO_TypeDef *regs)
{
    uint32_t rcc_pos = ((uint32_t)regs - IOPPERIPH_BASE) / 0x400;
    RCC->IOPENR |= 1 << rcc_pos;
    (void)RCC->IOPENR;
}

// Set the mode and extended function of a pin
void gpio_peripheral(uint32_t gpio, uint32_t mode, int pullup)
{
    GPIO_TypeDef *regs = digital_regs[GPIO2PORT(gpio)];
    GPIO_InitTypeDef init = {0};
    if (!regs)
        return;

    // Enable GPIO clock
    gpio_clock_enable(regs);

    init.Pin = GPIO2BIT(gpio);
    init.Speed = GPIO_SPEED_FREQ_MEDIUM;
    init.Pull = (!pullup) ? GPIO_NOPULL : ((pullup < 0) ? GPIO_PULLDOWN : GPIO_PULLUP);
    if (mode == GPIO_INPUT) {
        init.Mode = GPIO_MODE_INPUT;
    } else if (mode == GPIO_OUTPUT) {
        // push-pull, 0b00 | max speed 2 MHz, 0b01
        init.Mode = GPIO_MODE_OUTPUT_PP;
    } else if (mode == (GPIO_OUTPUT | GPIO_OPEN_DRAIN)) {
        // Open-drain, 0b01 | max speed 2 MHz, 0b01
        init.Mode = GPIO_MODE_OUTPUT_OD;
    } else if (mode == GPIO_ANALOG) {
        init.Mode = GPIO_MODE_ANALOG;
    } else {
        init.Alternate = (mode >> 4);
        init.Speed = GPIO_SPEED_FREQ_HIGH;
        // Alternate config
        if (mode & GPIO_OPEN_DRAIN) {
            // output open-drain mode, 10MHz
            init.Mode = GPIO_MODE_AF_OD;
        } else {
            // output push-pull mode, 10MHz
            init.Mode = GPIO_MODE_AF_PP;
        }
    }
    HAL_GPIO_Init(regs, &init);
}


// Return the current time (in absolute clock ticks).
uint32_t timer_read_time(void)
{
    // Cortex M0 does not have DWT so read us from SysTick
    uint32_t ms = millis();
    uint32_t period = SysTick->LOAD + 1;
    uint32_t us = period - SysTick->VAL;
    return (ms * 1000 + (us * 1000) / period);
}

uint32_t micros(void)
{
    return timer_read_time();
}

void delayMicroseconds(uint32_t usecs)
{
    //uint32_t end = timer_read_time() + microsecondsToClockCycles(usecs);
    //while (timer_is_before(timer_read_time(), end))
    //    ;
    uint32_t start = timer_read_time();
    while ((timer_read_time() - start) < usecs);
}

void timer_init(void)
{
    // Enable Debug Watchpoint and Trace (DWT) for its 32bit timer
    /*
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
    */

    // Enable SysTick
    NVIC_SetPriority(SysTick_IRQn, ISR_PRIO_TICKS);
    SysTick->LOAD = (uint32_t)(SystemCoreClock / 1000UL) - 1;
    SysTick->VAL = 0UL;
    SysTick->CTRL = (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            PLL_Source                     = HSE
  *            PLL_Mul                        = 9
  *            Flash Latency(WS)              = 2
  *            ADC Prescaler                  = 6
  *            USB Prescaler                  = 1.5
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* Configure the main internal regulator output voltage */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Initializes the CPU, AHB and APB busses clocks */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV2;
    RCC_OscInitStruct.HSICalibrationValue = 0x10;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /* Initializes the CPU, AHB and APB busses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                    | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection =
        RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART2 |
        RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_I2C2;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }

    SystemCoreClockUpdate();

    /* Enable SYSCFG Clock */
    __HAL_RCC_SYSCFG_CLK_ENABLE();
}

void hw_init(void)
{
    /* Configure Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
}

extern "C" {
void NMI_Handler(void) {}
void HardFault_Handler(void) {Error_Handler();}
void SVC_Handler(void) {}
void PendSV_Handler(void) {}
// void SysTick_Handler(void) {Error_Handler();}
void WWDG_IRQHandler(void) {Error_Handler();}
void PVD_IRQHandler(void) {Error_Handler();}
void RTC_IRQHandler(void) {Error_Handler();}
void FLASH_IRQHandler(void) {Error_Handler();}
void RCC_IRQHandler(void) {Error_Handler();}

void EXTI0_1_IRQHandler(void) {
    GPIO_EXTI_IRQHandler(0);
    GPIO_EXTI_IRQHandler(1);
}
void EXTI2_3_IRQHandler(void) {
    GPIO_EXTI_IRQHandler(2);
    GPIO_EXTI_IRQHandler(3);
}
void EXTI4_15_IRQHandler(void) {
    uint8_t pin;
    for (pin = 4; pin <= 15; pin++)
    {
        GPIO_EXTI_IRQHandler(pin);
    }
}

void DMA1_Channel1_IRQHandler(void) {Error_Handler();}
void DMA1_Channel2_3_IRQHandler(void) {Error_Handler();}
void DMA1_Channel4_5_6_7_IRQHandler(void) {
    USARTx_DMA_handler(0);
    USARTx_DMA_handler(1);
}

void ADC1_COMP_IRQHandler(void) {Error_Handler();}
void USART4_5_IRQHandler(void) {Error_Handler();}
//void TIM2_IRQHandler(void) {Error_Handler();}
void TIM3_IRQHandler(void) {Error_Handler();}
void TIM6_IRQHandler(void) {Error_Handler();}
void TIM7_IRQHandler(void) {Error_Handler();}
void TIM21_IRQHandler(void) {Error_Handler();}
void I2C3_IRQHandler(void) {Error_Handler();}
void TIM22_IRQHandler(void) {Error_Handler();}
void I2C1_IRQHandler(void) {Error_Handler();}
void I2C2_IRQHandler(void) {Error_Handler();}
void SPI1_IRQHandler(void) {Error_Handler();}
void SPI2_IRQHandler(void) {Error_Handler();}
void USART1_IRQHandler(void)
{
    USART_IDLE_IRQ_handler(0);
}
void USART2_IRQHandler(void)
{
    USART_IDLE_IRQ_handler(1);
}
void LPUART1_IRQHandler(void) {Error_Handler();}

} // extern

#endif /* STM32L0xx */
