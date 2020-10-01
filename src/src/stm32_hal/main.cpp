#include "Arduino.h"
#include "stm32_def.h"
#include "debug_elrs.h"

extern void setup(void);
extern void loop(void);
void yield(void);

uint32_t _ms_cntr;

#ifdef __cplusplus
extern "C" {
#endif

void __libc_init_array(void);

#ifdef __cplusplus
}
#endif

void shutdown(const char * reason)
{
    DEBUG_PRINTF("F**k! %s\n", reason);
    while(1);
}

void _Error_Handler(const char * error, int line)
{
    shutdown(error);
    (void)line;
}

// Enable a peripheral clock
void enable_pclock(uint32_t periph_base)
{
    if (periph_base < APB2PERIPH_BASE) {
        uint32_t pos = (periph_base - APB1PERIPH_BASE) / 0x400;
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
int is_enabled_pclock(uint32_t periph_base)
{
    if (periph_base < APB2PERIPH_BASE) {
        uint32_t pos = (periph_base - APB1PERIPH_BASE) / 0x400;
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
    if (periph_base < APB2PERIPH_BASE) {
        // APB1 is configured to F_CPU / 2
        return (CONFIG_CLOCK_FREQ / 2);
    }
    return CONFIG_CLOCK_FREQ;
}

// Enable a GPIO peripheral clock
void gpio_clock_enable(GPIO_TypeDef *regs)
{
    uint32_t rcc_pos = ((uint32_t)regs - APB2PERIPH_BASE) / 0x400;
    RCC->APB2ENR |= 1 << rcc_pos;
    RCC->APB2ENR;
}

// Set the mode and extended function of a pin
void gpio_peripheral(uint32_t gpio, uint32_t mode, int pullup)
{
    GPIO_TypeDef *regs = digital_regs[GPIO2PORT(gpio)];

    // Enable GPIO clock
    gpio_clock_enable(regs);

    // Configure GPIO
    uint32_t pos = gpio % 16, shift = (pos % 8) * 4, msk = 0xf << shift, cfg;
    if (mode == GPIO_INPUT) {
        cfg = pullup ? 0x8 : 0x4;
    } else if (mode == GPIO_OUTPUT) {
        cfg = 0x1; // push-pull, 0b00 | max speed 2 MHz, 0b01
    } else if (mode == (GPIO_OUTPUT | GPIO_OPEN_DRAIN)) {
        cfg = 0x5; // Open-drain, 0b01 | max speed 2 MHz, 0b01
    } else if (mode == GPIO_ANALOG) {
        cfg = 0x0;
    } else {
        // Alternate function
        if (mode & GPIO_OPEN_DRAIN)
            // output open-drain mode, 10MHz
            cfg = 0xd;
        else if (pullup > 0)
            // input pins use GPIO_INPUT mode on the stm32f1
            cfg = 0x8;
        else
            // output push-pull mode, 10MHz
            cfg = 0x9;
    }
    if (pos & 0x8)
        regs->CRH = (regs->CRH & ~msk) | (cfg << shift);
    else
        regs->CRL = (regs->CRL & ~msk) | (cfg << shift);

    if (pullup > 0)
        regs->BSRR = 1 << pos;
    else if (pullup < 0)
        regs->BSRR = 1 << (pos + 16);

    if (gpio == GPIO('A', 13) || gpio == GPIO('A', 14))
        // Disable SWD to free PA13, PA14
        AFIO->MAPR = AFIO_MAPR_SWJ_CFG_DISABLE;
}


// Return the current time (in absolute clock ticks).
uint32_t timer_read_time(void)
{
    return DWT->CYCCNT;
}

// Return true if time1 is before time2.  Always use this function to
// compare times as regular C comparisons can fail if the counter
// rolls over.
uint8_t timer_is_before(uint32_t time1, uint32_t time2)
{
    return (int32_t)(time1 - time2) < 0;
}

uint32_t micros(void)
{
    return clockCyclesToMicroseconds(timer_read_time());
}

uint32_t millis(void)
{
    uint32_t ms = read_u32(&_ms_cntr);
    return ms;
}

void delay(uint32_t ms)
{
    uint32_t start = millis();
    //ms += millis();
    //while (timer_is_before(millis(), ms))
    //    ;
    while((millis() - start) < ms);
}

void delayMicroseconds(uint32_t usecs)
{
    #if 0
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
    #endif

    //uint32_t end = timer_read_time() + microsecondsToClockCycles(usecs);
    //while (timer_is_before(timer_read_time(), end))
    //    ;
    usecs = microsecondsToClockCycles(usecs);
    uint32_t start = timer_read_time();
    while ((timer_read_time() - start) < usecs);
}

void HAL_IncTick(void)
{
    // just empty to avoid issue with HAL implementation
}

uint32_t HAL_GetTick(void)
{
    return millis();
}

void HAL_Delay(uint32_t Delay)
{
    delay(Delay);
}

static void timer_init(void)
{
    // Enable Debug Watchpoint and Trace (DWT) for its 32bit timer
    /*
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
    */

    // Enable SysTick
    write_u32(&_ms_cntr, 0);
    NVIC_SetPriority(SysTick_IRQn, 2);
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
static void SystemClock_Config(void)
{
    uint32_t FLatency = FLASH_LATENCY_2; // delay = (CPU CLK / 24MHz)
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /* Initializes the CPU, AHB and APB busses clocks */
#if !USE_INTERNAL_XO
    //
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9; // 8MHz * 9 = 72MHz
#else // USE_INTERNAL_XO
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; // HSI = 8MHz
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2; // 8MHz / 2 = 4MHz
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16; // 4MHz * 16 = 64MHz
#endif // !USE_INTERNAL_XO
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /* Initializes the CPU, AHB and APB busses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                    | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLatency) != HAL_OK) {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_USB;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }

    SystemCoreClockUpdate();
}

#ifdef DWT_BASE
static void dwt_access(bool ena)
{
#if (__CORTEX_M == 0x07U)
    /*
    * Define DWT LSR mask which is (currentuly) not defined by the CMSIS.
    * Same as ITM LSR one.
    */
#if !defined DWT_LSR_Present_Msk
#define DWT_LSR_Present_Msk ITM_LSR_Present_Msk
#endif
#if !defined DWT_LSR_Access_Msk
#define DWT_LSR_Access_Msk ITM_LSR_Access_Msk
#endif
    uint32_t lsr = DWT->LSR;

    if ((lsr & DWT_LSR_Present_Msk) != 0) {
        if (ena) {
        if ((lsr & DWT_LSR_Access_Msk) != 0) { //locked
            DWT->LAR = 0xC5ACCE55;
        }
        } else {
        if ((lsr & DWT_LSR_Access_Msk) == 0) { //unlocked
            DWT->LAR = 0;
        }
        }
    }
#else /* __CORTEX_M */
    UNUSED(ena);
#endif /* __CORTEX_M */
}

static uint32_t dwt_init(void)
{

    /* Enable use of DWT */
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }

    /* Unlock */
    dwt_access(true);

    /* Reset the clock cycle counter value */
    DWT->CYCCNT = 0;

    /* Enable  clock cycle counter */
    DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk;

    /* 3 NO OPERATION instructions */
    __asm volatile(" nop      \n\t"
                   " nop      \n\t"
                   " nop      \n\t");

    /* Check if clock cycle counter has started */
    return (DWT->CYCCNT) ? 0 : 1;
}
#endif /* DWT_BASE */

static void init(void)
{
    /* Deinit HAL just in case => reset all peripherals */
    HAL_DeInit();

    /* Init DWT if present */
#ifdef DWT_BASE
    if (dwt_init()) {
        Error_Handler();
    }
#endif

    /* Configure the system clock */
    SystemClock_Config();

    timer_init();

    // Enable AFIO peripheral
    enable_pclock((uint32_t)AFIO_BASE);

#if defined (STM32MP1xx)
    __HAL_RCC_HSEM_CLK_ENABLE();
#endif
}

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need HAL may fail.
__attribute__((constructor(101))) void premain()
{
    /* Reset vector location which is set wrongly by SystemInit */
    extern uint32_t g_pfnVectors;
    SCB->VTOR = (uint32_t) &g_pfnVectors;

/* Configure Flash prefetch */
#if (PREFETCH_ENABLE != 0)
#if defined(STM32F101x6) || defined(STM32F101xB) || defined(STM32F101xE) || defined(STM32F101xG) || \
    defined(STM32F102x6) || defined(STM32F102xB) || \
    defined(STM32F103x6) || defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
    defined(STM32F105xC) || defined(STM32F107xC)

  /* Prefetch buffer is not available on value line devices */
  __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
#endif
#endif /* PREFETCH_ENABLE */

    // Required by FreeRTOS, see http://www.freertos.org/RTOS-Cortex-M3-M4.html
#ifdef NVIC_PRIORITYGROUP_4
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
#endif
#if (__CORTEX_M == 0x07U)
    // Defined in CMSIS core_cm7.h
#ifndef I_CACHE_DISABLED
    SCB_EnableICache();
#endif
#ifndef D_CACHE_DISABLED
    SCB_EnableDCache();
#endif
#endif

    init();
}

extern void initVariant() __attribute__((weak));

/*
 * \brief Main entry point of Arduino application
 */
int main(void)
{
    initVariant();
    setup();
    for (;;) {
        loop();
    }

    return 0;
}

#ifdef __cplusplus
extern "C" {
#endif

void SysTick_Handler(void)
{
    ++_ms_cntr;
}

void NMI_Handler(void) {}
void HardFault_Handler(void) {Error_Handler();}
void MemManage_Handler(void) {Error_Handler();}
void BusFault_Handler(void) {Error_Handler();}
void UsageFault_Handler(void) {Error_Handler();}
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}
// SysTick_Handler
void WWDG_IRQHandler(void) {Error_Handler();}
void PVD_IRQHandler(void) {Error_Handler();}
void TAMPER_IRQHandler(void) {Error_Handler();}
void RTC_IRQHandler(void) {Error_Handler();}
void FLASH_IRQHandler(void) {Error_Handler();}
void RCC_IRQHandler(void) {Error_Handler();}
void DMA1_Channel1_IRQHandler(void) {Error_Handler();}
//void DMA1_Channel2_IRQHandler(void) {Error_Handler();}
void DMA1_Channel3_IRQHandler(void) {Error_Handler();}
//void DMA1_Channel4_IRQHandler(void) {Error_Handler();}
void DMA1_Channel5_IRQHandler(void) {Error_Handler();}
//void DMA1_Channel6_IRQHandler(void) {Error_Handler();}
void DMA1_Channel7_IRQHandler(void) {Error_Handler();}
void ADC1_2_IRQHandler(void) {Error_Handler();}
void USB_HP_CAN1_TX_IRQHandler(void) {Error_Handler();}
void USB_LP_CAN1_RX0_IRQHandler(void) {Error_Handler();}
void CAN1_RX1_IRQHandler(void) {Error_Handler();}
void CAN1_SCE_IRQHandler(void) {Error_Handler();}
void TIM1_BRK_IRQHandler(void) {Error_Handler();}
void TIM1_UP_IRQHandler(void) {Error_Handler();}
void TIM1_TRG_COM_IRQHandler(void) {Error_Handler();}
void TIM1_CC_IRQHandler(void) {Error_Handler();}
//void TIM2_IRQHandler(void) {Error_Handler();}
void TIM3_IRQHandler(void) {Error_Handler();}
void TIM4_IRQHandler(void) {Error_Handler();}
void I2C1_EV_IRQHandler(void) {Error_Handler();}
void I2C1_ER_IRQHandler(void) {Error_Handler();}
void I2C2_EV_IRQHandler(void) {Error_Handler();}
void I2C2_ER_IRQHandler(void) {Error_Handler();}
void SPI1_IRQHandler(void) {Error_Handler();}
void SPI2_IRQHandler(void) {Error_Handler();}
void RTC_Alarm_IRQHandler(void) {Error_Handler();}
void USBWakeUp_IRQHandler(void) {Error_Handler();}

#ifdef __cplusplus
}
#endif
