#include "Servo.h"
#include "internal.h"
#include "irq.h"
#include "priorities.h"
#include "hal_inc.h"
#include <Arduino.h>

#if defined(TARGET_R9M_RX)

#include <stm32f1xx_ll_tim.h>
//#include <stm32l0xx_ll_tim.h>

#ifdef TIM1
#define TIMx            TIM1
#define TIMx_IRQn       TIM1_IRQn
#elif defined(TIM6)
#define TIMx            TIM6
#define TIMx_IRQn       TIM6_IRQn
#endif

uint8_t TIM1_pin_to_ch[] = {PA8, PA9, PA10, PA11};


static uint8_t timer_initialized;
static void servo_timer_init(void)
{
    LL_TIM_InitTypeDef TIM_InitStruct;
    if (timer_initialized) return;

    /* Peripheral clock enable */
    enable_pclock((uint32_t)TIMx);

    // microsecond timer
    TIM_InitStruct.Prescaler = (2 * get_pclock_frequency((uint32_t)TIMx) / 1000000) - 1;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 20000; // 20ms
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.RepetitionCounter = 0;
    LL_TIM_Init(TIMx, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIMx);
    LL_TIM_SetClockSource(TIMx, LL_TIM_CLOCKSOURCE_INTERNAL);
    LL_TIM_DisableMasterSlaveMode(TIMx);
    LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);

    timer_initialized = 1;
}


static void TIMx_Channel_Init(uint32_t channel, uint16_t val=1500)
{
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct;

    servo_timer_init();

    channel = 0x1 << (4 * channel);

    /* TODO: Config GPIO output to AF_PUSHPULL mode! */

    /* Compare channels uses preload */
    LL_TIM_OC_EnablePreload(TIMx, channel);

    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.CompareValue = val;
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
    TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
    LL_TIM_OC_Init(TIMx, channel, &TIM_OC_InitStruct);
    //LL_TIM_OC_DisableFast(TIMx, channel);
    //LL_TIM_CC_EnableChannel(TIMx, channel);
}


void Servo::attach(int pin, uint16_t min, uint16_t max)
{
    uint8_t ch;
    reg = NULL;
    for (ch = 0; ch < sizeof(TIM1_pin_to_ch); ch++) {
        if (TIM1_pin_to_ch[ch] == pin)
            break;
    }
    if (sizeof(TIM1_pin_to_ch) <= ch)
        return;

    _min = min;
    _max = max;

    min = (min + max) / 2;
    TIMx_Channel_Init(ch, min);

    switch(ch) {
        case 0:
            reg = (void*)&TIMx->CCR1;
            break;
        case 1:
            reg = (void*)&TIMx->CCR2;
            break;
        case 2:
            reg = (void*)&TIMx->CCR3;
            break;
        case 3:
            reg = (void*)&TIMx->CCR4;
            break;
    }
}

void Servo::writeMicroseconds(uint32_t us)
{
    if (!reg) return;
    /* limit input to range */
    us = (us < _min) ? _min : (_max < us) ? _max : us;
    // set channel...
    *(__IO uint32_t *)reg = us;
    //LL_TIM_OC_SetCompareCH1((__IO uint32_t *)reg, us);
}

#endif /* TARGET_R9M_RX */
