#include "Servo.h"
#include "internal.h"
#include "irq.h"
#include "priorities.h"
#include <Arduino.h>

#ifdef TIM4
#define TIMx            TIM4
#define TIMx_IRQn       TIM4_IRQn
//#define TIMx_IRQx_FUNC  TIM4_IRQHandler
#elif defined(TIM6)
#define TIMx            TIM6
#define TIMx_IRQn       TIM6_IRQn
//#define TIMx_IRQx_FUNC  TIM6_IRQHandler
#endif

#ifdef TIMx_IRQx_FUNC
// Hardware timer IRQ handler - dispatch software timers
extern "C"
void TIMx_IRQx_FUNC(void)
{
    uint16_t SR = TIMx->SR;
    if (SR & TIM_SR_UIF) {
        TIMx->SR = SR & ~(TIM_SR_UIF);
    }
}
#endif // TIMx_IRQx_FUNC


void Servo::attach(int pin, int min, int max)
{
    enable_pclock((uint32_t)TIMx);

    // map pin to channel...

#ifdef TIMx_IRQx_FUNC
    NVIC_SetPriority(TIMx_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), ISR_PRIO_TIM, 0));
    NVIC_EnableIRQ(TIMx_IRQn);
#endif // TIMx_IRQx_FUNC
}

void Servo::writeMicroseconds(uint32_t us)
{

}
