#include "HwTimer.h"
#include "internal.h"
#include "irq.h"

#include <Arduino.h>

#define TIMER_IS_2US 0


HwTimer TxTimer;

/****************************************************************
 * Low level timer code
 ****************************************************************/

#ifdef TIM2
#define TIMx TIM2
#define TIMx_IRQn TIM2_IRQn
#define HAVE_TIMER_32BIT 1
#define TIMx_IRQx_FUNC TIM2_IRQHandler
#else
#define TIMx TIM3
#define TIMx_IRQn TIM3_IRQn
#define HAVE_TIMER_32BIT 0
#define TIMx_IRQx_FUNC TIM3_IRQHandler
#endif
// originally TIM1;

static inline uint32_t timer_counter_get(void)
{
    return TIMx->CNT;
}

static inline void timer_counter_set(uint32_t cnt)
{
    TIMx->CNT = cnt;
}

static inline void timer_set(uint32_t next)
{
    TIMx->CCR1 = next >> TIMER_IS_2US;
    TIMx->SR = 0;
}

/****************************************************************
 * HW Timer setup and irqs
 ****************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif
    // Hardware timer IRQ handler - dispatch software timers
    void TIMx_IRQx_FUNC(void)
    {
        uint16_t SR = TIMx->SR;
        if (SR & TIM_SR_UIF) {
            TIMx->SR = SR & ~(TIM_SR_UIF);
            //irqstatus_t flag = irq_save();
            TxTimer.callback();
            //irq_restore(flag);
        }
    }
#ifdef __cplusplus
}
#endif

void timer_enable(void)
{
    TIMx->CR1 = TIM_CR1_CEN;
}

void timer_disable(void)
{
    TIMx->CR1 = 0;
    TIMx->SR  &= ~(TIM_SR_UIF);
}

static void timer_init(void)
{
    irqstatus_t flag = irq_save();
    enable_pclock((uint32_t)TIMx);
    timer_disable();
    // Set clock prescaler to 1us or 2us
    TIMx->PSC = (get_pclock_frequency((uint32_t)TIMx) / (1000000 >> TIMER_IS_2US)) - 1;
    TIMx->ARR = (1U << 16) - 1; // Init to max
    TIMx->CNT = 0;
    //TIMx->RCR = 0;
    TIMx->EGR = TIM_EGR_UG;
    TIMx->DIER = TIM_IT_UPDATE;
    NVIC_SetPriority(TIMx_IRQn, 2);
    NVIC_EnableIRQ(TIMx_IRQn);
    irq_restore(flag);
}

/****************************************************************
 * Public
 ****************************************************************/
void HwTimer::init()
{
    timer_disable();
    timer_init();
    //timer_enable();
}

void HwTimer::start()
{
    timer_set(HWtimerInterval);
    timer_enable();
}

void HwTimer::stop()
{
    timer_disable();
}

void HwTimer::pause()
{
    timer_disable();
}

void HwTimer::reset(int32_t offset)
{
    if (running)
    {
        timer_counter_set(0);
        timer_set(HWtimerInterval - offset);
    }
}

void HwTimer::setTime(uint32_t time)
{
    if (!time)
        time = HWtimerInterval;
    timer_set(time);
}
