#include "HwTimer.h"
#include "targets.h"
#include "debug_elrs.h"
#include <user_interface.h>
#include <Arduino.h>

HwTimer TxTimer;

void ICACHE_RAM_ATTR MyTimCallback(void)
{
    TxTimer.callback();
}

void HwTimer::init()
{
    noInterrupts();
    timer1_attachInterrupt(MyTimCallback);
    running = false;
    interrupts();
}

void ICACHE_RAM_ATTR HwTimer::start()
{
    if (running)
        return;
    running = true;
    reset(0);
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP); //5MHz ticks
}

void ICACHE_RAM_ATTR HwTimer::stop()
{
    running = false;
    timer1_disable();
}

void ICACHE_RAM_ATTR HwTimer::pause()
{
    stop();
}

void ICACHE_RAM_ATTR HwTimer::reset(int32_t offset)
{
    if (running)
    {
        setTime(HWtimerInterval - offset);
    }
}

void ICACHE_RAM_ATTR HwTimer::setTime(uint32_t time)
{
    if (!time)
        time = HWtimerInterval;
    timer1_write(time * 5);
}

void ICACHE_RAM_ATTR HwTimer::triggerSoon(void)
{
    timer1_write(TIMER_SOON * 5);
}
