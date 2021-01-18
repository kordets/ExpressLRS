#include "HwTimer.h"
#include "debug_elrs.h"
#include "platform.h"

/* HW specific code can be found from <mcu type>/ folder */

static void nullCallback(uint32_t){};

HwTimer::HwTimer()
{
    callbackTockPre = nullCallback;
    callbackTock = nullCallback;

    HWtimerInterval = TimerIntervalUSDefault;
}

void ICACHE_RAM_ATTR HwTimer::updateInterval(uint32_t newTimerInterval)
{
    HWtimerInterval = newTimerInterval;
    //setTime(newTimerInterval);
}

void ICACHE_RAM_ATTR HwTimer::callback()
{
    uint32_t us = micros();
    callbackTockPre(us);
    callbackTock(us);
}
