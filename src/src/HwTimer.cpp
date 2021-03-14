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

void FAST_CODE_1 HwTimer::updateInterval(uint32_t newTimerInterval)
{
    HWtimerInterval = newTimerInterval;
    //setTime(newTimerInterval);
}

void FAST_CODE_1 HwTimer::callback()
{
    uint32_t us = micros();
    callbackTockPre(us);
    callbackTock(us);
}
