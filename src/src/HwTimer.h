#pragma once

#include "platform.h"
#include <stdint.h>

#define TimerIntervalUSDefault 20000

#define TIMER_SOON 40 // 40us

#if (PLATFORM_STM32 && !defined(ARDUINO)) || PLATFORM_ESP8266
#define USE_TIMER_KICK  1
#endif

//#ifdef STM32L0xx
#define TIMER_OFFSET_KICK       100
//#else
//#define TIMER_OFFSET_KICK       200
//#endif

#define TIMER_OFFSET       250
#define TIMER_OFFSET_LIMIT 50

class HwTimer
{
public:
    HwTimer();
    void init();
    void FAST_CODE_1 start();
    void FAST_CODE_1 reset(int32_t offset = 0);
    void FAST_CODE_1 pause();
    void FAST_CODE_1 stop();
    void FAST_CODE_1 updateInterval(uint32_t newTimerInterval);
    bool FAST_CODE_1 isRunning(void)
    {
        return running;
    }

    void FAST_CODE_1 callback();

    void (*callbackTockPre)(uint32_t us);
    void (*callbackTock)(uint32_t us);

    void FAST_CODE_1 setTime(uint32_t time = 0);

    void FAST_CODE_1 triggerSoon(void);

private:
    uint32_t HWtimerInterval;
    bool running = false;
};

extern HwTimer TxTimer;
