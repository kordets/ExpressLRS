#pragma once

#include "platform.h"
#include <stdint.h>

#define PRINT_TIMER 0
#define PRINT_HW_ISR 0
#define PRINT_RX_ISR 0
#define PRINT_TMR 0

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
    void ICACHE_RAM_ATTR start();
    void ICACHE_RAM_ATTR reset(int32_t offset = 0);
    void ICACHE_RAM_ATTR pause();
    void ICACHE_RAM_ATTR stop();
    void ICACHE_RAM_ATTR updateInterval(uint32_t newTimerInterval);
    bool ICACHE_RAM_ATTR isRunning(void)
    {
        return running;
    }

    void ICACHE_RAM_ATTR callback();

    void (*callbackTock)(uint32_t us);

    void ICACHE_RAM_ATTR setTime(uint32_t time = 0);

    void ICACHE_RAM_ATTR triggerSoon(void);

private:
    uint32_t HWtimerInterval;
    bool running = false;
};

extern HwTimer TxTimer;
