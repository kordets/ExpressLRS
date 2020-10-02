#include "Arduino.h"
#include "stm32_def.h"
#include "debug_elrs.h"

// -------------------------------------------
// Generic time methods

uint32_t _ms_cntr;

extern "C" void SysTick_Handler(void)
{
    ++_ms_cntr;
}

// Return true if time1 is before time2.  Always use this function to
// compare times as regular C comparisons can fail if the counter
// rolls over.
uint8_t timer_is_before(uint32_t time1, uint32_t time2)
{
    return (int32_t)(time1 - time2) < 0;
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

// -------------------------------------------
// weak HAL methods overwrite

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

// -------------------------------------------
// Error handlers

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

// -------------------------------------------

