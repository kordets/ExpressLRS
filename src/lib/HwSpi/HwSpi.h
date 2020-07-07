#pragma once

#include <Arduino.h>
#include <SPI.h>

class HwSpi : public SPIClass
{
public:
    HwSpi();

    void prepare(uint32_t speed)
    {
        pinMode(MOSI, OUTPUT);
        pinMode(MISO, INPUT);
        pinMode(SCK, OUTPUT);
        pinMode(SS, OUTPUT);
        digitalWrite(SS, HIGH);
        platform_init(speed);
    }

    void set_ss(uint8_t state)
    {
        digitalWrite(SS, state);
    }

    void ICACHE_RAM_ATTR write(uint8_t data);
    void ICACHE_RAM_ATTR write(uint8_t *data, uint8_t numBytes);

private:
    int SS, MOSI, MISO, SCK;

    void platform_init(uint32_t speed);
};

extern HwSpi RadioSpi;
