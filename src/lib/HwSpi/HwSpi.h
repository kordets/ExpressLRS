#pragma once

#include <Arduino.h>
#include <SPI.h>

class HwSpi : public SPIClass
{
public:
    HwSpi();

    void prepare(uint32_t speed, int sck, int miso, int mosi, int ss)
    {
        SS = ss;
        pinMode(ss, OUTPUT);
        digitalWrite(ss, HIGH);
        platform_init(speed, sck, miso, mosi);
    }

    void set_ss(uint8_t state)
    {
        digitalWrite(SS, state);
    }

    void ICACHE_RAM_ATTR write(uint8_t data);
    void ICACHE_RAM_ATTR write(uint8_t *data, uint8_t numBytes);

private:
    int SS;

    void platform_init(uint32_t speed, int sck, int miso, int mosi);
};

extern HwSpi RadioSpi;
