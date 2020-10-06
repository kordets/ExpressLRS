#include "HwSpi.h"
#include "targets.h"
#include <Arduino.h>

HwSpi::HwSpi() : SPIClass()
{
}

void HwSpi::platform_init(uint32_t speed, int sck, int miso, int mosi)
{
    // sck, miso, mosi, ss (ss can be any GPIO)
    SPIClass::begin(sck, miso, mosi, -1);
    SPIClass::setDataMode(SPI_MODE0); // mode0 by default
    SPIClass::setFrequency(speed);
}

void ICACHE_RAM_ATTR HwSpi::write(uint8_t data)
{
    taskDISABLE_INTERRUPTS();
    SPIClass::write(data);
    taskENABLE_INTERRUPTS();
}

void ICACHE_RAM_ATTR HwSpi::write(uint8_t *data, uint8_t numBytes)
{
    taskDISABLE_INTERRUPTS();
    SPIClass::writeBytes((uint8_t *)data, numBytes);
    taskENABLE_INTERRUPTS();
}

HwSpi RadioSpi;
