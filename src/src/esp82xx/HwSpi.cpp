#include "HwSpi.h"
#include "targets.h"
#include <Arduino.h>

HwSpi::HwSpi() : SPIClass()
{
}

void HwSpi::platform_init(uint32_t speed, int sck, int miso, int mosi)
{
    if (SPIClass::pins(sck, miso, mosi, -1)) {
        SPIClass::begin();
        SPIClass::setHwCs(false);
        SPIClass::setBitOrder(MSBFIRST);
        SPIClass::setDataMode(SPI_MODE0);
        SPIClass::setFrequency(speed);
    } else {
        /* TODO: Error handling!! */
    }
}

void ICACHE_RAM_ATTR HwSpi::write(uint8_t data)
{
    SPIClass::write(data);
}

void ICACHE_RAM_ATTR HwSpi::write(uint8_t *data, uint8_t numBytes)
{
    SPIClass::writeBytes((uint8_t *)data, numBytes);
}

HwSpi RadioSpi;
