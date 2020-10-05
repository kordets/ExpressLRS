#include "HwSpi.h"
#include "targets.h"

HwSpi::HwSpi() : SPIClass()
{
}

void HwSpi::platform_init(uint32_t speed, int sck, int miso, int mosi)
{
    SPIClass::Begin(speed, mosi, miso, sck);
}

void HwSpi::write(uint8_t data)
{
    SPIClass::transfer(data);
}

void HwSpi::write(uint8_t *data, uint8_t numBytes)
{
    SPIClass::transfer((uint8_t *)data, numBytes);
}

HwSpi RadioSpi;
