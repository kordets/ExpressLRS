#include "HwSpi.h"
#include "targets.h"

HwSpi::HwSpi() : SPIClass()
{
    SS = GPIO_PIN_NSS;
}

void HwSpi::platform_init(uint32_t speed, int sck, int miso, int mosi)
{
    SPIClass::begin(); // TODO: check pins!
    setBitOrder(MSBFIRST);
    setDataMode(SPI_MODE0);
    setFrequency(speed);
}

void HwSpi::write(uint8_t data)
{
    SPIClass::write(data);
}

void HwSpi::write(uint8_t *data, uint8_t numBytes)
{
    SPIClass::writeBytes((uint8_t *)data, numBytes);
}

HwSpi RadioSpi;
