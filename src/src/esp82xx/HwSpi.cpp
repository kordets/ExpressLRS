#include "HwSpi.h"
#include "targets.h"

HwSpi::HwSpi() : SPIClass()
{
    SS = GPIO_PIN_NSS;
    MOSI = GPIO_PIN_MOSI;
    MISO = GPIO_PIN_MISO;
    SCK = GPIO_PIN_SCK;
}

void HwSpi::platform_init(uint32_t speed)
{
    SPIClass::begin();
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
