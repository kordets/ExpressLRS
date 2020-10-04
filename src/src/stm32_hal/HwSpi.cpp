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
    SPIClass::Begin(speed, MOSI, MISO, SCK, SS);
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
