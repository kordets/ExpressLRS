#include "HwSpi.h"
#include "targets.h"

HwSpi::HwSpi() : SPIClass()
{
}

void HwSpi::platform_init(uint32_t speed, int sck, int miso, int mosi)
{
    (void)speed; // TODO take into use
    setMOSI(mosi);
    setMISO(miso);
    setSCLK(sck);
    setBitOrder(MSBFIRST);
    setDataMode(SPI_MODE0);
    SPIClass::begin();
    setClockDivider(SPI_CLOCK_DIV4); // 72 / 8 = 9 MHz //not correct for SPI2
}

void ICACHE_RAM_ATTR HwSpi::write(uint8_t data)
{
    SPIClass::transfer(data);
}

void ICACHE_RAM_ATTR HwSpi::write(uint8_t *data, uint8_t numBytes)
{
    SPIClass::transfer((uint8_t *)data, numBytes);
}

HwSpi RadioSpi;
