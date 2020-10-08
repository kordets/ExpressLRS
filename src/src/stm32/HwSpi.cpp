#include "gpio.h"
#include <Arduino.h>
#include <SPI.h>

/************************/
SPIClass SpiBus;

struct spi_config
spi_setup(uint32_t speed, int sck, int miso, int mosi, uint8_t mode)
{
    (void)speed;
    SpiBus.setMOSI(mosi);
    SpiBus.setMISO(miso);
    SpiBus.setSCLK(sck);
    SpiBus.setBitOrder(MSBFIRST);
    SpiBus.setDataMode(mode);
    SpiBus.begin();
    SpiBus.setClockDivider(SPI_CLOCK_DIV4); // 72 / 8 = 9 MHz //not correct for SPI2

    return {.spi = &SpiBus};
}

void spi_prepare(struct spi_config config)
{
    (void)config;
}

void spi_transfer(struct spi_config config, uint8_t receive_data,
                  uint8_t len, uint8_t *data)
{
    SPIClass * spi = (SPIClass*)config.spi;
    spi->transfer(data, len);
}
