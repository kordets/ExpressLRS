#include "platform.h"
#include <Arduino.h>
#include <SPI.h>

SPIClass DRAM_ATTR SpiBus;

struct spi_config IRAM_ATTR
spi_setup(uint32_t speed, int sck, int miso, int mosi, uint8_t mode)
{
    SpiBus.begin(sck, miso, mosi, -1);
    SpiBus.setDataMode(mode);
    SpiBus.setFrequency(speed);
    return {.spi = &SpiBus};
}

void IRAM_ATTR
spi_prepare(struct spi_config config)
{
    (void)config;
}

void IRAM_ATTR
spi_transfer(struct spi_config config, uint8_t receive_data,
             uint8_t len, uint8_t *data)
{
    SPIClass * spi = (SPIClass*)config.spi;
    taskDISABLE_INTERRUPTS();
    if (receive_data)
        spi->transfer(data, len);
    else
        spi->writeBytes(data, len);
    taskENABLE_INTERRUPTS();
}
