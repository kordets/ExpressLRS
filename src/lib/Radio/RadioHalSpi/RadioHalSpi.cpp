#include "RadioHalSpi.h"
#include "platform.h"

void RadioHalSpi::Begin(uint32_t speed, int sck, int miso, int mosi, int cs)
{
    CS = gpio_out_setup(cs, 1);
    spi_bus = spi_setup(speed, sck, miso, mosi, 0);
    spi_prepare(spi_bus);
}

uint8_t ICACHE_RAM_ATTR
RadioHalSpi::readRegister(uint8_t reg) const
{
    uint8_t data[] = {(uint8_t)(reg | p_read), 0};
    gpio_out_write(CS, 0);
    spi_transfer(spi_bus, 1, sizeof(data), data);
    gpio_out_write(CS, 1);
    return data[1];
}

void ICACHE_RAM_ATTR
RadioHalSpi::writeRegister(uint8_t reg, uint8_t data) const
{
    uint8_t _data[] = {(uint8_t)(reg | p_write), data};
    gpio_out_write(CS, 0);
    spi_transfer(spi_bus, 0, sizeof(data), _data);
    gpio_out_write(CS, 1);
}

void ICACHE_RAM_ATTR
RadioHalSpi::readRegisterBurst(uint8_t reg, uint8_t numBytes, uint8_t *inBytes) const
{
    uint8_t _data = reg | p_read;
    gpio_out_write(CS, 0);
    spi_transfer(spi_bus, 0, 1, &_data);
    spi_transfer(spi_bus, 1, numBytes, inBytes);
    gpio_out_write(CS, 1);
}

void ICACHE_RAM_ATTR
RadioHalSpi::writeRegisterBurst(uint8_t reg, uint8_t *data, uint8_t numBytes) const
{
    uint8_t _data = reg | p_write;
    gpio_out_write(CS, 0);
    spi_transfer(spi_bus, 0, 1, &_data);
    spi_transfer(spi_bus, 0, numBytes, data);
    gpio_out_write(CS, 1);
}

void ICACHE_RAM_ATTR
RadioHalSpi::readRegisterAddr(uint8_t reg, uint16_t addr,
                              uint8_t *data, uint8_t numBytes) const
{
    uint8_t hdr[] = {(uint8_t)(reg | p_write), (uint8_t)(addr >> 8), (uint8_t)addr, 0};
    gpio_out_write(CS, 0);
    spi_transfer(spi_bus, 0, sizeof(hdr), hdr);
    spi_transfer(spi_bus, 1, numBytes, data);
    gpio_out_write(CS, 1);
}

void ICACHE_RAM_ATTR
RadioHalSpi::writeRegisterAddr(uint8_t reg, uint16_t addr,
                               uint8_t *data, uint8_t numBytes) const
{
    uint8_t hdr[] = {(uint8_t)(reg | p_write), (uint8_t)(addr >> 8), (uint8_t)addr};
    gpio_out_write(CS, 0);
    spi_transfer(spi_bus, 0, sizeof(hdr), hdr);
    spi_transfer(spi_bus, 0, numBytes, data);
    gpio_out_write(CS, 1);
}

void ICACHE_RAM_ATTR
RadioHalSpi::readRegisterOffset(uint8_t reg, uint8_t offset,
                                uint8_t *data, uint8_t numBytes) const
{
    uint8_t hdr[] = {(uint8_t)(reg | p_write), offset, 0};
    gpio_out_write(CS, 0);
    spi_transfer(spi_bus, 0, sizeof(hdr), hdr);
    spi_transfer(spi_bus, 1, numBytes, data);
    gpio_out_write(CS, 1);
}

void ICACHE_RAM_ATTR
RadioHalSpi::writeRegisterOffset(uint8_t reg, uint8_t offset,
                                 uint8_t *data, uint8_t numBytes) const
{
    uint8_t hdr[] = {(uint8_t)(reg | p_write), offset};
    gpio_out_write(CS, 0);
    spi_transfer(spi_bus, 0, sizeof(hdr), hdr);
    spi_transfer(spi_bus, 0, numBytes, data);
    gpio_out_write(CS, 1);
}
