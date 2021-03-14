#include "RadioHalSpi.h"
#include "platform.h"

void RadioHalSpi::Begin(uint32_t speed, int sck, int miso, int mosi)
{
    if (!spi_is_valid(spi_bus)) {
        spi_bus = spi_setup(speed, sck, miso, mosi, 0);
    }
    gpio_out_write(CS, 1);
    spi_prepare(spi_bus);
}

void FAST_CODE_1
RadioHalSpi::transfer(uint8_t *data, uint8_t len, uint8_t receive) const
{
    gpio_out_write(CS, 0);
    spi_transfer(spi_bus, receive, len, data);
    gpio_out_write(CS, 1);
}

uint8_t FAST_CODE_1
RadioHalSpi::readRegister(uint8_t reg) const
{
    uint8_t _data[] = {(uint8_t)(reg | p_read), 0};
    gpio_out_write(CS, 0);
    spi_transfer(spi_bus, 1, sizeof(_data), _data);
    gpio_out_write(CS, 1);
    return _data[1];
}

void FAST_CODE_1
RadioHalSpi::writeRegister(uint8_t reg, uint8_t data) const
{
    uint8_t _data[] = {(uint8_t)(reg | p_write), data};
    gpio_out_write(CS, 0);
    spi_transfer(spi_bus, 0, sizeof(_data), _data);
    gpio_out_write(CS, 1);
}

void FAST_CODE_1
RadioHalSpi::readRegisterBurst(uint8_t reg, uint8_t numBytes, uint8_t *inBytes) const
{
    uint8_t _data = reg | p_read;
    gpio_out_write(CS, 0);
    spi_transfer(spi_bus, 0, 1, &_data);
    spi_transfer(spi_bus, 1, numBytes, inBytes);
    gpio_out_write(CS, 1);
}

void FAST_CODE_1
RadioHalSpi::writeRegisterBurst(uint8_t reg, uint8_t *data, uint8_t numBytes) const
{
    uint8_t _data = reg | p_write;
    gpio_out_write(CS, 0);
    spi_transfer(spi_bus, 0, 1, &_data);
    spi_transfer(spi_bus, 0, numBytes, data);
    gpio_out_write(CS, 1);
}

void FAST_CODE_1
RadioHalSpi::readRegisterOffset(uint8_t reg, uint8_t offset,
                                uint8_t *data, uint8_t numBytes) const
{
    uint8_t hdr[] = {(uint8_t)(reg | p_read), offset, 0};
    gpio_out_write(CS, 0);
    spi_transfer(spi_bus, 0, sizeof(hdr), hdr);
    spi_transfer(spi_bus, 1, numBytes, data);
    gpio_out_write(CS, 1);
}

void FAST_CODE_1
RadioHalSpi::writeRegisterOffset(uint8_t reg, uint8_t offset,
                                 uint8_t *data, uint8_t numBytes) const
{
    uint8_t hdr[] = {(uint8_t)(reg | p_write), offset};
    gpio_out_write(CS, 0);
    spi_transfer(spi_bus, 0, sizeof(hdr), hdr);
    spi_transfer(spi_bus, 0, numBytes, data);
    gpio_out_write(CS, 1);
}
