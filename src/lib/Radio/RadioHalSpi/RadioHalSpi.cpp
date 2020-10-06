#include "RadioHalSpi.h"
#include "platform.h"

void RadioHalSpi::Begin(uint32_t speed, int sck, int miso, int mosi, int ss)
{
    spi_bus.prepare(speed, sck, miso, mosi, ss);
}

uint8_t ICACHE_RAM_ATTR
RadioHalSpi::readRegister(uint8_t reg) const
{
    uint8_t InByte;
    spi_bus.set_ss(LOW);
    spi_bus.write(reg | p_read);
    InByte = spi_bus.transfer(0x00);
    spi_bus.set_ss(HIGH);

    return (InByte);
}

void ICACHE_RAM_ATTR
RadioHalSpi::writeRegister(uint8_t reg, uint8_t data) const
{
    spi_bus.set_ss(LOW);
    spi_bus.write(reg | p_write);
    spi_bus.write(data);
    spi_bus.set_ss(HIGH);
}

void ICACHE_RAM_ATTR
RadioHalSpi::readRegisterBurst(uint8_t reg, uint8_t numBytes, uint8_t *inBytes) const
{
    spi_bus.set_ss(LOW);
    spi_bus.write(reg | p_read);
    spi_bus.transfer(inBytes, numBytes);
    spi_bus.set_ss(HIGH);
}

void ICACHE_RAM_ATTR
RadioHalSpi::writeRegisterBurst(uint8_t reg, uint8_t *data, uint8_t numBytes) const
{
    spi_bus.set_ss(LOW);
    spi_bus.write(reg | p_write);
    spi_bus.write(data, numBytes);
    spi_bus.set_ss(HIGH);
}

void ICACHE_RAM_ATTR
RadioHalSpi::readRegisterAddr(uint8_t reg, uint16_t addr,
                              uint8_t *data, uint8_t numBytes) const
{
    uint8_t hdr[] = {(uint8_t)(reg | p_write), (uint8_t)(addr >> 8), (uint8_t)addr, 0};
    spi_bus.set_ss(LOW);
    //spi_bus.write(reg | p_write);
    //spi_bus.write((uint8_t)(addr >> 8));
    //spi_bus.write((uint8_t)addr);
    //spi_bus.write(0); // 1 NOP before data
    spi_bus.write(hdr, sizeof(hdr));
    spi_bus.transfer(data, numBytes);
    spi_bus.set_ss(HIGH);
}

void ICACHE_RAM_ATTR
RadioHalSpi::writeRegisterAddr(uint8_t reg, uint16_t addr,
                               uint8_t *data, uint8_t numBytes) const
{
    uint8_t hdr[] = {(uint8_t)(reg | p_write), (uint8_t)(addr >> 8), (uint8_t)addr};
    spi_bus.set_ss(LOW);
    //spi_bus.write(reg | p_write);
    //spi_bus.write((uint8_t)(addr >> 8));
    //spi_bus.write((uint8_t)addr);
    spi_bus.write(hdr, sizeof(hdr));
    spi_bus.write(data, numBytes);
    spi_bus.set_ss(HIGH);
}

void ICACHE_RAM_ATTR
RadioHalSpi::readRegisterOffset(uint8_t reg, uint8_t offset,
                                uint8_t *data, uint8_t numBytes) const
{
    uint8_t hdr[] = {(uint8_t)(reg | p_write), offset, 0};
    spi_bus.set_ss(LOW);
    //spi_bus.write(reg | p_write);
    //spi_bus.write(offset);
    //spi_bus.write(0); // 1 NOP before data
    spi_bus.write(hdr, sizeof(hdr));
    spi_bus.transfer(data, numBytes);
    spi_bus.set_ss(HIGH);
}

void ICACHE_RAM_ATTR
RadioHalSpi::writeRegisterOffset(uint8_t reg, uint8_t offset,
                                 uint8_t *data, uint8_t numBytes) const
{
    uint8_t hdr[] = {(uint8_t)(reg | p_write), offset};
    spi_bus.set_ss(LOW);
    //spi_bus.write(reg | p_write);
    //spi_bus.write(offset);
    spi_bus.write(hdr, sizeof(hdr));
    spi_bus.write(data, numBytes);
    spi_bus.set_ss(HIGH);
}
