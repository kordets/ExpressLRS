#ifndef RADIO_HAL_SPI_H_
#define RADIO_HAL_SPI_H_

#include "HwSpi.h"
#include <stdint.h>

class RadioHalSpi {
public:

protected:
    RadioHalSpi(HwSpi &spi, uint8_t read = 0, uint8_t write = 0)
            : spi_bus(spi), p_write(write), p_read(read) {}
    void Begin(uint32_t speed, int sck, int miso, int mosi, int ss);

    uint8_t readRegister(uint8_t reg) const;
    void writeRegister(uint8_t reg, uint8_t data) const;

    void readRegisterBurst(uint8_t reg, uint8_t numBytes, uint8_t *inBytes) const;
    void writeRegisterBurst(uint8_t reg, uint8_t *data, uint8_t numBytes) const;

    void readRegisterAddr(uint8_t reg, uint16_t addr,
                          uint8_t *data, uint8_t numBytes) const;
    void writeRegisterAddr(uint8_t reg, uint16_t addr,
                           uint8_t *data, uint8_t numBytes) const;

    void readRegisterOffset(uint8_t reg, uint8_t offset,
                            uint8_t *data, uint8_t numBytes) const;
    void writeRegisterOffset(uint8_t reg, uint8_t offset,
                             uint8_t *data, uint8_t numBytes) const;

private:
    HwSpi &spi_bus;
    const uint8_t p_write;
    const uint8_t p_read;
};

#endif /* RADIO_HAL_SPI_H_ */
