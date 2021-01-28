#ifndef RADIO_HAL_SPI_H_
#define RADIO_HAL_SPI_H_

#include <stdint.h>
#include "gpio.h"

class RadioHalSpi {
public:

protected:
    RadioHalSpi(uint8_t read = 0, uint8_t write = 0)
            : p_write(write), p_read(read) {}
    void Begin(uint32_t speed, int sck, int miso, int mosi);

    void transfer(uint8_t *data, uint8_t len, uint8_t receive=1) const;

    uint8_t readRegister(uint8_t reg) const;
    void writeRegister(uint8_t reg, uint8_t data) const;

    void readRegisterBurst(uint8_t reg, uint8_t numBytes, uint8_t *inBytes) const;
    void writeRegisterBurst(uint8_t reg, uint8_t *data, uint8_t numBytes) const;

    void readRegisterOffset(uint8_t reg, uint8_t offset,
                            uint8_t *data, uint8_t numBytes) const;
    void writeRegisterOffset(uint8_t reg, uint8_t offset,
                             uint8_t *data, uint8_t numBytes) const;

    struct gpio_out CS;

private:
    struct spi_config spi_bus;
    const uint8_t p_write;
    const uint8_t p_read;
};

#endif /* RADIO_HAL_SPI_H_ */
