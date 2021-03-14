#include "HwSerial.h"
#include "targets.h"
#include <Arduino.h>


HwSerial CrsfSerial(GPIO_PIN_RCSIGNAL_RX, GPIO_PIN_RCSIGNAL_TX,
                    BUFFER_OE, BUFFER_OE_INVERTED);

HwSerial::HwSerial(uint32_t _rx, uint32_t _tx, int32_t pin, uint8_t inv)
    : HardwareSerial(_rx, _tx)
{
    duplex_pin = gpio_out_setup(pin, LOW ^ inv);
    duplex_pin_inv = inv;
}

void HwSerial::Begin(uint32_t baud, uint32_t config)
{
    HardwareSerial::begin((unsigned long)baud, (uint8_t)config);
}

void FAST_CODE_1 HwSerial::enable_receiver(void)
{
    if (gpio_out_valid(duplex_pin)) {
        HardwareSerial::flush(); // wait until write ends
        gpio_out_write(duplex_pin, LOW ^ duplex_pin_inv);
        HAL_HalfDuplex_EnableReceiver(&_serial.handle);
    }
}

void FAST_CODE_1 HwSerial::enable_transmitter(void)
{
    if (gpio_out_valid(duplex_pin)) {
        HAL_HalfDuplex_EnableTransmitter(&_serial.handle);
        gpio_out_write(duplex_pin, HIGH ^ duplex_pin_inv);
    }
}
