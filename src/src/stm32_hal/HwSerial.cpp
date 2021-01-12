#include "HwSerial.h"
#include "targets.h"
#include "Arduino.h"
#include "gpio.h"


HwSerial CrsfSerial(GPIO_PIN_RCSIGNAL_RX, GPIO_PIN_RCSIGNAL_TX, BUFFER_OE);

HwSerial::HwSerial(uint32_t _rx, uint32_t _tx, int32_t pin)
    : HardwareSerial(_rx, _tx, 1)
{
    p_duplex_pin = gpio_out_setup(pin, 0);
}

void HwSerial::Begin(uint32_t baud, uint32_t config)
{
    HardwareSerial::begin((unsigned long)baud, (uint8_t)config);
}

void HwSerial::enable_receiver(void)
{
}

void HwSerial::enable_transmitter(void)
{
}
