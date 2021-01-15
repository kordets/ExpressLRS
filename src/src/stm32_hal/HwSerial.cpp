#include "HwSerial.h"
#include "targets.h"
#include "Arduino.h"
#include "gpio.h"

#if (GPIO_PIN_RCSIGNAL_RX != UNDEF_PIN) || (GPIO_PIN_RCSIGNAL_TX != UNDEF_PIN)
HwSerial CrsfSerial(GPIO_PIN_RCSIGNAL_RX, GPIO_PIN_RCSIGNAL_TX, BUFFER_OE);
#endif

HwSerial::HwSerial(uint32_t _rx, uint32_t _tx, int32_t pin)
    : HardwareSerial(_rx, _tx, RCSIGNAL_USE_DMA)
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
