#include "HwSerial.h"
#include "targets.h"
#include <Arduino.h>


#if (GPIO_PIN_RCSIGNAL_RX != UNDEF_PIN) && (GPIO_PIN_RCSIGNAL_TX != UNDEF_PIN)
#define SPORT_RX_TX GPIO_PIN_RCSIGNAL_RX, GPIO_PIN_RCSIGNAL_TX
#else
#if (SPORT == PB10)
// USART3
#define SPORT_RX_TX PB11, SPORT
#elif (SPORT == PA2)
// USART2
#define SPORT_RX_TX PA3, SPORT
#elif (SPORT == PA9)
// USART1
#define SPORT_RX_TX PA10, SPORT
#else
#error "No valid S.Port UART TX pin defined!"
#endif
#endif

HwSerial CrsfSerial(SPORT_RX_TX, BUFFER_OE);

HwSerial::HwSerial(uint32_t _rx, uint32_t _tx, int32_t pin)
    : HardwareSerial(_rx, _tx)
{
    duplex_pin = gpio_out_setup(pin, 0);
}

HwSerial::HwSerial(void *peripheral, int32_t pin)
    : HardwareSerial(peripheral)
{
    duplex_pin = gpio_out_setup(pin, 0);
}

void HwSerial::Begin(uint32_t baud, uint32_t config)
{
    HardwareSerial::begin((unsigned long)baud, (uint8_t)config);
}

void ICACHE_RAM_ATTR HwSerial::enable_receiver(void)
{
    if (gpio_out_valid(duplex_pin)) {
        HardwareSerial::flush(); // wait until write ends
        gpio_out_write(duplex_pin, LOW);
        HAL_HalfDuplex_EnableReceiver(&_serial.handle);
    }
}

void ICACHE_RAM_ATTR HwSerial::enable_transmitter(void)
{
    if (gpio_out_valid(duplex_pin)) {
        HAL_HalfDuplex_EnableTransmitter(&_serial.handle);
        gpio_out_write(duplex_pin, HIGH);
    }
}
