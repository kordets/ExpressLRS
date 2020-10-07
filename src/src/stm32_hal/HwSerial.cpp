#include "HwSerial.h"
#include "targets.h"
#include "Arduino.h"
#include "gpio.h"

#if defined(GPIO_PIN_RCSIGNAL_RX) && defined(GPIO_PIN_RCSIGNAL_TX)
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

#define USE_DMA 0

HwSerial CrsfSerial(SPORT_RX_TX, BUFFER_OE);

HwSerial::HwSerial(uint32_t _rx, uint32_t _tx, int32_t pin)
    : HardwareSerial(_rx, _tx, USE_DMA)
{
    p_duplex_pin = gpio_out_setup(pin, 0);
}

HwSerial::HwSerial(void *peripheral, int32_t pin)
    : HardwareSerial(peripheral, USE_DMA)
{
    p_duplex_pin = gpio_out_setup(pin, 0);
}

void HwSerial::Begin(uint32_t baud, uint32_t config)
{
    HardwareSerial::begin((unsigned long)baud, (uint8_t)config);
}

void HwSerial::enable_receiver(void)
{
    //HardwareSerial::enable_receiver();
}

void HwSerial::enable_transmitter(void)
{
    //HardwareSerial::enable_transmitter();
}
