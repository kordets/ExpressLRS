#include "ws2812.h"
#include "gpio.h"
#include "platform_internal.h"

#if PLATFORM_STM32
static struct gpio_out led_pin;
static uint8_t current_rgb[3];

#include "hal_inc.h"
#include <string.h>

#ifndef BRIGHTNESS
#define BRIGHTNESS 10 // 1...256
#endif

#ifndef __NOP
#define __NOP() asm ("nop")
#endif

#define WS2812_DELAY_LONG() \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();

#define WS2812_DELAY_SHORT() \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); \
    __NOP(); __NOP(); __NOP(); __NOP(); __NOP();

static inline void
ws2812_send_1(GPIO_TypeDef *regs, uint32_t bit)
{
    regs->BSRR = bit;
    WS2812_DELAY_LONG();
    regs->BSRR = bit << 16;
    WS2812_DELAY_SHORT();
}

static inline void
ws2812_send_0(GPIO_TypeDef *regs, uint32_t bit)
{
    regs->BSRR = bit;
    WS2812_DELAY_SHORT();
    regs->BSRR = bit << 16;
    WS2812_DELAY_LONG();
}

static uint32_t bitReverse(uint32_t input)
{
    // r will be reversed bits of v; first get LSB of v
    uint8_t r = (uint8_t)((input * BRIGHTNESS) >> 8);
    uint8_t s = 8 - 1; // extra shift needed at end

    for (input >>= 1; input; input >>= 1) {
        r <<= 1;
        r |= input & 1;
        s--;
    }
    r <<= s; // shift when input's highest bits are zero
    return r;
}

static void ws2812_send_color(uint8_t const *const RGB) // takes RGB data
{
    if ((current_rgb[0] == RGB[0] &&
         current_rgb[1] == RGB[1] &&
         current_rgb[2] == RGB[2]) ||
        !gpio_out_valid(led_pin))
        return;

    GPIO_TypeDef *regs = (GPIO_TypeDef *)led_pin.regs;
    uint32_t bit = led_pin.bit;
    uint32_t LedColourData =
        bitReverse(RGB[1]) +        // Green
        (bitReverse(RGB[0]) << 8) + // Red
        (bitReverse(RGB[2]) << 16); // Blue
    uint8_t bits = 24;
    while (bits--) {
        (LedColourData & 0x1) ? ws2812_send_1(regs, bit) : ws2812_send_0(regs, bit);
        LedColourData >>= 1;
    }
    memcpy(current_rgb, RGB, sizeof(current_rgb));
}

void ws2812_init(uint32_t pin)
{
    led_pin = gpio_out_setup(pin, 0);
}

void ws2812_set_color(uint8_t const r, uint8_t const g, uint8_t const b)
{
    uint8_t data[3] = {r, g, b};
    ws2812_send_color(data);
}

void ws2812_set_color_u32(uint32_t const rgb)
{
    uint8_t data[3] = {(uint8_t)(rgb), (uint8_t)(rgb >> 8), (uint8_t)(rgb >> 16)};
    ws2812_send_color(data);
}

#endif /* PLATFORM_STM32 */
