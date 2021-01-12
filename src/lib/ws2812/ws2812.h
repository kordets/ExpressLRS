#ifndef __WS2812_H_
#define __WS2812_H_

#include <stdint.h>

#if PLATFORM_STM32
void ws2812_init(uint32_t pin);
void ws2812_set_color(uint8_t const r, uint8_t const g, uint8_t const b);
void ws2812_set_color_u32(uint32_t const rgb);
#else // !PLATFORM_STM32
#define ws2812_init()
#define ws2812_set_color(...)
#define ws2812_set_color_u32(...)
#endif // PLATFORM_STM32
#endif /* __WS2812_H_ */
