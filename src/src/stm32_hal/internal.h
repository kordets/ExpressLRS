// Code to setup clocks and gpio on stm32f1
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
// https://github.com/KevinOConnor/klipper
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#ifndef __INTERNAL_H_
#define __INTERNAL_H_

#include "hal_inc.h"

#if defined(STM32L0xx)
#define NVIC_EncodePriority(_x, _y, _z) (_y)
#define NVIC_GetPriorityGrouping() 0
#endif

#define _NOP() asm ("nop")

#define barrier() __asm__ __volatile__("": : :"memory")

static inline void write_u32(void *addr, uint32_t val) {
    barrier();
    *(volatile uint32_t *)addr = val;
}
static inline void write_u16(void *addr, uint16_t val) {
    barrier();
    *(volatile uint16_t *)addr = val;
}
static inline void write_u8(void *addr, uint8_t val) {
    barrier();
    *(volatile uint8_t *)addr = val;
}
static inline uint32_t read_u32(const void *addr) {
    uint32_t val = *(volatile const uint32_t *)addr;
    barrier();
    return val;
}
static inline uint16_t read_u16(const void *addr) {
    uint16_t val = *(volatile const uint16_t *)addr;
    barrier();
    return val;
}
static inline uint8_t read_u8(const void *addr) {
    uint8_t val = *(volatile const uint8_t *)addr;
    barrier();
    return val;
}

extern GPIO_TypeDef * digital_regs[];

#define GPIO_NUM_PINS   16
#define GPIO(PORT, NUM) (((PORT) - 'A') * GPIO_NUM_PINS + (NUM))
#define GPIO2PORT(PIN)  ((PIN) / GPIO_NUM_PINS)
#define GPIO2BIT(PIN)   (1U << GPIO2IDX(PIN))
#define GPIO2IDX(PIN)   ((PIN) % GPIO_NUM_PINS)

#define GPIO_INPUT        0
#define GPIO_OUTPUT       1
#define GPIO_AF           2
#define GPIO_ANALOG       3
#define GPIO_INPUT_PULLUP 4 /* For arduino ide compatibility */
#define GPIO_OPEN_DRAIN   0x100
#define GPIO_FUNCTION(fn) (GPIO_AF | ((fn) << 4))

#define CONFIG_CLOCK_FREQ     F_CPU

//extern uint32_t SystemCoreClock;
#define clockCyclesPerMicrosecond()  ( CONFIG_CLOCK_FREQ / 1000000U )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

void shutdown(const char * reason);

uint32_t timer_read_time(void);
uint8_t timer_is_before(uint32_t time1, uint32_t time2);

void enable_pclock(uint32_t periph_base);
uint32_t is_enabled_pclock(uint32_t periph_base);
uint32_t get_pclock_frequency(uint32_t periph_base);
void gpio_clock_enable(GPIO_TypeDef *regs);
void gpio_peripheral(uint32_t gpio, uint32_t mode, int pullup);

enum {
    DMA_USART_RX,
    DMA_USART_TX,
};

uint32_t dma_get(uint32_t periph, uint8_t type, uint8_t index);
uint32_t dma_channel_get(uint32_t periph, uint8_t type, uint8_t index);
uint32_t dma_irq_get(uint32_t periph, uint8_t type, uint8_t index);

uint32_t uart_peripheral_get(uint32_t rx_pin, uint32_t tx_pin);
void uart_pins_get(uint32_t periph, uint32_t *rx_pin, uint32_t *tx_pin, uint8_t alt);
void uart_config_afio(uint32_t periph, uint32_t rx_pin, uint32_t tx_pin);
#endif
