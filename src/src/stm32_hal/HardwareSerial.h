#pragma once

#include <stdint.h>
#include <stddef.h>
#include "gpio.h"
#include "irq.h"

#define SERIAL_8N1 0x06

#define UART_BUFF_SIZE 256

/* This is to avoid unnecessary data copy */
#define UART_USE_TX_POOL_ONLY 0

// Used by the TX DMA
// Note: make sure the TX is not called ofter for same buffer!
struct tx_pool_s {
    struct tx_pool_s * next;
    uint8_t * data_ptr;
    uint32_t len;
};


/* Interface kept arduino compatible */
class HardwareSerial
{
public:
    HardwareSerial(uint32_t rx, uint32_t tx, uint8_t dma=0);
    void setTx(uint32_t tx_pin);
    void setRx(uint32_t rx_pin);
    void begin(unsigned long baud, uint8_t mode = SERIAL_8N1);
    void end(void);
    int available(void);
    int read(void);
    void flush(void);
    uint32_t write(const uint8_t *buff, uint32_t len);
    void setTimeout(unsigned long) {}

    void hw_enable_receiver(void);
    void hw_enable_transmitter(void);

    uint8_t rx_head;
    uint8_t rx_tail;
    uint8_t rx_buffer[UART_BUFF_SIZE];

#if UART_USE_TX_POOL_ONLY
    uint8_t * tx_buffer_ptr;
    uint32_t tx_buffer_len;
#else
    uint8_t tx_head;
    uint8_t tx_tail;
    uint8_t tx_buffer[UART_BUFF_SIZE];
#endif

    // for DMA TX
    uint8_t* tx_pool_get(uint32_t * len)
    {
        uint8_t * ret = NULL;
        irqstatus_t flag = irq_save();
        if (tx_pool_head != tx_pool_tail) {
            ret = tx_pool_tail->data_ptr;
            *len = tx_pool_tail->len;
            tx_pool_tail->data_ptr = NULL;
            tx_pool_tail = tx_pool_tail->next;
        }
        irq_restore(flag);
        return ret;
    }

    void* dma_unit_tx;
    void* dma_unit_rx;
    void* p_usart_tx;
    void* p_usart_rx;
    uint8_t dma_ch_tx;
    uint8_t dma_ch_rx;
    uint8_t usart_tx_idx;
    uint8_t usart_rx_idx;

protected:
    uint32_t rx_pin;
    uint32_t tx_pin;
    struct gpio_out p_duplex_pin;

private:
    uint8_t usart_irq_rx;
    uint8_t usart_irq_tx;
    uint8_t p_use_dma;
    uint8_t dma_irq_tx;
    uint8_t dma_irq_rx;
    uint8_t half_duplex;

    // for DMA TX
    struct tx_pool_s tx_pool[16];
    struct tx_pool_s * tx_pool_head;
    struct tx_pool_s * tx_pool_tail;

    uint32_t DR_RX;
    uint32_t DR_TX;

    void tx_pool_add(const uint8_t * data, uint32_t len)
    {
        irqstatus_t flag = irq_save();
        struct tx_pool_s * next = tx_pool_head;
        if (!next->data_ptr) {
            next->data_ptr = (uint8_t*)data;
            next->len = len;
            tx_pool_head = next->next;
        }
        irq_restore(flag);
    }
};

#if defined(TARGET_R9M_TX)
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;
#endif
