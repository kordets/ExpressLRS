#pragma once

#include <stdint.h>
#include <stddef.h>
#include "gpio.h"
#include "irq.h"

#define SERIAL_8N1 0x06

#define RX_BUFF_SIZE 256

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
    HardwareSerial(void *peripheral, uint8_t dma=0);
    void begin(unsigned long baud, uint8_t mode = SERIAL_8N1);
    void end(void);
    int available(void);
    int read(void);
    void flush(void);
    //size_t write(uint8_t);
    uint32_t write(const uint8_t *buff, uint32_t len);
    void setTimeout(unsigned long) {}

    void enable_receiver(void);
    void enable_transmitter(void);

    uint8_t rx_head;
    uint8_t rx_tail;
    uint8_t rx_buffer[RX_BUFF_SIZE];

    uint8_t tx_head;
    uint8_t tx_tail;
    uint8_t tx_buffer[RX_BUFF_SIZE];

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

protected:
    void* p_usart;
    uint32_t rx_pin;
    uint32_t tx_pin;
    int32_t usart_irq;
    struct gpio_out p_duplex_pin;

private:
    uint8_t p_use_dma;
    uint8_t dma_ch_tx;
    uint8_t dma_ch_rx;
    uint8_t dma_irq_tx;
    uint8_t dma_irq_rx;

    // for DMA TX
    struct tx_pool_s tx_pool[16];
    struct tx_pool_s * tx_pool_head;
    struct tx_pool_s * tx_pool_tail;

    uint32_t DR_RX;
    uint32_t DR_TX;

    void tx_pool_add(const uint8_t * data, uint32_t len)
    {
        irqstatus_t flag = irq_save();
        struct tx_pool_s * next = tx_pool_head->next;
        if (!next->data_ptr) {
            next->data_ptr = (uint8_t*)data;
            tx_pool_head = next;
        }
        irq_restore(flag);
    }
};

#if defined(TARGET_R9M_TX)
extern HardwareSerial Serial1;
#endif
