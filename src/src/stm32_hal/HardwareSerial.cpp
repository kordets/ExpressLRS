//
// STM32 serial
//

#include "HardwareSerial.h"
#include "internal.h"
#include "irq.h"
#include "debug_elrs.h"
#include "targets.h"

#include <stm32f1xx_ll_bus.h>
#include <stm32f1xx_ll_dma.h>
#include <string.h>

#define UART_USE_DMA_RX 1
#define UART_USE_DMA_TX 1


#define DIV_ROUND_CLOSEST(x, divisor) ({       \
    typeof(divisor) __divisor = divisor;       \
    (((x) + ((__divisor) / 2)) / (__divisor)); \
})


#ifdef DEBUG_SERIAL
// dummy putchar
#define PRINTF_BUFF_SIZE 128
char   printf_out[4][PRINTF_BUFF_SIZE];
size_t printf_idx, printf_buff;

void Printf::_putchar(char character)
{
    printf_out[printf_buff][printf_idx++] = character;
    /* Send buff out if line end or buffer is full */
    if (character == '\n' || PRINTF_BUFF_SIZE <= printf_idx) {
        DEBUG_SERIAL.write((uint8_t*)printf_out[printf_buff], printf_idx);
        printf_buff = (printf_buff+1) % 4;
        printf_idx = 0;
    }
}
#endif //DEBUG_SERIAL

constexpr uint8_t serial_cnt = 0
#ifdef USART1
    + 1
#endif
#ifdef USART2
    + 1
#endif
#ifdef USART3
    + 1
#endif
#ifdef USART4
    + 1
#endif
#ifdef USART5
    + 1
#endif
    ;

struct usart_config
{
    USART_TypeDef *usart;
    uint8_t rx_pin, tx_pin;
};



HardwareSerial * _started_serials[serial_cnt];

int8_t DMA_transmit(HardwareSerial * ptr, uint8_t dma_ch)
{
    //irqstatus_t irqs = irq_save();

    /* Check if TX ongoing... */
    /*if (!LL_DMA_IsEnabledChannel(DMA1, dma_ch))*/ {
        /* Check if something to send */
        uint32_t len = 0, data = (uint32_t)ptr->tx_pool_get(&len);
        if (data && len) {
            //LL_DMA_DisableChannel(DMA1, dma_ch);
            /* Clear all irq flags */
            WRITE_REG(DMA1->IFCR, 0xF << (dma_ch - 1));
            /* Set source address */
            LL_DMA_SetMemoryAddress(DMA1, dma_ch, data);
            LL_DMA_SetDataLength(DMA1, dma_ch, len);
            /* Start transfer */
            LL_DMA_EnableChannel(DMA1, dma_ch);
            return 0;
        }
    }

    //irq_restore(irqs);
    return -1;
}

#ifdef __cplusplus
extern "C"
{
#endif

void USART_IDLE_IRQ_handler(HardwareSerial *serial, USART_TypeDef * uart, uint8_t dma_ch)
{
    uint32_t SR = uart->SR, CR1 = uart->CR1;

    /* Check for IDLE line interrupt */
    if ((SR & USART_SR_IDLE) && (CR1 & USART_CR1_RE)) {
        uint8_t head_pos = sizeof(serial->rx_buffer) - LL_DMA_GetDataLength(DMA1, dma_ch);
        uint8_t head = serial->rx_head;
        uint8_t tail = head + (uint8_t)(head_pos - head);
        if (tail >= serial->rx_tail)
            serial->rx_tail = tail;
        serial->rx_head = head_pos;
    }
    /* Check for RX data */
    else if ((SR & (USART_SR_RXNE | USART_SR_ORE)) && (CR1 & USART_CR1_RXNEIE)) {
        uint8_t next = serial->rx_head;
        if (next != serial->rx_tail) {
            serial->rx_buffer[next] = uart->DR;
            serial->rx_head = next + 1;
        }
    }

    // Check if TX is enabled and TX Empty IRQ triggered
    if ((SR & USART_SR_TXE) && (CR1 & USART_CR1_TXEIE)) {
        //  Check if data available
        if (serial->tx_head <= serial->tx_tail)
            serial->enable_receiver();
        else
            uart->DR = serial->tx_buffer[serial->tx_tail++];
    }

    //uart->SR = SR; // Needed?
}

#ifdef USART1
    void USART1_IRQHandler(void)
    {
        USART_IDLE_IRQ_handler(_started_serials[0], USART1, LL_DMA_CHANNEL_5);
    }
#endif /* USART1 */
#ifdef USART2
    void USART2_IRQHandler(void)
    {
        USART_IDLE_IRQ_handler(_started_serials[1], USART2, LL_DMA_CHANNEL_7);
    }
#endif /* USART2 */
#ifdef USART3
    void USART3_IRQHandler(void)
    {
        USART_IDLE_IRQ_handler(_started_serials[2], USART3, LL_DMA_CHANNEL_3);
    }
#endif /* USART3 */

/* USART1 TX DMA */
void DMA1_Channel4_IRQHandler(void)
{
    uint32_t sr = DMA1->ISR;
    if (sr & DMA_ISR_TCIF4) {
        DMA1->ISR = DMA_ISR_TCIF4; // Clear IRQ
        CLEAR_BIT(DMA1_Channel4->CCR, DMA_CCR_EN); // Disable DMA stream
#ifdef USART1
        if (_started_serials[0] && DMA_transmit(_started_serials[0], LL_DMA_CHANNEL_4) < 0)
            _started_serials[0]->enable_receiver();
#endif
    }
}

/* USART2 TX DMA */
void DMA1_Channel6_IRQHandler(void)
{
    uint32_t sr = DMA1->ISR;
    if (sr & DMA_ISR_TCIF6) {
        DMA1->ISR = DMA_ISR_TCIF6; // Clear IRQ
        CLEAR_BIT(DMA1_Channel6->CCR, DMA_CCR_EN); // Disable DMA stream
#ifdef USART2
        if (_started_serials[1] && DMA_transmit(_started_serials[1], LL_DMA_CHANNEL_6) < 0)
            _started_serials[1]->enable_receiver();
#endif
    }
}

/* USART3 TX DMA */
void DMA1_Channel2_IRQHandler(void)
{
    uint32_t sr = DMA1->ISR;
    if (sr & DMA_ISR_TCIF2) {
        DMA1->ISR = DMA_ISR_TCIF2; // Clear IRQ
        CLEAR_BIT(DMA1_Channel2->CCR, DMA_CCR_EN); // Disable DMA stream
#ifdef USART3
        if (_started_serials[2] && DMA_transmit(_started_serials[2], LL_DMA_CHANNEL_2) < 0)
            _started_serials[2]->enable_receiver();
#endif
    }
}

#ifdef __cplusplus
}
#endif


HardwareSerial::HardwareSerial(uint32_t rx, uint32_t tx, uint8_t dma)
{
    USART_TypeDef * uart = NULL;
    rx_pin = rx;
    tx_pin = tx;

    if (rx == GPIO('A', 10) && tx == GPIO('A', 9))
        uart = USART1;
    else if (rx == GPIO('A', 3) && tx == GPIO('A', 2))
        uart = USART2;
    else if ((rx == GPIO('D', 9) && tx == GPIO('D', 8)) ||
             (rx == GPIO('B', 11) && tx == GPIO('B', 10)))
        uart = USART3;
    p_usart = uart;
    p_use_dma = dma && (UART_USE_DMA_RX | UART_USE_DMA_TX);
}

HardwareSerial::HardwareSerial(void *peripheral, uint8_t dma)
{
    USART_TypeDef * uart = (USART_TypeDef *)peripheral;
    if (uart == USART1) {
        rx_pin = GPIO('A', 10);
        tx_pin = GPIO('A', 9);
    } else if (uart == USART2) {
        rx_pin = GPIO('A', 3);
        tx_pin = GPIO('A', 2);
    } else if (uart == USART3) {
        rx_pin = GPIO('B', 11);
        tx_pin = GPIO('B', 10);
    }
    p_usart = uart;
    p_use_dma = dma && (UART_USE_DMA_RX | UART_USE_DMA_TX);
}


void HardwareSerial::begin(unsigned long baud, uint8_t mode)
{
    USART_TypeDef * uart = (USART_TypeDef *)p_usart;

    if (uart == USART1) {
        usart_irq = USART1_IRQn;
        dma_ch_tx = LL_DMA_CHANNEL_4;
        dma_ch_rx = LL_DMA_CHANNEL_5;
        dma_irq_tx = DMA1_Channel4_IRQn;
        dma_irq_rx = DMA1_Channel5_IRQn;
        write_u32(&_started_serials[0], (uint32_t)this);
#ifdef USART2
    } else if (uart == USART2) {
        usart_irq = USART2_IRQn;
        dma_ch_tx = LL_DMA_CHANNEL_6;
        dma_ch_rx = LL_DMA_CHANNEL_7;
        dma_irq_tx = DMA1_Channel6_IRQn;
        dma_irq_rx = DMA1_Channel7_IRQn;
        write_u32(&_started_serials[1], (uint32_t)this);
#endif // USART2
#ifdef USART3
    } else if (uart == USART3) {
        usart_irq = USART3_IRQn;
        dma_ch_tx = LL_DMA_CHANNEL_2;
        dma_ch_rx = LL_DMA_CHANNEL_3;
        dma_irq_tx = DMA1_Channel2_IRQn;
        dma_irq_rx = DMA1_Channel3_IRQn;
        write_u32(&_started_serials[2], (uint32_t)this);
#endif // USART3
    } else {
        // Invalid HW UART config!
        return;
    }
    /* Init RX buffer */
    rx_head = rx_tail = 0;
    tx_head = tx_tail = 0;

    if (p_use_dma) {
        /*********** USART DMA Init ***********/
        enable_pclock((uint32_t)DMA1);
        //LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    #if UART_USE_DMA_RX
        /* RX DMA stream config */
        LL_DMA_DisableChannel(DMA1, dma_ch_rx);
        LL_DMA_SetDataTransferDirection(DMA1, dma_ch_rx, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
        LL_DMA_SetChannelPriorityLevel(DMA1, dma_ch_rx, LL_DMA_PRIORITY_LOW);
        LL_DMA_SetMode(DMA1, dma_ch_rx, LL_DMA_MODE_CIRCULAR);
        LL_DMA_SetPeriphIncMode(DMA1, dma_ch_rx, LL_DMA_PERIPH_NOINCREMENT);
        LL_DMA_SetMemoryIncMode(DMA1, dma_ch_rx, LL_DMA_MEMORY_INCREMENT);
        LL_DMA_SetPeriphSize(DMA1, dma_ch_rx, LL_DMA_PDATAALIGN_BYTE);
        LL_DMA_SetMemorySize(DMA1, dma_ch_rx, LL_DMA_MDATAALIGN_BYTE);
        /* Set source and target */
        LL_DMA_SetPeriphAddress(DMA1, dma_ch_rx, (uint32_t)&uart->DR);
        LL_DMA_SetMemoryAddress(DMA1, dma_ch_rx, (uint32_t)rx_buffer);
        LL_DMA_SetDataLength(DMA1, dma_ch_rx, sizeof(rx_buffer));
        /* Set interrupts */
        //LL_DMA_EnableIT_HT(DMA1, dma_ch_rx);
        //LL_DMA_EnableIT_TC(DMA1, dma_ch_rx);
        LL_DMA_DisableIT_HT(DMA1, dma_ch_rx);
        LL_DMA_DisableIT_TC(DMA1, dma_ch_rx);
        /* Set interrupt configuration */
        NVIC_SetPriority((IRQn_Type)dma_irq_rx, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
        NVIC_EnableIRQ((IRQn_Type)dma_irq_rx);
        /* Enable DMA */
        LL_DMA_EnableChannel(DMA1, dma_ch_rx);
    #endif // UART_USE_DMA_RX

    #if UART_USE_DMA_TX
        /* Init TX list */
        tx_pool_tail = tx_pool_head = tx_pool;
        uint8_t iter;
        for (iter = 0; iter < (sizeof(tx_pool)/sizeof(tx_pool[0]) - 1); iter++) {
            tx_pool[iter].next = &tx_pool[iter+1];
        }
        tx_pool[iter].next = &tx_pool[0];

        /* TX DMA stream config */
        LL_DMA_DisableChannel(DMA1, dma_ch_tx);
        LL_DMA_SetDataTransferDirection(DMA1, dma_ch_tx, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
        LL_DMA_SetChannelPriorityLevel(DMA1, dma_ch_tx, LL_DMA_PRIORITY_LOW);
        LL_DMA_SetMode(DMA1, dma_ch_tx, LL_DMA_MODE_NORMAL);
        LL_DMA_SetPeriphIncMode(DMA1, dma_ch_tx, LL_DMA_PERIPH_NOINCREMENT);
        LL_DMA_SetMemoryIncMode(DMA1, dma_ch_tx, LL_DMA_MEMORY_INCREMENT);
        LL_DMA_SetPeriphSize(DMA1, dma_ch_tx, LL_DMA_PDATAALIGN_BYTE);
        LL_DMA_SetMemorySize(DMA1, dma_ch_tx, LL_DMA_MDATAALIGN_BYTE);
        /* Set target address */
        LL_DMA_SetPeriphAddress(DMA1, dma_ch_tx, (uint32_t)&uart->DR);
        /* Set interrupts */
        LL_DMA_DisableIT_HT(DMA1, dma_ch_tx);
        LL_DMA_EnableIT_TC(DMA1, dma_ch_tx);
        /* Set interrupt configuration */
        NVIC_SetPriority((IRQn_Type)dma_irq_tx, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
        NVIC_EnableIRQ((IRQn_Type)dma_irq_tx);
    #endif // UART_USE_DMA_TX
    }

    /*********** USART Init ***********/
    enable_pclock((uint32_t)uart);
    uint32_t pclk = get_pclock_frequency((uint32_t)uart);
    uint32_t div = DIV_ROUND_CLOSEST(pclk, baud);
    uart->BRR = (((div / 16) << USART_BRR_DIV_Mantissa_Pos) | ((div % 16) << USART_BRR_DIV_Fraction_Pos));

    // Half duplex
    DR_RX = USART_CR1_UE | USART_CR1_RE;
    DR_TX = USART_CR1_UE | USART_CR1_TE;
    DR_RX |= ((p_use_dma) ? USART_CR1_IDLEIE : USART_CR1_RXNEIE);
    DR_TX |= ((p_use_dma) ? 0 : USART_CR1_TXEIE);
    if ((BUFFER_OE == UNDEF_PIN) || (!p_duplex_pin.regs)) {
        // Full duplex
        DR_RX |= USART_CR1_TE;
        DR_TX |= DR_RX;
    }
    enable_receiver();

    NVIC_SetPriority((IRQn_Type)usart_irq, 1);
    NVIC_EnableIRQ((IRQn_Type)usart_irq);

    gpio_peripheral(rx_pin, GPIO_FUNCTION(7), 1);
    gpio_peripheral(tx_pin, GPIO_FUNCTION(7), 0);
}

void HardwareSerial::end(void)
{
    USART_TypeDef * uart = (USART_TypeDef *)p_usart;

    LL_DMA_DisableChannel(DMA1, dma_ch_rx);
    NVIC_DisableIRQ((IRQn_Type)dma_irq_rx);

    LL_DMA_DisableChannel(DMA1, dma_ch_tx);
    NVIC_DisableIRQ((IRQn_Type)dma_irq_tx);

    uart->CR1 = 0;
    NVIC_DisableIRQ((IRQn_Type)usart_irq);
}

int HardwareSerial::available(void)
{
    //irqstatus_t flag = irq_save();
    uint8_t head = read_u8(&rx_head), tail = read_u8(&rx_tail);
    //irq_restore(flag);
    return (uint8_t)(head - tail);
}

int HardwareSerial::read(void)
{
    if (!available())
        return -1;
    //irqstatus_t flag = irq_save();
    uint8_t tail = read_u8(&rx_tail);
    write_u8(&rx_tail, tail+1);
    //irq_restore(flag);
    return rx_buffer[tail++];
}

void HardwareSerial::flush(void)
{
    // Wait until data is sent
    //while (output.size())
        ;
}

#if 0
size_t HardwareSerial::write(uint8_t data)
{
    USART_TypeDef * uart = (USART_TypeDef *)p_usart;
    /* Write is used only for debug prints so ignore DMA */
    if (!p_use_dma) {
        irqstatus_t flag = irq_save();
        // push data into tx_buffer...
        irq_restore(flag);
    }
    enable_transmitter();
    return 1;
}
#endif

uint32_t HardwareSerial::write(const uint8_t *buff, uint32_t len)
{
    if (p_use_dma) {
        tx_pool_add(buff, len);
        if (!LL_DMA_IsEnabledChannel(DMA1, dma_ch_tx))
            DMA_transmit(this, dma_ch_tx);
    } else {
        //USART_TypeDef * uart = (USART_TypeDef *)p_usart;
        //irqstatus_t flag = irq_save();
        // push data into tx_buffer...
        uint8_t tmax = read_u8(&tx_head), tpos = read_u8(&tx_tail);
        if (tpos >= tmax) {
            tpos = tmax = 0;
            write_u8(&tx_head, 0);
            write_u8(&tx_tail, 0);
        }
        if ((tmax + len) > sizeof(tx_buffer)) {
            if ((tmax + len - tpos) > sizeof(tx_buffer))
                // Not enough space for message
                return 0;
            // Disable TX irq and move buffer
            write_u8(&tx_head, 0); // this stops TX irqs

            tpos = read_u8(&tx_tail);
            tmax -= tpos;
            memmove(&tx_buffer[0], &tx_buffer[tpos], tmax);
            write_u8(&tx_tail, 0);
            write_u8(&tx_head, tmax);
            //enable_transmitter();
        }

        memcpy(&tx_buffer[tmax], buff, len);
        write_u8(&tx_head, (tmax + len));

        //irq_restore(flag);
    }
    enable_transmitter();
    return len;
}

void HardwareSerial::enable_receiver(void)
{
    USART_TypeDef * uart = (USART_TypeDef *)p_usart;
#if BUFFER_OE != UNDEF_PIN
    if (p_duplex_pin.regs) {
        // flush();
        gpio_out_write(p_duplex_pin, 0);
    }
#endif // BUFFER_OE
    uart->CR1 = DR_RX;
}

void HardwareSerial::enable_transmitter(void)
{
    USART_TypeDef * uart = (USART_TypeDef *)p_usart;
#if BUFFER_OE != UNDEF_PIN
    if (p_duplex_pin.regs) {
        //delayMicroseconds(20); // TODO: Check if this is really needed!
        gpio_out_write(p_duplex_pin, 1);
    }
#endif // BUFFER_OE
    uart->CR1 = DR_TX;
}

#if defined(TARGET_R9M_TX)
#define PRINTF_USE_DMA 0
HardwareSerial Serial1(USART1, PRINTF_USE_DMA);
#endif
