//
// STM32 serial
//

#include "HardwareSerial.h"
#include "internal.h"
#include "irq.h"
#include "debug_elrs.h"
#include "targets.h"
#include "priorities.h"

#if defined(STM32F1xx)
#include <stm32f1xx_ll_bus.h>
#include <stm32f1xx_ll_dma.h>
#define StatReg         SR
#define RxDataReg       DR
#define TxDataReg       DR
#elif defined(STM32L0xx)
#include <stm32l0xx_ll_bus.h>
#include <stm32l0xx_ll_dma.h>
#define StatReg         ISR
#define RxDataReg       RDR
#define TxDataReg       TDR
#define USART_SR_IDLE   USART_ISR_IDLE
#define USART_SR_RXNE   USART_ISR_RXNE
#define USART_SR_ORE    USART_ISR_ORE
#define USART_SR_TXE    USART_ISR_TXE
#elif defined(STM32L4xx)
#include <stm32l4xx_ll_bus.h>
#include <stm32l4xx_ll_dma.h>
#define StatReg         ISR
#define RxDataReg       RDR
#define TxDataReg       TDR
#define USART_SR_IDLE   USART_ISR_IDLE
#define USART_SR_RXNE   USART_ISR_RXNE
#define USART_SR_ORE    USART_ISR_ORE
#define USART_SR_TXE    USART_ISR_TXE
#endif
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
    uint8_t is_end = character == '\n';
    printf_out[printf_buff][printf_idx++] = character;

    /* Send buff out if line end or buffer is full */
    if (is_end || PRINTF_BUFF_SIZE <= printf_idx) {
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
    ;
HardwareSerial * _started_serials[serial_cnt];


int8_t DMA_transmit(HardwareSerial * ptr, uint8_t dma_ch)
{
    DMA_TypeDef * dma = (DMA_TypeDef *)ptr->dma_unit_tx;
    //irqstatus_t irqs = irq_save();

    /* Check if TX ongoing... */
    /*if (!LL_DMA_IsEnabledChannel(dma, dma_ch))*/ {
        /* Check if something to send */
        uint32_t len = 0, data = (uint32_t)ptr->tx_pool_get(&len);
        if (data && len) {
            //LL_DMA_DisableChannel(dma, dma_ch);
            /* Clear all irq flags */
            WRITE_REG(dma->IFCR, 0xF << (dma_ch - 1));
            /* Set source address */
            LL_DMA_SetMemoryAddress(dma, dma_ch, data);
            LL_DMA_SetDataLength(dma, dma_ch, len);
            /* enable tx */
            ptr->hw_enable_transmitter();
            /* Start transfer */
            LL_DMA_EnableChannel(dma, dma_ch);
            return 0;
        }
    }

    //irq_restore(irqs);
    return -1;
}

void USART_IDLE_IRQ_handler(uint32_t index)
{
    if (serial_cnt <= index) return;
    HardwareSerial *serial = _started_serials[index];
    if (!serial) return;
    USART_TypeDef * uart = (USART_TypeDef *)serial->p_usart;

    uint32_t SR = uart->StatReg, CR1 = uart->CR1;

    /* Check for IDLE line interrupt */
    if ((SR & USART_SR_IDLE) && (CR1 & USART_CR1_IDLEIE)) {
        uint8_t head_pos = sizeof(serial->rx_buffer) -
            LL_DMA_GetDataLength((DMA_TypeDef *)serial->dma_unit_rx, serial->dma_ch_rx);
        uint8_t head = serial->rx_head;
        uint8_t tail = head + (uint8_t)(head_pos - head);
        if (tail >= serial->rx_tail)
            serial->rx_tail = tail;
        serial->rx_head = head_pos;
    }
    /* Check for RX data */
    else if ((SR & (USART_SR_RXNE | USART_SR_ORE)) && (CR1 & USART_CR1_RXNEIE)) {
        uint8_t next = serial->rx_head;
        uint8_t data = uart->RxDataReg;
        if ((next + 1) != serial->rx_tail) {
            serial->rx_buffer[next] = data;
            serial->rx_head = next + 1;
        }
    }

    // Check if TX is enabled and TX Empty IRQ triggered
    if ((SR & USART_SR_TXE) && (CR1 & USART_CR1_TXEIE)) {
        //  Check if data available
        if (serial->tx_head <= serial->tx_tail)
            serial->hw_enable_receiver();
        else
            uart->TxDataReg = serial->tx_buffer[serial->tx_tail++];
    }
}

void USARTx_DMA_handler(uint32_t index)
{
    if (serial_cnt <= index) return;
    HardwareSerial *serial = _started_serials[index];
    if (!serial) return;
    uint32_t channel = serial->dma_ch_tx;
    DMA_TypeDef * dma = (DMA_TypeDef *)serial->dma_unit_tx;
    uint32_t sr = dma->ISR, mask = (DMA_ISR_TCIF1_Msk << (4 * (channel-1)));
    if (sr & mask) {
        LL_DMA_DisableChannel(dma, channel);
        if (serial && DMA_transmit(serial, channel) < 0)
            serial->hw_enable_receiver();
        dma->IFCR = mask;
    }
}

HardwareSerial::HardwareSerial(uint32_t rx, uint32_t tx, uint8_t dma)
{
    USART_TypeDef * uart = NULL;
    rx_pin = rx;
    tx_pin = tx;

    if (rx == GPIO('A', 10) && tx == GPIO('A', 9))
        uart = USART1;
    else if (rx == GPIO('A', 3) && tx == GPIO('A', 2))
        uart = USART2;
#ifdef USART3
    else if ((rx == GPIO('D', 9) && tx == GPIO('D', 8)) ||
             (rx == GPIO('B', 11) && tx == GPIO('B', 10)))
        uart = USART3;
#endif // USART3
    p_usart = uart;
    p_use_dma = dma && (UART_USE_DMA_RX | UART_USE_DMA_TX);
}

HardwareSerial::HardwareSerial(void *peripheral, uint8_t dma)
{
    USART_TypeDef * uart = (USART_TypeDef *)peripheral;
    if (uart == USART1) {
        rx_pin = GPIO('A', 10);
        tx_pin = GPIO('A', 9);
#ifdef USART2
    } else if (uart == USART2) {
        rx_pin = GPIO('A', 3);
        tx_pin = GPIO('A', 2);
#endif // USART2
#ifdef USART3
    } else if (uart == USART3) {
        rx_pin = GPIO('B', 11);
        tx_pin = GPIO('B', 10);
#endif // USART3
    }
    p_usart = uart;
    p_use_dma = dma && (UART_USE_DMA_RX | UART_USE_DMA_TX);
}

void HardwareSerial::begin(unsigned long baud, uint8_t mode)
{
    USART_TypeDef * uart = (USART_TypeDef *)p_usart;

    if (uart == USART1) {
        usart_irq = USART1_IRQn;
        write_u32(&_started_serials[0], (uint32_t)this);
#ifdef USART2
    } else if (uart == USART2) {
        usart_irq = USART2_IRQn;
        write_u32(&_started_serials[1], (uint32_t)this);
#endif // USART2
#ifdef USART3
    } else if (uart == USART3) {
        usart_irq = USART3_IRQn;
        write_u32(&_started_serials[2], (uint32_t)this);
#endif // USART3
    } else {
        // Invalid HW UART config!
        return;
    }

    dma_ch_tx = dma_channel_get((uint32_t)uart, DMA_USART_TX, 0);
    dma_ch_rx = dma_channel_get((uint32_t)uart, DMA_USART_RX, 0);
    dma_irq_tx = dma_irq_get((uint32_t)uart, DMA_USART_TX, 0);
    dma_irq_rx = dma_irq_get((uint32_t)uart, DMA_USART_RX, 0);

    /* Init RX buffer */
    rx_head = rx_tail = 0;
    tx_head = tx_tail = 0;

    if (p_use_dma) {
        DMA_TypeDef * dmaptr;
        /*********** USART DMA Init ***********/
    #if UART_USE_DMA_RX
        dmaptr = (DMA_TypeDef *)dma_get((uint32_t)uart, DMA_USART_RX, 0);
        enable_pclock((uint32_t)dmaptr);
        dma_unit_rx = (uint32_t)dmaptr;

        /* RX DMA stream config */
        LL_DMA_DisableChannel(dmaptr, dma_ch_rx);
        LL_DMA_SetDataTransferDirection(dmaptr, dma_ch_rx, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
        LL_DMA_SetChannelPriorityLevel(dmaptr, dma_ch_rx, LL_DMA_PRIORITY_LOW);
        LL_DMA_SetMode(dmaptr, dma_ch_rx, LL_DMA_MODE_CIRCULAR);
        LL_DMA_SetPeriphIncMode(dmaptr, dma_ch_rx, LL_DMA_PERIPH_NOINCREMENT);
        LL_DMA_SetMemoryIncMode(dmaptr, dma_ch_rx, LL_DMA_MEMORY_INCREMENT);
        LL_DMA_SetPeriphSize(dmaptr, dma_ch_rx, LL_DMA_PDATAALIGN_BYTE);
        LL_DMA_SetMemorySize(dmaptr, dma_ch_rx, LL_DMA_MDATAALIGN_BYTE);
        /* Set source and target */
        LL_DMA_SetPeriphAddress(dmaptr, dma_ch_rx, (uint32_t)&uart->RxDataReg);
        LL_DMA_SetMemoryAddress(dmaptr, dma_ch_rx, (uint32_t)rx_buffer);
        LL_DMA_SetDataLength(dmaptr, dma_ch_rx, sizeof(rx_buffer));
        /* Set interrupts */
        //LL_DMA_EnableIT_HT(dmaptr, dma_ch_rx);
        //LL_DMA_EnableIT_TC(dmaptr, dma_ch_rx);
        LL_DMA_DisableIT_HT(dmaptr, dma_ch_rx);
        LL_DMA_DisableIT_TC(dmaptr, dma_ch_rx);
        /* Set interrupt configuration */
        NVIC_SetPriority((IRQn_Type)dma_irq_rx,
            NVIC_EncodePriority(NVIC_GetPriorityGrouping(), ISR_PRIO_UART_DMA, 0));
        NVIC_EnableIRQ((IRQn_Type)dma_irq_rx);
        /* Enable DMA */
        LL_DMA_EnableChannel(dmaptr, dma_ch_rx);
    #endif // UART_USE_DMA_RX

    #if UART_USE_DMA_TX
        /* Init TX list */
        tx_pool_tail = tx_pool_head = tx_pool;
        uint8_t iter;
        for (iter = 0; iter < (sizeof(tx_pool)/sizeof(tx_pool[0]) - 1); iter++) {
            tx_pool[iter].next = &tx_pool[iter+1];
        }
        tx_pool[iter].next = &tx_pool[0];

        dmaptr = (DMA_TypeDef *)dma_get((uint32_t)uart, DMA_USART_TX, 0);
        enable_pclock((uint32_t)dmaptr);
        dma_unit_tx = (uint32_t)dmaptr;

        /* TX DMA stream config */
        LL_DMA_DisableChannel(dmaptr, dma_ch_tx);
        LL_DMA_SetDataTransferDirection(dmaptr, dma_ch_tx, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
        LL_DMA_SetChannelPriorityLevel(dmaptr, dma_ch_tx, LL_DMA_PRIORITY_LOW);
        LL_DMA_SetMode(dmaptr, dma_ch_tx, LL_DMA_MODE_NORMAL);
        LL_DMA_SetPeriphIncMode(dmaptr, dma_ch_tx, LL_DMA_PERIPH_NOINCREMENT);
        LL_DMA_SetMemoryIncMode(dmaptr, dma_ch_tx, LL_DMA_MEMORY_INCREMENT);
        LL_DMA_SetPeriphSize(dmaptr, dma_ch_tx, LL_DMA_PDATAALIGN_BYTE);
        LL_DMA_SetMemorySize(dmaptr, dma_ch_tx, LL_DMA_MDATAALIGN_BYTE);
        /* Set target address */
        LL_DMA_SetPeriphAddress(dmaptr, dma_ch_tx, (uint32_t)&uart->TxDataReg);
        /* Set interrupts */
        LL_DMA_DisableIT_HT(dmaptr, dma_ch_tx);
        LL_DMA_EnableIT_TC(dmaptr, dma_ch_tx);
        /* Set interrupt configuration */
        NVIC_SetPriority((IRQn_Type)dma_irq_tx,
            NVIC_EncodePriority(NVIC_GetPriorityGrouping(), ISR_PRIO_UART_DMA, 0));
        NVIC_EnableIRQ((IRQn_Type)dma_irq_tx);
    #endif // UART_USE_DMA_TX

        (void)dmaptr;
    }

    /*********** USART Init ***********/
    enable_pclock((uint32_t)uart);
    uint32_t pclk = get_pclock_frequency((uint32_t)uart);
    uint32_t div = DIV_ROUND_CLOSEST(pclk, baud);
#ifdef USART_BRR_DIV_Mantissa_Pos
    uart->BRR = (((div / 16) << USART_BRR_DIV_Mantissa_Pos) | ((div % 16) << USART_BRR_DIV_Fraction_Pos));
#else
    uart->BRR = div;
#endif

    // Half duplex
    DR_RX = USART_CR1_UE | USART_CR1_RE;
    DR_TX = USART_CR1_UE | USART_CR1_TE;
    DR_RX |= ((p_use_dma) ? USART_CR1_IDLEIE : USART_CR1_RXNEIE);
    DR_TX |= ((p_use_dma) ? 0 : USART_CR1_TXEIE);
    if ((BUFFER_OE == UNDEF_PIN) || (!gpio_out_valid(p_duplex_pin))) {
        // Full duplex
        DR_RX |= USART_CR1_TE;
        DR_TX |= DR_RX;
    }
    hw_enable_receiver();

    NVIC_SetPriority((IRQn_Type)usart_irq,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), ISR_PRIO_UART, 0));
    NVIC_EnableIRQ((IRQn_Type)usart_irq);

    uart_config_afio((uint32_t)uart, rx_pin, tx_pin);
}

void HardwareSerial::end(void)
{
    USART_TypeDef * uart = (USART_TypeDef *)p_usart;
    if (p_use_dma) {
#if UART_USE_DMA_RX
        LL_DMA_DisableChannel((DMA_TypeDef *)dma_unit_rx, dma_ch_rx);
        NVIC_DisableIRQ((IRQn_Type)dma_irq_rx);
#endif
#if UART_USE_DMA_TX
        LL_DMA_DisableChannel((DMA_TypeDef *)dma_unit_tx, dma_ch_tx);
        NVIC_DisableIRQ((IRQn_Type)dma_irq_tx);
#endif
    }
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
#if BUFFER_OE != UNDEF_PIN && 1
    /*if (gpio_out_valid(p_duplex_pin)) {
        while (gpio_out_read(p_duplex_pin))
            ;
    }*/
#endif
    //while(read_u8(&tx_head) != read_u8(&tx_tail))
    //    ;
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
    hw_enable_transmitter();
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
        }

        memcpy(&tx_buffer[tmax], buff, len);
        write_u8(&tx_head, (tmax + len));

        //irq_restore(flag);
        hw_enable_transmitter();

        /* flush... */
        //while(read_u8(&tx_head) != read_u8(&tx_tail));
    }
    return len;
}

void HardwareSerial::hw_enable_receiver(void)
{
    USART_TypeDef * uart = (USART_TypeDef *)p_usart;
#if BUFFER_OE != UNDEF_PIN
    if (gpio_out_valid(p_duplex_pin)) {
        // Wait until transfer is completed
        while (!(uart->SR & USART_SR_TC));
        gpio_out_write(p_duplex_pin, 0);
    }
#endif // BUFFER_OE
    uart->CR1 = DR_RX;
}

void HardwareSerial::hw_enable_transmitter(void)
{
    USART_TypeDef * uart = (USART_TypeDef *)p_usart;
    uart->CR1 = DR_TX;
#if BUFFER_OE != UNDEF_PIN
    if (gpio_out_valid(p_duplex_pin)) {
        gpio_out_write(p_duplex_pin, 1);
    }
#endif // BUFFER_OE
}

#if defined(TARGET_R9M_TX)
#define PRINTF_USE_DMA 0
HardwareSerial Serial1(USART1, PRINTF_USE_DMA);
#endif
