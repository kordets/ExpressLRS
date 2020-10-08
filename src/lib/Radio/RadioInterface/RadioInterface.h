#ifndef RADIO_INTERFACE_H_
#define RADIO_INTERFACE_H_

#include "platform.h"
#include "RadioHalSpi.h"
#include "gpio.h"

// default payload size is 8 bytes
#define RX_BUFFER_LEN (8)

enum isr_states
{
    NONE,
    RX_DONE,
    RX_TIMEOUT,
    TX_DONE,
    CRC_ERROR,
    CAD_DETECTED,
    CAD_DONE,
    ISR_RCVD,
};

class RadioInterface : public RadioHalSpi
{
public:
    RadioInterface(HwSpi &spi, uint8_t payload_len, uint8_t read = 0, uint8_t write = 0):
            RadioHalSpi(spi, read, write), RX_buffer_size(payload_len) {
        RXdoneCallback1 = RadioInterface::rx_nullCallback;
        TXdoneCallback1 = RadioInterface::tx_nullCallback;
    }

    void SetPins(int rst, int dio1, int dio2, int dio3,
                 int busy, int txpin, int rxpin);

    enum isr_states ICACHE_RAM_ATTR isr_state_get(void) const {
        return p_state_isr;
    }
    void ICACHE_RAM_ATTR isr_state_set(enum isr_states isr);

    ////////// Callback Function Pointers //////////
    static void rx_nullCallback(uint8_t *, uint32_t){};
    static void tx_nullCallback(void){};
    void (*RXdoneCallback1)(uint8_t *buff, uint32_t rx_us);
    void (*TXdoneCallback1)(void);

    ////////// Packet Stats //////////
    volatile int16_t LastPacketRSSI;
    volatile int8_t LastPacketSNR;
    const uint8_t RX_buffer_size;

protected:
    void Reset(void);
    void ICACHE_RAM_ATTR WaitOnBusy() const;
    void ICACHE_RAM_ATTR TxEnable();
    void ICACHE_RAM_ATTR RxEnable();
    void ICACHE_RAM_ATTR TxRxDisable();

#if TX_MODULE
    gpio_out _RXen;
    gpio_out _TXen;
#endif // TX_MODULE
    gpio_out _RST;
    gpio_in _DIO1;
    gpio_in _DIO2;
    gpio_in _DIO3;
    gpio_in _BUSY;

    ////////// Config Variables //////////
    volatile uint32_t current_freq;
    volatile int8_t current_power;

private:
    volatile enum isr_states p_state_isr;
};

#endif /* RADIO_INTERFACE_H_ */
