#ifndef RADIO_INTERFACE_H_
#define RADIO_INTERFACE_H_

#include "platform.h"
#include "RadioHalSpi.h"

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
    RadioInterface(HwSpi &spi, uint32_t read = 0, uint32_t write = 0):
        RadioHalSpi(spi, read, write) {}

    void SetPins(int rst, int dio1, int dio2, int dio3,
                 int busy, int txpin, int rxpin);

    ////////// Callback Function Pointers //////////
    static void rx_nullCallback(uint8_t *){};
    static void tx_nullCallback(void){};
    static void (*RXdoneCallback1)(uint8_t *buff);
    //static void (*RXdoneCallback2)(uint8_t *buff);
    static void (*TXdoneCallback1)(void);
    static void (*TXdoneCallback2)(void);
    static void (*TXdoneCallback3)(void);
    static void (*TXdoneCallback4)(void);

    static volatile enum isr_states DRAM_ATTR p_state_isr;

    ////////// Packet Stats //////////
    volatile uint32_t LastPacketIsrMicros = 0;
    volatile int16_t LastPacketRSSI;
    volatile uint8_t LastPacketRssiRaw;
    volatile int8_t LastPacketSNR;
    volatile uint8_t NonceTX = 0;
    volatile uint8_t NonceRX = 0;

protected:
    void InitPins(void);
    void Reset(void);
    void ICACHE_RAM_ATTR WaitOnBusy() const;
    void ICACHE_RAM_ATTR TxEnable() const;
    void ICACHE_RAM_ATTR RxEnable() const;
    void ICACHE_RAM_ATTR TxRxDisable() const;

    volatile int8_t _RXen = -1;
    volatile int8_t _TXen = -1;
    volatile int8_t _DIO1 = -1;
    volatile int8_t _DIO2 = -1;
    volatile int8_t _DIO3 = -1;
    volatile int8_t _RST = -1;
    volatile int8_t _BUSY = -1;

    ////////// Config Variables //////////
    volatile uint32_t current_freq = 0;
    volatile int8_t current_power;

private:
};

#endif /* RADIO_INTERFACE_H_ */
