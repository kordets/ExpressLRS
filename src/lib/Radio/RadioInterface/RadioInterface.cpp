#include "RadioInterface.h"

volatile enum isr_states DRAM_ATTR RadioInterface::p_state_isr;

/////////////////////////////////////////////////////////////////

void (*RadioInterface::RXdoneCallback1)(uint8_t *, uint32_t) = RadioInterface::rx_nullCallback;
//void (*RadioInterface::RXdoneCallback2)(uint8_t *) = RadioInterface::rx_nullCallback;

void (*RadioInterface::TXdoneCallback1)() = RadioInterface::tx_nullCallback;
void (*RadioInterface::TXdoneCallback2)() = RadioInterface::tx_nullCallback;
void (*RadioInterface::TXdoneCallback3)() = RadioInterface::tx_nullCallback;
void (*RadioInterface::TXdoneCallback4)() = RadioInterface::tx_nullCallback;

/////////////////////////////////////////////////////////////////

void RadioInterface::SetPins(int rst, int dio1, int dio2, int dio3,
                             int busy, int txpin, int rxpin)
{
    _RST = rst;
    _DIO1 = dio1;
    _DIO2 = dio2,
    _DIO3 = dio3,
    _BUSY = busy;
    _TXen = txpin;
    _RXen = rxpin;

    if (-1 < rst) {
        pinMode(rst, OUTPUT);
        digitalWrite(rst, HIGH);
    }
    if (-1 < dio1)
        pinMode(dio1, INPUT);
    if (-1 < dio2)
        pinMode(dio2, INPUT);
    if (-1 < dio3)
        pinMode(dio3, INPUT);
    if (-1 < busy)
        pinMode(busy, INPUT);
    if (-1 < txpin) {
        pinMode(txpin, OUTPUT);
        digitalWrite(txpin, LOW);
    }
    if (-1 < rxpin) {
        pinMode(rxpin, OUTPUT);
        digitalWrite(rxpin, LOW);
    }
}

void RadioInterface::Reset(void)
{
    if (0 > _RST) return;

    delay(100);
    digitalWrite(_RST, LOW);
    delay(100);
    digitalWrite(_RST, HIGH);

    WaitOnBusy();
}

void ICACHE_RAM_ATTR RadioInterface::WaitOnBusy() const
{
    if (0 > _BUSY) return;
    while (unlikely(digitalRead(_BUSY)));
#if 0
#ifdef __NOP
        __NOP();
#else
        _NOP();
#endif
#endif
}

void ICACHE_RAM_ATTR RadioInterface::TxEnable() const
{
    p_state_isr = TX_DONE;
    if (0 > _RXen) return;
    digitalWrite(_RXen, LOW);
    digitalWrite(_TXen, HIGH);
}

void ICACHE_RAM_ATTR RadioInterface::RxEnable() const
{
    p_state_isr = RX_DONE;
    if (0 > _RXen) return;
    digitalWrite(_TXen, LOW);
    digitalWrite(_RXen, HIGH);
}

void ICACHE_RAM_ATTR RadioInterface::TxRxDisable() const
{
    p_state_isr = NONE;
    if (0 > _RXen) return;
    digitalWrite(_RXen, LOW);
    digitalWrite(_TXen, LOW);
}
