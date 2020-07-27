#include "RadioInterface.h"

volatile enum isr_states DRAM_ATTR RadioInterface::p_state_isr = NONE;

/////////////////////////////////////////////////////////////////

void (*RadioInterface::RXdoneCallback1)(uint8_t *) = RadioInterface::rx_nullCallback;
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
}

void RadioInterface::InitPins(void)
{
    if (-1 < _RST) {
        pinMode(_RST, OUTPUT);
        digitalWrite(_RST, HIGH);
    }
    if (-1 < _BUSY)
        pinMode(_BUSY, INPUT);
    if (-1 < _DIO1)
        pinMode(_DIO1, INPUT);
    if (-1 < _DIO2)
        pinMode(_DIO2, INPUT);
    if (-1 < _DIO3)
        pinMode(_DIO3, INPUT);

    if (-1 < _TXen) {
        pinMode(_TXen, OUTPUT);
        digitalWrite(_TXen, LOW);
    }
    if (-1 < _RXen) {
        pinMode(_RXen, OUTPUT);
        digitalWrite(_RXen, LOW);
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
    while (digitalRead(_BUSY) == HIGH)
#ifdef __NOP
        __NOP();
#else
        _NOP();
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
