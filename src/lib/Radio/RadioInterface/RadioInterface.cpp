#include "RadioInterface.h"
#include "platform.h"
#include <Arduino.h>

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
    _RST = gpio_out_setup(rst, HIGH);
    _DIO1 = gpio_in_setup(dio1, 0);
    _DIO2 = gpio_in_setup(dio2, 0),
    _DIO3 = gpio_in_setup(dio3, 0),
    _BUSY = gpio_in_setup(busy, 0);
#if TX_MODULE
    _TXen = gpio_out_setup(txpin, LOW);
    _RXen = gpio_out_setup(rxpin, LOW);
#endif // TX_MODULE
}

void RadioInterface::Reset(void)
{
    if (!gpio_out_valid(_RST)) return;

    delay(100);
    gpio_out_write(_RST, LOW);
    delay(100);
    gpio_out_write(_RST, HIGH);

    WaitOnBusy();
}

void ICACHE_RAM_ATTR RadioInterface::WaitOnBusy() const
{
    if (!gpio_in_valid(_BUSY)) return;
    while (unlikely(gpio_in_read(_BUSY)));
}

void ICACHE_RAM_ATTR RadioInterface::TxEnable() const
{
    p_state_isr = TX_DONE;
#if TX_MODULE
    if (!gpio_out_valid(_RXen)) return;
    gpio_out_write(_RXen, LOW);
    gpio_out_write(_TXen, HIGH);
#endif // TX_MODULE
}

void ICACHE_RAM_ATTR RadioInterface::RxEnable() const
{
    p_state_isr = RX_DONE;
#if TX_MODULE
    if (!gpio_out_valid(_RXen)) return;
    gpio_out_write(_TXen, LOW);
    gpio_out_write(_RXen, HIGH);
#endif // TX_MODULE
}

void ICACHE_RAM_ATTR RadioInterface::TxRxDisable() const
{
    p_state_isr = NONE;
#if TX_MODULE
    if (!gpio_out_valid(_RXen)) return;
    gpio_out_write(_RXen, LOW);
    gpio_out_write(_TXen, LOW);
#endif // TX_MODULE
}
