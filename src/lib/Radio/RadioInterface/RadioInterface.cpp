#include "RadioInterface.h"
#include "platform.h"
#include <Arduino.h>

volatile enum isr_states DRAM_ATTR RadioInterface::p_state_isr;

/////////////////////////////////////////////////////////////////

void RadioInterface::SetPins(int rst, int dio1, int dio2, int dio3,
                             int busy, int txpin, int rxpin)
{
    _RST = gpio_out_setup(rst, 1);
    _DIO1 = gpio_in_setup(dio1, 0);
    _DIO2 = gpio_in_setup(dio2, 0),
    _DIO3 = gpio_in_setup(dio3, 0),
    _BUSY = gpio_in_setup(busy, 0);
#if TX_MODULE
    _TXen = gpio_out_setup(txpin, 0);
    _RXen = gpio_out_setup(rxpin, 0);
#endif // TX_MODULE
}

void RadioInterface::Reset(void)
{
    if (gpio_out_valid(_RST)) {
        delay(100);
        gpio_out_write(_RST, 0);
        delay(100);
        gpio_out_write(_RST, 1);
    }
}

void ICACHE_RAM_ATTR RadioInterface::WaitOnBusy() const
{
    while (unlikely(gpio_in_read(_BUSY)));
}

void ICACHE_RAM_ATTR RadioInterface::TxEnable() const
{
    p_state_isr = TX_DONE;
#if TX_MODULE
    if (!gpio_out_valid(_RXen)) return;
    gpio_out_write(_RXen, 0);
    gpio_out_write(_TXen, 1);
#endif // TX_MODULE
}

void ICACHE_RAM_ATTR RadioInterface::RxEnable() const
{
    p_state_isr = RX_DONE;
#if TX_MODULE
    if (!gpio_out_valid(_RXen)) return;
    gpio_out_write(_TXen, 0);
    gpio_out_write(_RXen, 1);
#endif // TX_MODULE
}

void ICACHE_RAM_ATTR RadioInterface::TxRxDisable() const
{
    p_state_isr = NONE;
#if TX_MODULE
    if (!gpio_out_valid(_RXen)) return;
    gpio_out_write(_RXen, 0);
    gpio_out_write(_TXen, 0);
#endif // TX_MODULE
}
