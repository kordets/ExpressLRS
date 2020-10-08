#include "RadioInterface.h"
#include "platform.h"
#include <Arduino.h>

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

void ICACHE_RAM_ATTR RadioInterface::isr_state_set(enum isr_states isr) {
    p_state_isr = isr;
}

void ICACHE_RAM_ATTR RadioInterface::WaitOnBusy() const
{
    while (unlikely(gpio_in_read(_BUSY)));
}

void ICACHE_RAM_ATTR RadioInterface::TxEnable()
{
    isr_state_set(TX_DONE);
#if TX_MODULE
    if (!gpio_out_valid(_RXen)) return;
    gpio_out_write(_RXen, 0);
    gpio_out_write(_TXen, 1);
#endif // TX_MODULE
}

void ICACHE_RAM_ATTR RadioInterface::RxEnable()
{
    isr_state_set(RX_DONE);
#if TX_MODULE
    if (!gpio_out_valid(_RXen)) return;
    gpio_out_write(_TXen, 0);
    gpio_out_write(_RXen, 1);
#endif // TX_MODULE
}

void ICACHE_RAM_ATTR RadioInterface::TxRxDisable()
{
    isr_state_set(NONE);
#if TX_MODULE
    if (!gpio_out_valid(_RXen)) return;
    gpio_out_write(_RXen, 0);
    gpio_out_write(_TXen, 0);
#endif // TX_MODULE
}
