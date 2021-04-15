#pragma once

#include "gpio.h"
#include <stdint.h>

class R9DAC
{
#if DAC_IN_USE
private:
    struct gpio_out pin_RFswitch;
    struct gpio_out pin_RFamp;
    struct gpio_out pin_RFamp2;
    uint32_t CurrVoltageMV;
    uint8_t CurrVoltageRegVal;
    uint8_t ADDR;

    void setVoltageMV(uint32_t voltsMV);
    void setVoltageRegDirect(uint8_t voltReg);
    uint8_t get_lut_index(uint8_t power);
#endif /* DAC_IN_USE */
public:
    R9DAC();
    void init(uint8_t SDA_, uint8_t SCL_, uint8_t ADDR_,
              int8_t pin_switch = -1, int8_t pin_amp = -1, int8_t pin_amp2 = -1);
    void standby();
    void resume();
    void setPower(uint8_t power);
};

extern R9DAC r9dac;
