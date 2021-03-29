#pragma once

#include "gpio.h"
#include <stdint.h>

class R9DAC
{
#if DAC_IN_USE
private:
    enum
    {
        R9_PWR_10mW = 0,
        R9_PWR_25mW,
        R9_PWR_50mW,
        R9_PWR_100mW,
        R9_PWR_250mW,
        R9_PWR_500mW,
        R9_PWR_1000mW,
        R9_PWR_2000mW,
        R9_PWR_MAX
    };

    typedef struct
    {
        uint16_t mW;
        uint8_t dB;
        uint8_t gain;
        uint16_t volts; // APC2volts*1000
    } r9dac_lut_s;

    const r9dac_lut_s LUT[R9_PWR_MAX] = {
        // mw, dB, gain, APC2volts*1000, figures assume 2dBm input
#if defined(TARGET_R9M_TX)
#if 0
        {10, 11, 9, 800},
        {25, 14, 12, 920},
        {50, 17, 15, 1030},
        {100, 20, 18, 1150},
        {250, 24, 22, 1330},
        {500, 27, 25, 1520},
        {1000, 30, 28, 1790},
        {2000, 33, 31, 2160},
#else
        // Alessandro modified values
        {10, 10, 8, 650},
        {25, 14, 12, 860},
        {50, 17, 15, 1000},
        {100, 20, 18, 1160},
        {250, 24, 22, 1420},
        {500, 27, 25, 1730},
        {1000, 30, 28, 2100},
        {2000, 33, 31, 2600}, // Danger untested at high power
#endif
#elif defined(TARGET_NAMIMNORC_TX)
        {10, 10, 8, 315},
        {25, 14, 12, 460},
        {50, 17, 15, 595},
        {100, 20, 18, 750},
        {250, 24, 22, 1125},
        {500, 27, 25, 1505},
        {1000, 30, 28, 2105},
        {2000, 30, 28, 2105},
#endif
    };

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
