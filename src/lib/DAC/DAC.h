#pragma once

#include "POWERMGNT.h"
#include "gpio.h"
#include <stdint.h>

class R9DAC
{
#if defined(TARGET_R9M_TX) && !defined(R9M_lITE_TX)
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
    };

    struct gpio_out pin_RFswitch;
    struct gpio_out pin_RFamp;
    struct gpio_out pin_RFamp2;
    uint32_t CurrVoltageMV;
    uint8_t CurrVoltageRegVal;
    uint8_t ADDR;

    void setVoltageMV(uint32_t voltsMV);
    void setVoltageRegDirect(uint8_t voltReg);
    uint8_t get_lut_index(PowerLevels_e &power);
#endif
public:
    R9DAC();
    void init(uint8_t SDA_, uint8_t SCL_, uint8_t ADDR_,
              int8_t pin_switch = -1, int8_t pin_amp = -1, int8_t pin_amp2 = -1);
    void standby();
    void resume();
    void setPower(PowerLevels_e &power);
};

extern R9DAC r9dac;
