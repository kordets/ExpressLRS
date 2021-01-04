
#if defined(TARGET_R9M_TX) && !defined(R9M_lITE_TX)

#include "DAC.h"
#include "helpers.h"
#include "LoRa_SX127x.h"
#include <Wire.h>

#define VCC 3300

R9DAC::R9DAC()
{
    CurrVoltageRegVal = 0;
    ADDR = 0;
}

void R9DAC::init(uint8_t sda, uint8_t scl, uint8_t addr,
                 int8_t pin_switch, int8_t pin_amp, int8_t pin_amp2)
{
    if (0 <= pin_switch) {
        pin_RFswitch = gpio_out_setup(pin_switch, 0);
    }
    if (0 <= pin_amp) {
        pin_RFamp = gpio_out_setup(pin_amp, 1);
    }
    if (0 <= pin_amp2) {
        pin_RFamp2 = gpio_out_setup(pin_amp2, 1);
    }

    Wire.setSDA(sda);
    Wire.setSCL(scl);
    Wire.begin();
    ADDR = addr;
}

void R9DAC::standby()
{
    if (gpio_out_valid(pin_RFswitch))
        gpio_out_write(pin_RFswitch, 1);
    if (gpio_out_valid(pin_RFamp))
        gpio_out_write(pin_RFamp, 0);
    if (gpio_out_valid(pin_RFamp2))
        gpio_out_write(pin_RFamp2, 0);
}

void R9DAC::resume()
{
    if (gpio_out_valid(pin_RFswitch))
        gpio_out_write(pin_RFswitch, 0);
    if (gpio_out_valid(pin_RFamp))
        gpio_out_write(pin_RFamp, 1);
    if (gpio_out_valid(pin_RFamp2))
        gpio_out_write(pin_RFamp2, 1);
}

void R9DAC::setVoltageMV(uint32_t const voltsMV)
{
    uint8_t ScaledVolts = MAP(voltsMV, 0, VCC, 0, 255);
    setVoltageRegDirect(ScaledVolts);
}

void R9DAC::setVoltageRegDirect(uint8_t const voltReg)
{
    if (voltReg == CurrVoltageRegVal || !ADDR)
        return;
    //uint8_t RegH = ((voltReg & 0b11110000) >> 4) + (0b0000 << 4);
    //uint8_t RegL = (voltReg & 0b00001111) << 4;
    uint8_t const RegH = (voltReg >> 4) & 0x0F;
    uint8_t const RegL = (voltReg << 4) & 0xF0;

    CurrVoltageRegVal = voltReg;

    Wire.beginTransmission(ADDR);
    Wire.write(RegH);
    Wire.write(RegL);
    Wire.endTransmission();
}

void R9DAC::setPower(PowerLevels_e &power)
{
    uint32_t const reqVolt = LUT[get_lut_index(power)].volts;
    setVoltageMV(reqVolt);
}

uint8_t R9DAC::get_lut_index(PowerLevels_e &power)
{
    uint8_t index;
    switch (power)
    {
        case PWR_25mW:
            index = R9_PWR_25mW;
            break;
        case PWR_50mW:
            index = R9_PWR_50mW;
            break;
        case PWR_100mW:
            index = R9_PWR_100mW;
            break;
        case PWR_250mW:
            index = R9_PWR_250mW;
            break;
        case PWR_500mW:
            index = R9_PWR_500mW;
            break;
        case PWR_1000mW:
            index = R9_PWR_1000mW;
            break;
        case PWR_2000mW:
            index = R9_PWR_2000mW;
            break;
        case PWR_10mW:
        default:
            index = R9_PWR_10mW;
            power = PWR_10mW;
            break;
    };

    return index;
}

#endif
