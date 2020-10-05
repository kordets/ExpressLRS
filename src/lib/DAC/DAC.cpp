
#ifdef TARGET_R9M_TX

#include "DAC.h"
#include "helpers.h"
#include "LoRa_SX127x.h"
#include <Wire.h>

#define VCC 3300

R9DAC::R9DAC()
{
    CurrVoltageMV = 0;
    CurrVoltageRegVal = 0;
    ADDR = 0;
    DAC_State = UNKNOWN;
}

void R9DAC::init(uint8_t sda, uint8_t scl, uint8_t addr, int8_t pin_switch, int8_t pin_amp)
{
    ADDR = addr;
    if (0 <= pin_switch)
    {
        pin_RFswitch = gpio_out_setup(pin_switch, 0);
    }
    if (0 <= pin_amp)
    {
        pin_RFamp = gpio_out_setup(pin_amp, 1);
    }
    DAC_State = UNKNOWN;

    Wire.setSDA(sda); // set is needed or it wont work :/
    Wire.setSCL(scl);
    Wire.begin();
}

void R9DAC::standby()
{
#if 0 // takes too long
    if (DAC_State != STANDBY)
    {
        Wire.beginTransmission(ADDR);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.endTransmission();
        DAC_State = STANDBY;
    }
#else
    if (gpio_out_valid(pin_RFswitch))
        gpio_out_write(pin_RFswitch, HIGH);
    if (gpio_out_valid(pin_RFamp))
        gpio_out_write(pin_RFamp, LOW);
#endif
}

void R9DAC::resume()
{
#if 0 // takes too long
    if (DAC_State != RUNNING)
    {
        setVoltageRegDirect(CurrVoltageRegVal);
        DAC_State = RUNNING;
    }
#else
    if (gpio_out_valid(pin_RFswitch))
        gpio_out_write(pin_RFswitch, LOW);
    if (gpio_out_valid(pin_RFamp))
        gpio_out_write(pin_RFamp, HIGH);
#endif
}

void R9DAC::setVoltageMV(uint32_t voltsMV)
{
    uint8_t ScaledVolts = MAP(voltsMV, 0, VCC, 0, 255);
    setVoltageRegDirect(ScaledVolts);
    CurrVoltageMV = voltsMV;
}

void R9DAC::setVoltageRegDirect(uint8_t voltReg)
{
    if (voltReg == CurrVoltageRegVal)
        return;

    CurrVoltageRegVal = voltReg;
    uint8_t RegH = ((voltReg & 0b11110000) >> 4) + (0b0000 << 4);
    uint8_t RegL = (voltReg & 0b00001111) << 4;

    Wire.beginTransmission(ADDR);
    Wire.write(RegH);
    Wire.write(RegL);
    Wire.endTransmission();
}

void R9DAC::setPower(PowerLevels_e &power)
{
    uint32_t reqVolt = LUT[get_lut_index(power)].volts;
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
