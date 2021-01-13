#include "POWERMGNT.h"
#include "DAC.h"

/* Power arrays per module type */
typedef struct {
    int8_t power[PWR_UNKNOWN];
} PowerArray_t;

PowerArray_t power_array[MODULE_COUNT] = {
    // D, 10mW, 25mW, 50mW, 100mW, 250mW, 500mW, 1000mW, 2000mW
    {  0,    8,   12,   15,    15,    15,    15,     15,     15}, // MODULE_DEFAULT
    {  0,    0,    0,    0,     0,     0,     0,      0,      0}, // MODULE_R9M_DAC
    {  0,   -7,   -3,    0,     3,     8,    13,     13,     13}, // MODULE_LORA1280F27
    {  0,  -17,  -13,  -10,    -7,    -3,     0,      0,      0}, // MODULE_E28_2G4M12S
};

POWERMGNT::POWERMGNT()
    : p_radio(NULL), p_current_power(PWR_UNKNOWN)
{
}

void POWERMGNT::Begin(RadioInterface *radio, R9DAC *dac)
{
    p_dac = dac;
    p_radio = radio;
    if (radio && radio->GetModuleType() == MODULE_R9M_DAC && dac)
        radio->SetOutputPower(0b0000);
}

PowerLevels_e POWERMGNT::incPower()
{
    if (p_dyn_power && p_current_power < MaxPower) {
        p_set_power((PowerLevels_e)((uint8_t)p_current_power + 1));
    }
    return p_current_power;
}

PowerLevels_e POWERMGNT::decPower()
{
    if (p_dyn_power && p_current_power > PWR_10mW) {
        p_set_power((PowerLevels_e)((uint8_t)p_current_power - 1));
    }
    return p_current_power;
}

PowerLevels_e POWERMGNT::loopPower()
{
    PowerLevels_e next;
    if (p_dyn_power) {
        next = PWR_10mW;
    } else {
        next = (PowerLevels_e)((p_current_power + 1) % (MaxPower+1));
    }
    setPower(next);
    return next;
}

void POWERMGNT::setPower(PowerLevels_e power)
{
    if (power == PWR_DYNAMIC) {
        // enable dynamic power and reset current level to default
        p_dyn_power = 1;
        power = TX_POWER_DEFAULT;
    } else {
        if (power == PWR_UNKNOWN)
            power = TX_POWER_DEFAULT;
        p_dyn_power = 0;
    }

    p_set_power(power);
}

void ICACHE_RAM_ATTR POWERMGNT::pa_off(void) const
{
    if (p_dac)
        p_dac->standby();
}

void ICACHE_RAM_ATTR POWERMGNT::pa_on(void) const
{
    if (p_dac)
        p_dac->resume();
}

/************************** PRIVATE ******************************/

void POWERMGNT::p_set_power(PowerLevels_e power)
{
    if (power == p_current_power || power < PWR_10mW ||
        power > MaxPower || !p_radio)
        return;

    if (p_dac) {
        p_dac->setPower(power);
    } else {
        PowerArray_t * powers = &power_array[p_radio->GetModuleType()];
        p_radio->SetOutputPower(powers->power[power]);
    }
    p_current_power = power;
}
