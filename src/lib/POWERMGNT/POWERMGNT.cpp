#include "POWERMGNT.h"

#if defined(TARGET_R9M_TX) && !defined(R9M_lITE_TX)
#include "DAC.h"
extern R9DAC r9dac;
#endif

POWERMGNT::POWERMGNT(RadioInterface &radio)
    : p_radio(radio), p_current_power(PWR_UNKNOWN)
{
}

void POWERMGNT::Begin()
{
#if defined(TARGET_R9M_TX) && !defined(R9M_lITE_TX)
    p_radio.SetOutputPower(0b0000);
#endif
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
#if defined(TARGET_R9M_TX) && !defined(R9M_lITE_TX)
    r9dac.standby();
#endif
}

void ICACHE_RAM_ATTR POWERMGNT::pa_on(void) const
{
#if defined(TARGET_R9M_TX) && !defined(R9M_lITE_TX)
    r9dac.resume();
#endif
}

/************************** PRIVATE ******************************/

void POWERMGNT::p_set_power(PowerLevels_e power)
{
    if (power == p_current_power || power < PWR_10mW ||
        power > MaxPower)
        return;

#if RADIO_SX128x
#if defined(TARGET_MODULE_LORA1280F27)
    switch (power)
    {
    case PWR_10mW:
        p_radio.SetOutputPower(-7); // -4
        break;
    case PWR_25mW:
        p_radio.SetOutputPower(-3); // 0
        break;
    case PWR_50mW:
        p_radio.SetOutputPower(0); // 3
        break;
    case PWR_250mW:
        p_radio.SetOutputPower(8); // 12??
        break;
    case PWR_500mW:
        p_radio.SetOutputPower(13);
        break;
    case PWR_100mW:
    default:
        power = PWR_100mW;
        p_radio.SetOutputPower(3); // 6
        break;
    }

#elif defined(TARGET_MODULE_E28)
    // TODO: Measure outputs!!
    switch (power)
    {
    case PWR_10mW:
        p_radio.SetOutputPower(-17);
        break;
    case PWR_25mW:
        p_radio.SetOutputPower(-13);
        break;
    case PWR_50mW:
        p_radio.SetOutputPower(-10);
        break;
    case PWR_250mW:
        p_radio.SetOutputPower(-3);
        break;
    case PWR_500mW:
        p_radio.SetOutputPower(0);
        break;
    case PWR_100mW:
    default:
        power = PWR_100mW;
        p_radio.SetOutputPower(-7);
        break;
    }

#elif defined(TARGET_MODULE_LoRa1280)
    p_radio.SetOutputPower(13); // 12.5dBm, ~18mW

#else
#error "!! Unknown module, cannot control power !!"
#endif

#elif defined(TARGET_R9M_TX) && !defined(R9M_lITE_TX)
    r9dac.setPower(power);

#else
    switch (power)
    {
        case PWR_10mW:
            p_radio.SetOutputPower(0b1000);
            break;
        case PWR_25mW:
            p_radio.SetOutputPower(0b1100);
            break;
        case PWR_50mW:
        default:
            p_radio.SetOutputPower(0b1111);
            power = PWR_50mW;
            break;
    }
#endif
    p_current_power = power;
}
