#include "switches.h"
#include "internal.h"
#include "stm32_def.h"
#include "gpio.h"
#include "Arduino.h"
#include "targets.h"

static uint32_t gpios[] = {
#ifdef SWITCH_1_1
    SWITCH_1_1,
#endif
#ifdef SWITCH_1_2
    SWITCH_1_1,
#endif
#ifdef SWITCH_2_1
    SWITCH_1_1,
#endif
#ifdef SWITCH_2_2
    SWITCH_1_1,
#endif
#ifdef SWITCH_3_1
    SWITCH_1_1,
#endif
#ifdef SWITCH_3_2
    SWITCH_1_1,
#endif
#ifdef SWITCH_4_1
    SWITCH_1_1,
#endif
#ifdef SWITCH_4_2
    SWITCH_1_1,
#endif
#ifdef SWITCH_5_1
    SWITCH_1_1,
#endif
#ifdef SWITCH_5_2
    SWITCH_1_1,
#endif
#ifdef SWITCH_6_1
    SWITCH_1_1,
#endif
#ifdef SWITCH_6_2
    SWITCH_1_1,
#endif
};

static struct gpio_out switches[8];

void switches_init(void)
{
    uint32_t gpios[] = {PB1, PB2};
}

void switches_collect(rc_channels_t * const out)
{

}
