#include "switches.h"
#include "internal.h"
#include "stm32_def.h"
#include "gpio.h"
#include "Arduino.h"
#include "targets.h"

#define CONCAT_helper(x, y) x ## y
#define CONCAT(S, N) CONCAT_helper(S, _##N)

struct switch_init {
    int32_t low;
    int32_t high;
};

struct switch_init switch_gpios[] = {
#ifdef SWITCH_AUX1
    {
#if CONCAT(SWITCH_AUX1, 1)
        .low = CONCAT(SWITCH_AUX1, 1),
#endif
#if CONCAT(SWITCH_AUX1, 2)
        .high = CONCAT(SWITCH_AUX1, 2)
#else
        .high = UNDEF_PIN
#endif
    },
#endif

#ifdef SWITCH_AUX2
    {
#if CONCAT(SWITCH_AUX2, 1)
        .low = CONCAT(SWITCH_AUX2, 1),
#endif
#if CONCAT(SWITCH_AUX2, 2)
        .high = CONCAT(SWITCH_AUX2, 2)
#else
        .high = UNDEF_PIN
#endif
    },
#endif

#ifdef SWITCH_AUX3
    {
#if CONCAT(SWITCH_AUX3, 1)
        .low = CONCAT(SWITCH_AUX3, 1),
#endif
#if CONCAT(SWITCH_AUX3, 2)
        .high = CONCAT(SWITCH_AUX3, 2)
#else
        .high = UNDEF_PIN
#endif
    },
#endif

#ifdef SWITCH_AUX4
    {
#if CONCAT(SWITCH_AUX4, 1)
        .low = CONCAT(SWITCH_AUX4, 1),
#endif
#if CONCAT(SWITCH_AUX4, 2)
        .high = CONCAT(SWITCH_AUX4, 2)
#else
        .high = UNDEF_PIN
#endif
    },
#endif

#ifdef SWITCH_AUX5
    {
#if CONCAT(SWITCH_AUX5, 1)
        .low = CONCAT(SWITCH_AUX5, 1),
#endif
#if CONCAT(SWITCH_AUX5, 2)
        .high = CONCAT(SWITCH_AUX5, 2)
#else
        .high = UNDEF_PIN
#endif
    },
#endif

#ifdef SWITCH_AUX6
    {
#if CONCAT(SWITCH_AUX6, 1)
        .low = CONCAT(SWITCH_AUX6, 1),
#endif
#if CONCAT(SWITCH_AUX6, 2)
        .high = CONCAT(SWITCH_AUX6, 2)
#else
        .high = UNDEF_PIN
#endif
    },
#endif
};

constexpr uint8_t num_switches = ARRAY_SIZE(switch_gpios);

struct switch_pins {
    struct gpio_in low;
    struct gpio_in high;
};

static struct switch_pins switches[num_switches];

void switches_init(void)
{
    uint8_t iter;
    for (iter = 0; iter < num_switches; iter++) {
        switches[iter].low =
            gpio_in_setup(switch_gpios[iter].low, 1);

        if (switch_gpios[iter].high != UNDEF_PIN) {
            switches[iter].high =
                gpio_in_setup(switch_gpios[iter].high, 1);
        }
    }
}

void switches_collect(rc_channels_t * const out)
{
#ifdef SWITCH_AUX1
    if (!gpio_in_read(switches[0].low))
        out->ch4 = CRSF_CHANNEL_IN_VALUE_MIN;
    else if (!gpio_in_valid(switches[0].high) || !gpio_in_read(switches[0].high))
        out->ch4 = CRSF_CHANNEL_IN_VALUE_MAX;
    else
        out->ch4 = CRSF_CHANNEL_IN_VALUE_MID;
#endif

#ifdef SWITCH_AUX2
    if (!gpio_in_read(switches[1].low))
        out->ch5 = CRSF_CHANNEL_IN_VALUE_MIN;
    else if (!gpio_in_valid(switches[1].high) || !gpio_in_read(switches[1].high))
        out->ch5 = CRSF_CHANNEL_IN_VALUE_MAX;
    else
        out->ch5 = CRSF_CHANNEL_IN_VALUE_MID;
#endif

#ifdef SWITCH_AUX3
    if (!gpio_in_read(switches[2].low))
        out->ch6 = CRSF_CHANNEL_IN_VALUE_MIN;
    else if (!gpio_in_valid(switches[2].high) || !gpio_in_read(switches[2].high))
        out->ch6 = CRSF_CHANNEL_IN_VALUE_MAX;
    else
        out->ch6 = CRSF_CHANNEL_IN_VALUE_MID;
#endif

#ifdef SWITCH_AUX4
    if (!gpio_in_read(switches[3].low))
        out->ch7 = CRSF_CHANNEL_IN_VALUE_MIN;
    else if (!gpio_in_read(switches[3].high))
        out->ch7 = CRSF_CHANNEL_IN_VALUE_MAX;
    else
        out->ch7 = CRSF_CHANNEL_IN_VALUE_MID;
#endif

#ifdef SWITCH_AUX5
    if (!gpio_in_read(switches[4].low))
        out->ch8 = CRSF_CHANNEL_IN_VALUE_MIN;
    else if (!gpio_in_read(switches[4].high))
        out->ch8 = CRSF_CHANNEL_IN_VALUE_MAX;
    else
        out->ch8 = CRSF_CHANNEL_IN_VALUE_MID;
#endif

#ifdef SWITCH_AUX6
    if (!gpio_in_read(switches[5].low))
        out->ch9 = CRSF_CHANNEL_IN_VALUE_MIN;
    else if (!gpio_in_read(switches[5].high))
        out->ch9 = CRSF_CHANNEL_IN_VALUE_MAX;
    else
        out->ch9 = CRSF_CHANNEL_IN_VALUE_MID;
#endif
}
