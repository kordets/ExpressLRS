#include "rx_servo_out.h"
#include "LowPassFilter.h"

#if SERVO_OUTPUTS_ENABLED

#define US_OUT_MIN 1000
#define US_OUT_MAX 2000

#define SERVO_USE_LPF 1
#define SERVO_UPDATE_INTERVAL 100 //ms

#undef CRSF_CHANNEL_IN_VALUE_MIN
#define CRSF_CHANNEL_IN_VALUE_MIN 188
#undef CRSF_CHANNEL_IN_VALUE_MAX
#define CRSF_CHANNEL_IN_VALUE_MAX 1792
#ifdef CRSF_IN_to_US
#undef CRSF_IN_to_US
#endif
#define CRSF_IN_to_US(val) MAP_U16((val), CRSF_CHANNEL_IN_VALUE_MIN, CRSF_CHANNEL_IN_VALUE_MAX, US_OUT_MIN, US_OUT_MAX)


#ifndef SERVO_OUTPUTS_THROTTLE_CH
#define SERVO_OUTPUTS_THROTTLE_CH 1
#endif
#if SERVO_OUTPUTS_THROTTLE_CH < 1 || 4 < SERVO_OUTPUTS_THROTTLE_CH
#error "Servo channel must be 1 ... 4"
#endif

/**
 * Control for servo (PWM) outputs
 */

#include "platform.h"
#include "targets.h"
#include "Arduino.h"
#include <Servo.h>

#define LPF_SMOOTHING_FACTOR 3

#if (SERVO_PIN_CH1 != UNDEF_PIN)
Servo ch1;
#if SERVO_USE_LPF
static LPF DRAM_FORCE_ATTR lpf_ch1(LPF_SMOOTHING_FACTOR);
#endif // SERVO_USE_LPF
#endif
#if (SERVO_PIN_CH1 != UNDEF_PIN)
Servo ch2;
#if SERVO_USE_LPF
static LPF DRAM_FORCE_ATTR lpf_ch2(LPF_SMOOTHING_FACTOR);
#endif // SERVO_USE_LPF
#endif
#if (SERVO_PIN_CH1 != UNDEF_PIN)
Servo ch3;
#if SERVO_USE_LPF
static LPF DRAM_FORCE_ATTR lpf_ch3(LPF_SMOOTHING_FACTOR);
#endif // SERVO_USE_LPF
#endif
#if (SERVO_PIN_CH1 != UNDEF_PIN)
Servo ch4;
#if SERVO_USE_LPF
static LPF DRAM_FORCE_ATTR lpf_ch4(LPF_SMOOTHING_FACTOR);
#endif // SERVO_USE_LPF
#endif

static uint32_t DRAM_ATTR last_update;


void servo_out_init(void) {
#if (SERVO_PIN_CH1 != UNDEF_PIN)
    ch1.attach(SERVO_PIN_CH1, 0, US_OUT_MAX);
#if SERVO_USE_LPF
    lpf_ch1.init(0);
#endif // SERVO_USE_LPF
#endif
#if (SERVO_PIN_CH2 != UNDEF_PIN)
    ch2.attach(SERVO_PIN_CH2, US_OUT_MIN, US_OUT_MAX);
#if SERVO_USE_LPF
    lpf_ch2.init(0);
#endif // SERVO_USE_LPF
#endif
#if (SERVO_PIN_CH3 != UNDEF_PIN)
    ch3.attach(SERVO_PIN_CH3, US_OUT_MIN, US_OUT_MAX);
#if SERVO_USE_LPF
    lpf_ch3.init(0);
#endif // SERVO_USE_LPF
#endif
#if (SERVO_PIN_CH4 != UNDEF_PIN)
    ch4.attach(SERVO_PIN_CH4, US_OUT_MIN, US_OUT_MAX);
#if SERVO_USE_LPF
    lpf_ch4.init(0);
#endif // SERVO_USE_LPF
#endif

    servo_out_fail_safe();
}


void ICACHE_RAM_ATTR servo_out_fail_safe(void)
{
    Servo * servos[] = {
#if (SERVO_PIN_CH1 != UNDEF_PIN)
        &ch1,
#endif
#if (SERVO_PIN_CH2 != UNDEF_PIN)
        &ch2,
#endif
#if (SERVO_PIN_CH3 != UNDEF_PIN)
        &ch3,
#endif
#if (SERVO_PIN_CH4 != UNDEF_PIN)
        &ch4,
#endif
    };
    uint16_t mid = (US_OUT_MIN + US_OUT_MAX) / 2;
    uint8_t iter;
    for (iter = 0; iter < ARRAY_SIZE(servos); iter++) {
        if (iter == (SERVO_OUTPUTS_THROTTLE_CH-1))
            servos[iter]->writeMicroseconds(US_OUT_MIN);
        else
            servos[iter]->writeMicroseconds(mid);
    }

#if 0
#if (SERVO_PIN_CH1 != UNDEF_PIN)
    ch1.writeMicroseconds(US_OUT_MIN);
#endif
#if (SERVO_PIN_CH2 != UNDEF_PIN)
    ch2.writeMicroseconds(US_OUT_MIN);
#endif
#if (SERVO_PIN_CH3 != UNDEF_PIN)
    ch3.writeMicroseconds(US_OUT_MIN);
#endif
#if (SERVO_PIN_CH4 != UNDEF_PIN)
    ch4.writeMicroseconds(US_OUT_MIN);
#endif
#endif
}


void ICACHE_RAM_ATTR servo_out_write(rc_channels_t const * const channels) {
    /* set pwm outputs for servos */
    uint32_t now = millis();

#if !SERVO_WRITE_FROM_ISR
    uint32_t irq = _SAVE_IRQ();
#endif // SERVO_WRITE_FROM_ISR

#if SERVO_USE_LPF
#if (SERVO_PIN_CH1 != UNDEF_PIN)
    uint16_t ch0_val = lpf_ch1.update(channels->ch0);
#endif
#if (SERVO_PIN_CH2 != UNDEF_PIN)
    uint16_t ch1_val = lpf_ch2.update(channels->ch1);
#endif
#if (SERVO_PIN_CH3 != UNDEF_PIN)
    uint16_t ch2_val = lpf_ch3.update(channels->ch2);
#endif
#if (SERVO_PIN_CH4 != UNDEF_PIN)
    uint16_t ch3_val = lpf_ch4.update(channels->ch3);
#endif
#else // !SERVO_USE_LPF
#if (SERVO_PIN_CH1 != UNDEF_PIN)
    uint16_t ch0_val = channels->ch0;
#endif
#if (SERVO_PIN_CH2 != UNDEF_PIN)
    uint16_t ch1_val = channels->ch1;
#endif
#if (SERVO_PIN_CH3 != UNDEF_PIN)
    uint16_t ch2_val = channels->ch2;
#endif
#if (SERVO_PIN_CH4 != UNDEF_PIN)
    uint16_t ch3_val = channels->ch3;
#endif
#endif // SERVO_USE_LPF

#if !SERVO_WRITE_FROM_ISR
    _RESTORE_IRQ(irq);
#endif // SERVO_WRITE_FROM_ISR

    if (SERVO_UPDATE_INTERVAL < (now - last_update)) {
#if (SERVO_PIN_CH1 != UNDEF_PIN)
        // 188 ... 1792
        ch1.writeMicroseconds(CRSF_IN_to_US(ch0_val));
#endif
#if (SERVO_PIN_CH2 != UNDEF_PIN)
        ch2.writeMicroseconds(CRSF_IN_to_US(ch1_val));
#endif
#if (SERVO_PIN_CH3 != UNDEF_PIN)
        ch3.writeMicroseconds(CRSF_IN_to_US(ch2_val));
#endif
#if (SERVO_PIN_CH4 != UNDEF_PIN)
        ch4.writeMicroseconds(CRSF_IN_to_US(ch3_val));
#endif
        last_update = now;
    }
}

#endif /* SERVO_OUTPUTS_ENABLED */
