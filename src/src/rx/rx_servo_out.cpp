#include "rx_servo_out.h"

#if SERVO_OUTPUTS_ENABLED

/**
 * Control for servo (PWM) outputs
 */

#include "platform.h"
#include "targets.h"
#include <Arduino.h>
#include <Servo.h>


#if (SERVO_PIN_CH1 != UNDEF_PIN)
Servo ch1;
#endif
#if (SERVO_PIN_CH1 != UNDEF_PIN)
Servo ch2;
#endif
#if (SERVO_PIN_CH1 != UNDEF_PIN)
Servo ch3;
#endif
#if (SERVO_PIN_CH1 != UNDEF_PIN)
Servo ch4;
#endif


uint32_t last_update = 0;
constexpr uint32_t servo_update_interval = 100; //ms


void servo_out_init(void) {
#if (SERVO_PIN_CH1 != UNDEF_PIN)
    ch1.attach(SERVO_PIN_CH1);
#endif
#if (SERVO_PIN_CH2 != UNDEF_PIN)
    ch2.attach(SERVO_PIN_CH2);
#endif
#if (SERVO_PIN_CH3 != UNDEF_PIN)
    ch3.attach(SERVO_PIN_CH3);
#endif
#if (SERVO_PIN_CH4 != UNDEF_PIN)
    ch4.attach(SERVO_PIN_CH4);
#endif
}


void ICACHE_RAM_ATTR servo_out_write(volatile crsf_channels_t &channels) {
    /* set pwm outputs for servos */
    uint32_t now = millis();
    if (servo_update_interval < (now - last_update)) {
#if (SERVO_PIN_CH1 != UNDEF_PIN)
        //ch1.write(map(channels.ch0, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 0, 180));
        ch1.writeMicroseconds(map(channels.ch0, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 1000, 2000));
#endif
#if (SERVO_PIN_CH2 != UNDEF_PIN)
        //ch2.write(map(channels.ch1, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 0, 180));
        ch2.writeMicroseconds(map(channels.ch1, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 1000, 2000));
#endif
#if (SERVO_PIN_CH3 != UNDEF_PIN)
        //ch3.write(map(channels.ch2, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 0, 180));
        ch3.writeMicroseconds(map(channels.ch2, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 1000, 2000));
#endif
#if (SERVO_PIN_CH4 != UNDEF_PIN)
        //ch4.write(map(channels.ch3, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 0, 180));
        ch4.writeMicroseconds(map(channels.ch3, CRSF_CHANNEL_VALUE_MIN, CRSF_CHANNEL_VALUE_MAX, 1000, 2000));
#endif
        last_update = now;
    }
}

#endif /* SERVO_OUTPUTS_ENABLED */
