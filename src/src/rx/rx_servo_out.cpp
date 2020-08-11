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
        //ch1.write(CRSF_IN_to_DEG(channels.ch0));
        ch1.writeMicroseconds(CRSF_IN_to_US(channels.ch0));
#endif
#if (SERVO_PIN_CH2 != UNDEF_PIN)
        //ch2.write(CRSF_IN_to_DEG(channels.ch1));
        ch2.writeMicroseconds(CRSF_IN_to_US(channels.ch1));
#endif
#if (SERVO_PIN_CH3 != UNDEF_PIN)
        //ch3.write(CRSF_IN_to_DEG(channels.ch2));
        ch3.writeMicroseconds(CRSF_IN_to_US(channels.ch2));
#endif
#if (SERVO_PIN_CH4 != UNDEF_PIN)
        //ch4.write(CRSF_IN_to_DEG(channels.ch3));
        ch4.writeMicroseconds(CRSF_IN_to_US(channels.ch3));
#endif
        last_update = now;
    }
}

#endif /* SERVO_OUTPUTS_ENABLED */
