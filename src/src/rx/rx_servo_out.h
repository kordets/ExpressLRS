#ifndef RX_SERVO_OUT_H_
#define RX_SERVO_OUT_H_

#include "rc_channels.h"
#include "platform.h"

void servo_out_init(void);
void servo_out_fail_safe(void);
void ICACHE_RAM_ATTR servo_out_write(rc_channels_t const * const output);

#endif /* RX_SERVO_OUT_H_ */
