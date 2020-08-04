#ifndef RX_SERVO_OUT_H_
#define RX_SERVO_OUT_H_

#include "CRSF.h"

void servo_out_init(void);
void ICACHE_RAM_ATTR servo_out_write(volatile crsf_channels_t &output);

#endif /* RX_SERVO_OUT_H_ */
