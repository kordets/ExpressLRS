#ifndef _SERVO_H__
#define _SERVO_H__

#include <stdint.h>

class Servo
{
public:
    Servo() {};

    void attach(int pin, uint16_t min=1000, uint16_t max=2000);
    void writeMicroseconds(uint32_t us);

private:
    void * reg;
    uint16_t _min;
    uint16_t _max;
};

#endif // _SERVO_H__
