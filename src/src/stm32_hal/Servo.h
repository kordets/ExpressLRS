#ifndef _SERVO_H__
#define _SERVO_H__

#include <stdint.h>

class Servo
{
public:
    Servo() {};

    void attach(int pin, int min, int max);
    void writeMicroseconds(uint32_t us);

private:

};

#endif // _SERVO_H__
