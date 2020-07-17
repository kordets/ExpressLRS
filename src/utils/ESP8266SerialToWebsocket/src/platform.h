#ifndef _PALTFORM_H_
#define _PALTFORM_H_

#include "platform_internal.h"

class CtrlSerial
{
public:
    size_t available(void);
    uint8_t read(void);

    void write(uint8_t data);
    void write(uint8_t * buffer, size_t size);

private:
};

extern CtrlSerial ctrl_serial;

#endif /* _PALTFORM_H_ */
