#include "debug_elrs.h"

void Printf::_putchar(char character)
{
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print(character);
#else
    (void)character;
#endif
}
