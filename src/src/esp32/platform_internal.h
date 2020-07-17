#ifndef __PLATFORM_H_
#define __PLATFORM_H_

#include <stdint.h>
#include <esp_attr.h>

#include <Stream.h>


#if !defined(ICACHE_RAM_ATTR)
#define ICACHE_RAM_ATTR IRAM_ATTR
#endif // !defined(ICACHE_RAM_ATTR)

#define _DISABLE_IRQ()
#define _ENABLE_IRQ()
#define _SAVE_IRQ() 0
#define _RESTORE_IRQ(_x)

class DebugSerial: public Stream
{
public:
    void begin(uint32_t) {}

    int available(void);
    int availableForWrite(void);
    int peek(void);
    int read(void);
    void flush(void);
    size_t write(uint8_t);
    size_t write(const uint8_t *buffer, size_t size);

    inline size_t write(const char * s)
    {
        return write((uint8_t*) s, strlen(s));
    }
    inline size_t write(unsigned long n)
    {
        return write((uint8_t) n);
    }
    inline size_t write(long n)
    {
        return write((uint8_t) n);
    }
    inline size_t write(unsigned int n)
    {
        return write((uint8_t) n);
    }
    inline size_t write(int n)
    {
        return write((uint8_t) n);
    }

private:
};

#ifdef WIFI_LOGGER
extern DebugSerial debug_serial;
#else
#define debug_serial Serial1
#endif

#endif /* __PLATFORM_H_ */
