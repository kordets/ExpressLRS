#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#include "platform_internal.h"
#include <stdint.h>
#include <stddef.h>

/** EEPROM storage key
* v0 - initial
* v1 - tlm added
* v2 - power range changed (dynamic added)
*/
#define ELRS_EEPROM_KEY 0x454c5202 // ELR + version nbr

struct platform_config
{
    uint32_t key;
    uint32_t mode;
    uint32_t power;
    uint32_t tlm;
};

int8_t platform_config_load(struct platform_config &config);
int8_t platform_config_save(struct platform_config &config);
void platform_setup(void);
void platform_mode_notify(void);
void platform_loop(int state);
void platform_connection_state(int state);
void platform_set_led(bool state);
void platform_restart(void);
void platform_wd_feed(void);
void platform_wifi_start(void);
void platform_radio_force_stop(void);

class CtrlSerial
{
public:
    virtual size_t available(void) = 0;
    virtual uint8_t read(void) = 0;

    void write(uint8_t data) {
        write(&data, 1);
    }
    virtual void write(uint8_t * buffer, size_t size) = 0;

private:
};

// NOTE! tx_main uses CtrlSerial class for RX and TX
extern CtrlSerial& ctrl_serial;

#endif // _PLATFORM_H_
