#include "platform.h"
#include "targets.h"
#include "debug_elrs.h"
#include "common.h"
#include "POWERMGNT.h"
#include "gpio.h"
#include <EEPROM.h>

uint8_t rate_config_dips = 0xff;

#if (GPIO_PIN_LED != UNDEF_PIN)
struct gpio_out led_red;
#endif // GPIO_PIN_LED
#if (GPIO_PIN_LED_GREEN != UNDEF_PIN)
struct gpio_out led_green;
#define LED_STATE_GREEN(_x) gpio_out_write(led_green, ((!!(_x)) ^ GPIO_PIN_LED_GREEN_INV))
#else
#define LED_STATE_GREEN(_x) (void)(_x);
#endif // GPIO_PIN_LED_GREEN

#if (GPIO_PIN_BUZZER != UNDEF_PIN)
struct gpio_out buzzer;
static inline void PLAY_SOUND(uint32_t wait = 244, uint32_t cnt = 50)
{
    for (uint32_t x = 0; x < cnt; x++)
    {
        // 1 / 2048Hz = 488uS, or 244uS high and 244uS low to create 50% duty cycle
        gpio_out_write(buzzer, HIGH);
        delayMicroseconds(wait);
        gpio_out_write(buzzer, LOW);
        delayMicroseconds(wait);
    }
}
#else
#define PLAY_SOUND(_x, _y)
#endif

#if defined(TX_MODULE)
extern POWERMGNT PowerMgmt;
static uint32_t tone_play_cnt = 0;
static uint32_t tone_last_played = 0;
void play_tone_loop(uint32_t ms)
{
    if (tone_play_cnt && 350 < (ms - tone_last_played))
    {
        PLAY_SOUND(244, 50);
        tone_play_cnt--;
        tone_last_played = ms;
    }
}

#if defined(TARGET_R9M_TX) && !defined(R9M_lITE_TX)
#include "DAC.h"
R9DAC r9dac;
#endif /* TARGET_R9M_TX */
#endif /* TX_MODULE */


#if (GPIO_PIN_BUTTON != UNDEF_PIN)
#include "ClickButton.h"

/* Button is inverted */
ClickButton clickButton(GPIO_PIN_BUTTON, true, 40,  400,  600);

void button_handle(void)
{
    uint32_t ms = millis();
    clickButton.update(ms);
#if defined(TX_MODULE)
    if (clickButton.clicks == 1 && clickButton.lastClickLong) {
        tone_play_cnt = PowerMgmt.loopPower() + 1;
        clickButton.reset();
    } else if (clickButton.clicks == 2 && clickButton.lastClickLong) {
        extern int8_t tx_tlm_toggle(void);
        tone_play_cnt = tx_tlm_toggle() + 1;
        clickButton.reset();
    }
#elif defined(RX_MODULE)
    if (clickButton.clicks <= -(BUTTON_RESET_INTERVAL_RX / 600)) {
        platform_restart();
        clickButton.reset();
    }
#endif // RX_MODULE
}
#endif // GPIO_PIN_BUTTON

/******************* CONFIG *********************/
int8_t platform_config_load(struct platform_config &config)
{
#if STORE_TO_FLASH
    int8_t res = -1;
    struct platform_config temp;
    EEPROM.get(0, temp);
    if (temp.key == ELRS_EEPROM_KEY)
    {
        /* load ok, copy values */
        config.key = temp.key;
        config.mode = temp.mode;
        config.power = temp.power;
        config.tlm = temp.tlm;
        res = 0;
    }
#else
    int8_t res = 0;
    config.key = ELRS_EEPROM_KEY;
#endif
    if (rate_config_dips < get_elrs_airRateMax())
        config.mode = rate_config_dips;
    return res;
}

int8_t platform_config_save(struct platform_config &config)
{
    if (config.key != ELRS_EEPROM_KEY)
        return -1;
#if STORE_TO_FLASH
    EEPROM.put(0, config);
#endif
    return 0;
}

/******************* SETUP *********************/
void platform_setup(void)
{
    EEPROM.begin();

#if defined(DEBUG_SERIAL) && defined(TX_MODULE)
    // Init debug serial if not done already
    if (((void *)&DEBUG_SERIAL != (void *)&CrsfSerial) &&
        ((void *)&DEBUG_SERIAL != (void *)&Serial1))
    {
        // init debug serial
#if (GPIO_PIN_DEBUG_TX != UNDEF_PIN)
        DEBUG_SERIAL.setTx(GPIO_PIN_DEBUG_TX);
#endif
#if (GPIO_PIN_DEBUG_RX != UNDEF_PIN)
        DEBUG_SERIAL.setRx(GPIO_PIN_DEBUG_RX);
#endif
        DEBUG_SERIAL.begin(400000);
    }
#endif /* DEBUG_SERIAL */

#if defined(CTRL_SERIAL)
    CTRL_SERIAL.begin(460800);
    CTRL_SERIAL.setTimeout(5);
#endif // CTRL_SERIAL
#if defined(BT_SERIAL)
    BT_SERIAL.begin(BT_SERIAL_BAUD);
#endif // BT_SERIAL

    /**** SWTICHES ****/
#if defined(GPIO_PIN_DIP1) && defined(GPIO_PIN_DIP2)
    struct gpio_in dip = gpio_in_setup(GPIO_PIN_DIP1, 1);
    rate_config_dips = gpio_in_read(dip) ? 0u : 1u;
    rate_config_dips <<= 1;
    dip = gpio_in_setup(GPIO_PIN_DIP2, 1);
    rate_config_dips |= gpio_in_read(dip) ? 0u : 1u;
    if (rate_config_dips < get_elrs_airRateMax())
    {
        current_rate_config = rate_config_dips;
    }
#endif

    /*************** CONFIGURE LEDs *******************/
#if (GPIO_PIN_LED != UNDEF_PIN)
    led_red = gpio_out_setup(GPIO_PIN_LED, (0 ^ GPIO_PIN_LED_RED_INV));
#endif
#if (GPIO_PIN_LED_GREEN != UNDEF_PIN)
    led_green = gpio_out_setup(GPIO_PIN_LED_GREEN, (0  ^ GPIO_PIN_LED_GREEN_INV));
#endif

#if defined(TX_MODULE)
    /*************** CONFIGURE TX *******************/

#if defined(TARGET_R9M_TX) && !defined(R9M_lITE_TX)
    // DAC is used to control ADC which sets PA output
    r9dac.init(GPIO_PIN_SDA, GPIO_PIN_SCL, 0b0001100,
               GPIO_PIN_RFswitch_CONTROL, GPIO_PIN_RFamp_APC1, GPIO_PIN_RFamp_APC2);
#endif // TARGET_R9M_TX
#if (GPIO_PIN_BUZZER != UNDEF_PIN)
    buzzer = gpio_out_setup(GPIO_PIN_BUZZER, LOW);
#endif // GPIO_PIN_BUZZER

#elif defined(RX_MODULE)
    /*************** CONFIGURE RX *******************/

#endif /* RX_MODULE */

#if defined(GPIO_SELECT_RFIO_HIGH) && defined(GPIO_SELECT_RFIO_LOW)
#if defined(Regulatory_Domain_AU_433) || defined(Regulatory_Domain_EU_433)
    (void)gpio_out_setup(GPIO_SELECT_RFIO_HIGH, 0);
    (void)gpio_out_setup(GPIO_SELECT_RFIO_LOW, 1);
#elif defined(RADIO_SX128x)
#error "Not implemented!"

#else
    (void)gpio_out_setup(GPIO_SELECT_RFIO_LOW, 0);
    (void)gpio_out_setup(GPIO_SELECT_RFIO_HIGH, 1);
#endif
#endif /* RFIO HIGH / LOW */
}

void platform_mode_notify(uint8_t mode)
{
#if (GPIO_PIN_BUZZER != UNDEF_PIN) || (GPIO_PIN_LED_GREEN != UNDEF_PIN)
    for (int i = 0; i < mode; i++)
    {
        delay(300);
        LED_STATE_GREEN(HIGH);
        PLAY_SOUND(244, 50);
        delay(50);
        LED_STATE_GREEN(LOW);
    }
#endif
}

void platform_loop(int state)
{
#if (GPIO_PIN_BUTTON != UNDEF_PIN)
#if defined(TX_MODULE)
    button_handle();
    play_tone_loop(millis());
#elif defined(RX_MODULE)
    if (state == STATE_disconnected)
        button_handle();
#endif /* RX_MODULE */
#endif // GPIO_PIN_BUTTON
    (void)state;
}

void platform_connection_state(int const state)
{
    bool connected = (state == STATE_connected);
    LED_STATE_GREEN(connected);
#if defined(TX_MODULE)
    //platform_set_led(!connected);
#endif
}

void platform_set_led(bool state)
{
#if (GPIO_PIN_LED != UNDEF_PIN)
    gpio_out_write(led_red, (state ^ GPIO_PIN_LED_RED_INV));
#endif
}

void platform_restart(void)
{
    NVIC_SystemReset();
}

void platform_wifi_start(void)
{
}

void platform_wd_feed(void)
{
}

/*************************************************************************/

class CtrlSerialPrivate: public CtrlSerial
{
public:
    size_t available(void);
    uint8_t read(void);

    void write(uint8_t * buffer, size_t size);
};

void CtrlSerialPrivate::write(uint8_t * data, size_t len)
{
#if defined(CTRL_SERIAL)
    CTRL_SERIAL.write(data, len);
#endif
}

size_t CtrlSerialPrivate::available(void)
{
#if defined(CTRL_SERIAL)
    return CTRL_SERIAL.available();
#else
    return 0;
#endif
}

uint8_t CtrlSerialPrivate::read(void)
{
#if defined(CTRL_SERIAL)
    return CTRL_SERIAL.read();
#else
    return 0;
#endif
}

CtrlSerialPrivate ctrl_serial_private;
CtrlSerial& ctrl_serial = ctrl_serial_private;
