#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <Hash.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#if WIFI_MANAGER
#include <WiFiManager.h>
#endif /* WIFI_MANAGER */
#include <ESP8266HTTPUpdateServer.h>

#include "main.h"
#include "stm32Updater.h"
#include "stk500.h"
#include "msp.h"
#include "common_defs.h"
#include "rc_channels.h"

#ifdef WS2812_PIN
#include <Adafruit_NeoPixel.h>
#define PIXEL_FORMAT    (NEO_GRB + NEO_KHZ800)

static Adafruit_NeoPixel led_rgb;
static uint32_t led_rgb_state;
#endif

#ifdef ESP_NOW
#ifndef ESP_NOW_PEERS
#undef ESP_NOW
#endif // ESP_NOW_PEERS
#endif // ESP_NOW

#ifndef SERIAL_BAUD
#define SERIAL_BAUD 460800
#endif

#define STRINGIFY(s) #s
#define STRINGIFY_TMP(A) STRINGIFY(A)
#define CONCAT(A, B) A##B

//reference for spiffs upload https://taillieu.info/index.php/internet-of-things/esp8266/335-esp8266-uploading-files-to-the-server

//#define INVERTED_SERIAL // Comment this out for non-inverted serial


#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID "ExpressLRS AP"
#endif
#ifndef WIFI_AP_PSK
#define WIFI_AP_PSK "expresslrs"
#endif

#ifndef WIFI_TIMEOUT
#define WIFI_TIMEOUT 60 // default to 1min
#endif

#if !ESP_NOW
#define WIFI_CHANNEL 0 // Not defined
#if defined(ESP_NOW_CHANNEL)
#undef ESP_NOW_CHANNEL
#endif
#define ESP_NOW_CHANNEL 1

#else // ESP_NOW
#define WIFI_CHANNEL 2
#ifndef ESP_NOW_CHANNEL
#define ESP_NOW_CHANNEL 1
#endif
#if (ESP_NOW_CHANNEL == WIFI_CHANNEL)
#error "WiFi Channel config error! ESPNOW and WiFi must be on different channels"
#endif
#endif // ESP_NOW

MDNSResponder mdns;

ESP8266WebServer server(80);

WebSocketsServer webSocket = WebSocketsServer(81);
ESP8266HTTPUpdateServer httpUpdater;

File fsUploadFile; // a File object to temporarily store the received file
String uploadedfilename; // filename of uploaded file

//uint8_t socketNumber;

String inputString = "";
String my_ipaddress_info_str = "NA";

#if ESP_NOW
String espnow_init_info = "";
#endif
String bootlog = "";

#if CONFIG_HANDSET
/* Handset specific data */
struct gimbal_limit gimbals[TX_NUM_ANALOGS];
struct mixer mixer[TX_NUM_MIXER];
static uint8_t handset_num_switches, handset_num_aux;
static uint8_t handset_mixer_ok = 0, handset_adjust_ok = 0;
#endif

/*************************************************************************/
enum led_state_e {
  LED_OFF       = 0x000000,
  LED_INIT      = 0xffffff,
  LED_WIFI_OK   = 0x00ff00,

  LED_FREQ_900  = 0xfc00b5,
  LED_FREQ_2400 = 0x1d00fc,

  LED_WARNING   = 0xffff00,
  LED_ERROR     = 0xff0000,
};

void led_init(void)
{
#ifdef WS2812_PIN
  led_rgb.setPin(WS2812_PIN);
  led_rgb.updateType(PIXEL_FORMAT);
  led_rgb.begin();
  led_rgb.updateLength(1);
  led_rgb.setBrightness(255);
  led_rgb.fill();
  led_rgb.show();
#endif
}

void led_set(uint32_t state)
{
#ifdef WS2812_PIN
  uint32_t led_rgb_color = (state == LED_OFF) ? 0 : led_rgb.Color(
    (uint8_t)(state >> 16), (uint8_t)(state >> 8), (uint8_t)state);
  led_rgb.fill(led_rgb_color);
  led_rgb.show();
  led_rgb_state = state;
#endif
}

/*************************************************************************/

static const char PROGMEM GO_BACK[] = R"rawliteral(
<!DOCTYPE html><html><head></head>
<body><script>javascript:history.back();</script></body>
</html>
)rawliteral";

static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width = device-width, initial-scale = 1.0, maximum-scale = 1.0, user-scalable=0">
<title>ExpressLRS</title>
<style>
body {
background-color: #1E1E1E;
font-family: Arial, Helvetica, Sans-Serif;
Color: #69cbf7;
}
</style>
</head><body onload="javascript:start();">
This is a backup page<br/>
Open <a href="/update">/update</a> and update filesystem also...<br/>
</body></html>
)rawliteral";

/*************************************************************************/

class CtrlSerialPrivate: public CtrlSerial
{
public:
  size_t available(void) {
    return Serial.available();
  }
  uint8_t read(void) {
    return Serial.read();
  }

  void write(uint8_t * buffer, size_t size) {
    Serial.write(buffer, size);
  }
};

CtrlSerialPrivate my_ctrl_serial;
CtrlSerial& ctrl_serial = my_ctrl_serial;

static uint8_t settings_rate;
static uint8_t settings_power, settings_power_max;
static uint8_t settings_tlm;
static uint8_t settings_region;
static uint8_t settings_valid;
String settings_out;

MSP msp_handler;
mspPacket_t msp_out;

void SettingsWrite(uint8_t * buff, uint8_t len)
{
  // Fill MSP packet
  msp_out.type = MSP_PACKET_V1_ELRS;
  msp_out.flags = MSP_ELRS_INT;
  msp_out.payloadSize = len;
  msp_out.function = ELRS_INT_MSP_PARAMS;
  memcpy((void*)msp_out.payload, buff, len);
  // Send packet
  MSP::sendPacket(&msp_out, &my_ctrl_serial);
}

void handleSettingRate(const char * input, int num = -1)
{
  settings_out = "[INTERNAL ERROR] something went wrong";
  if (input == NULL || *input == '?') {
    settings_out = "ELRS_setting_rates_input=";
    settings_out += settings_rate;
  } else if (*input == '=') {
    input++;
    settings_out = "Setting rate: ";
    settings_out += input;
    // Write to ELRS
    char val = *input;
    uint8_t buff[] = {1, (uint8_t)(val == 'R' ? 0xff : (val - '0'))};
    SettingsWrite(buff, sizeof(buff));
  }
  if (0 <= num)
    webSocket.sendTXT(num, settings_out);
  else
    webSocket.broadcastTXT(settings_out);
}

void handleSettingPower(const char * input, int num = -1)
{
  settings_out = "[INTERNAL ERROR] something went wrong";
  if (input == NULL || *input == '?') {
    settings_out = "ELRS_setting_power_input=";
    settings_out += settings_power;
    settings_out += ",";
    settings_out += settings_power_max;
  } else if (*input == '=') {
    input++;
    settings_out = "Setting power: ";
    settings_out += input;
    // Write to ELRS
    char val = *input;
    uint8_t buff[] = {3, (uint8_t)(val == 'R' ? 0xff : (val - '0'))};
    SettingsWrite(buff, sizeof(buff));
  }
  if (0 <= num)
    webSocket.sendTXT(num, settings_out);
  else
    webSocket.broadcastTXT(settings_out);
}

void handleSettingTlm(const char * input, int num = -1)
{
  settings_out = "[INTERNAL ERROR] something went wrong";
  if (input == NULL || *input == '?') {
    settings_out = "ELRS_setting_tlm_input=";
    settings_out += settings_tlm;
  } else if (*input == '=') {
    input++;
    settings_out = "Setting telemetry: ";
    settings_out += input;
    // Write to ELRS
    char val = *input;
    uint8_t buff[] = {2, (uint8_t)(val == 'R' ? 0xff : (val - '0'))};
    SettingsWrite(buff, sizeof(buff));
  }
  if (0 <= num)
    webSocket.sendTXT(num, settings_out);
  else
    webSocket.broadcastTXT(settings_out);
}

void handleSettingRfPwr(const char * input, int num = -1)
{
  settings_out = "[INTERNAL ERROR] something went wrong";
  if (input == NULL || *input == '?') {
    return;
  } else if (*input == '=') {
    input++;
    settings_out = "Setting RF PWR: ";
    settings_out += input;
    // Write to ELRS
    char val = *input;
    if ('A' <= val)
      val = 10 + (val - 'A');
    else
      val = (val - '0');
    uint8_t buff[] = {6, (uint8_t)val};
    SettingsWrite(buff, sizeof(buff));
  }
  if (0 <= num)
    webSocket.sendTXT(num, settings_out);
  else
    webSocket.broadcastTXT(settings_out);
}

void handleSettingRfModule(const char * input, int num = -1)
{
  settings_out = "[INTERNAL ERROR] something went wrong";
  if (input == NULL || *input == '?') {
    return;
  } else if (*input == '=') {
    input++;
    settings_out = "Setting RF module: ";
    settings_out += input;
    // Write to ELRS
    char val = *input;
    if ('A' <= val)
      val = 10 + (val - 'A');
    else
      val = (val - '0');
    uint8_t buff[] = {4, (uint8_t)val};
    SettingsWrite(buff, sizeof(buff));
  }
  if (0 <= num)
    webSocket.sendTXT(num, settings_out);
  else
    webSocket.broadcastTXT(settings_out);
}

void handleSettingDomain(const char * input, int num = -1)
{
  settings_out = "[ERROR] Domain set is not supported!";
  if (input == NULL || *input == '?') {
    settings_out = "ELRS_setting_region_domain=";
    settings_out += settings_region;
  }
  if (0 <= num)
    webSocket.sendTXT(num, settings_out);
  else
    webSocket.broadcastTXT(settings_out);
}

void SettingsGet(uint8_t wsnum)
{
  if (!settings_valid) {
    /* Unknown... reguest update */
    uint8_t buff[] = {0, 0};
    SettingsWrite(buff, sizeof(buff));
  } else {
    /* Valid, just send those to client */
    delay(5);
    handleSettingDomain(NULL, wsnum);
    delay(5);
    handleSettingRate(NULL, wsnum);
    delay(5);
    handleSettingPower(NULL, wsnum);
    delay(5);
    handleSettingTlm(NULL, wsnum);
  }
}

void MspVtxWrite(const char * input, int num = -1)
{
  (void)num;
  (void)input;
  //uint8_t power = 1;
  uint16_t freq = 5840; //(uint16_t)msg[0] * 8 + msg[1]; // band * 8 + channel

  if (input[0] == '=') {
    freq = (input[1] - '0');
    freq = freq*10 + (input[2] - '0');
    freq = freq*10 + (input[3] - '0');
    freq = freq*10 + (input[4] - '0');
  }

  uint8_t vtx_cmd[] = {
    (uint8_t)freq, (uint8_t)(freq >> 8),
    //power,
    //(power == 0), // pit mode
  };

  // Fill MSP packet
  msp_out.reset();
  msp_out.type = MSP_PACKET_V1_CMD;
  msp_out.flags = MSP_VERSION | MSP_STARTFLAG;
  msp_out.function = MSP_VTX_SET_CONFIG;
  msp_out.payloadSize = sizeof(vtx_cmd);
  memcpy((void*)msp_out.payload, vtx_cmd, sizeof(vtx_cmd));
  // Send packet
  MSP::sendPacket(&msp_out, &my_ctrl_serial);
}

#if CONFIG_HANDSET
uint8_t char_to_dec(uint8_t const chr)
{
  if ('0' <= chr && chr <= '9') {
    return chr - '0';
  }
  return 0;
}

uint8_t char_to_hex(uint8_t chr)
{
  if ('A' <= chr && chr <= 'F')
    return (10 + (chr - 'A'));
  if ('a' <= chr && chr <= 'f')
    return (10 + (chr - 'a'));
  return char_to_dec(chr);
}

uint8_t char_u8_to_hex(const char * chr)
{
  uint8_t val = char_to_hex(*chr++);
  val <<= 4;
  val += char_to_hex(*chr++);
  return val;
}

uint8_t char_u8_to_dec(const char * chr)
{
  uint8_t val = char_to_dec(*chr++) * 10;
  val += char_to_dec(*chr++);
  return val;
}

uint16_t char_u12_to_hex(const char * chr)
{
  uint16_t val = char_to_hex(*chr++);
  val <<= 4;
  val += char_to_hex(*chr++);
  val <<= 4;
  val += char_to_hex(*chr++);
  return val;
}

void handleHandsetCalibrate(const char * input)
{
  uint8_t value = 0;
  // Fill MSP packet
  msp_out.reset();

  value = char_to_hex(input[3]);
  if (value == 1)
    value = GIMBAL_CALIB_LOW;
  else if (value == 2)
    value = GIMBAL_CALIB_MID;
  else if (value == 3)
    value = GIMBAL_CALIB_HIGH;

  if (!strncmp(input, "L1_", 3)) {
    value |= (GIMBAL_CALIB_L_V);
  } else if (!strncmp(input, "L2_", 3)) {
    value |= (GIMBAL_CALIB_L_H);
  } else if (!strncmp(input, "R1_", 3)) {
    value |= (GIMBAL_CALIB_R_V);
  } else if (!strncmp(input, "R2_", 3)) {
    value |= (GIMBAL_CALIB_R_H);
  }

  if (!value)
    return;

  msp_out.type = MSP_PACKET_V1_ELRS;
  msp_out.flags = MSP_ELRS_INT;
  msp_out.function = ELRS_HANDSET_CALIBRATE;
  msp_out.addByte(value);
  msp_out.addByte(1); // Start
  msp_out.setIteratorToSize();
  // Send packet
  MSP::sendPacket(&msp_out, &my_ctrl_serial);
}

void handleHandsetCalibrateResp(uint8_t * data, int num = -1)
{
  String out = "ELRS_handset_calibrate=ERROR!";
  if (0 <= num)
    webSocket.sendTXT(num, out);
  else
    webSocket.broadcastTXT(out);
}


void handleHandsetMixer(const char * input, size_t length)
{
  //webSocket.broadcastTXT(input, length);
  uint32_t iter, index, scale;
  const char * read = input;
  // Fill MSP packet
  msp_out.reset();
  msp_out.type = MSP_PACKET_V1_ELRS;
  msp_out.flags = MSP_ELRS_INT;
  msp_out.function = ELRS_HANDSET_MIXER;
  for (iter = 0; iter < ARRAY_SIZE(mixer) && ((size_t)(read - input) < length); iter++) {
    // channel index
    index = char_to_hex(*read++);
    msp_out.addByte(index);
    // channel out
    msp_out.addByte(char_to_hex(*read++));
    // inverted
    msp_out.addByte(char_to_hex(*read++));
    // Scale
    if (index < 4) {
      scale = char_u8_to_dec(read);
      msp_out.addByte(scale);

      String temp = "Gimbal: ";
      temp += index;
      temp += " Scale: ";
      temp += scale;
      webSocket.broadcastTXT(temp);

      read += 2;
    }
  }
  msp_out.setIteratorToSize();
  // Send packet
  MSP::sendPacket(&msp_out, &my_ctrl_serial);
}

void handleHandsetMixerResp(uint8_t * data, int num = -1)
{
  String out = "ELRS_handset_mixer=";
  uint8_t iter;
  if (data) {
    for (iter = 0; iter < ARRAY_SIZE(mixer); iter++) {
      if (data[0] < ARRAY_SIZE(mixer)) {
        mixer[data[0]] = (struct mixer){
          .index=data[1], .inv=data[2], .scale=data[3]};
      }
      data += 4;
    }
    handset_num_switches = *data++;
    handset_num_aux = *data++;
  }

  out += handset_num_aux;
  out += ";";
  out += handset_num_switches;
  out += ";";

  for (iter = 0; iter < ARRAY_SIZE(mixer); iter++) {
    if (iter)
      out += ',';
    out += iter;
    out += ":";
    out += mixer[iter].index;
    out += ":";
    out += mixer[iter].inv;
    out += ":";
    out += mixer[iter].scale;
  }

  if (0 <= num)
    webSocket.sendTXT(num, out);
  else
    webSocket.broadcastTXT(out);
}


void handleHandsetAdjust(const char * input)
{
  const char * temp;
  uint16_t value;
  // Fill MSP packet
  msp_out.reset();
  msp_out.type = MSP_PACKET_V1_ELRS;
  msp_out.flags = MSP_ELRS_INT;

  if (strncmp(&input[3], "min", 3) == 0)
    msp_out.function = ELRS_HANDSET_ADJUST_MIN;
  else if (strncmp(&input[3], "mid", 3) == 0)
    msp_out.function = ELRS_HANDSET_ADJUST_MID;
  else if (strncmp(&input[3], "max", 3) == 0)
    msp_out.function = ELRS_HANDSET_ADJUST_MAX;
  else
    /* not valid cmd */
    return;

  if (!strncmp(input, "L1_", 3))
    msp_out.payload[0] = GIMBAL_IDX_L1;
  else if (!strncmp(input, "L2_", 3))
    msp_out.payload[0] = GIMBAL_IDX_L2;
  else if (!strncmp(input, "R1_", 3))
    msp_out.payload[0] = GIMBAL_IDX_R1;
  else if (!strncmp(input, "R2_", 3))
    msp_out.payload[0] = GIMBAL_IDX_R2;
  else
    /* not valid cmd */
    return;

  temp = strstr((char*)input, "=");
  value = char_u12_to_hex(&temp[1]);
  msp_out.payload[1] = (uint8_t)(value >> 8);
  msp_out.payload[2] = (uint8_t)value;
  // Send packet
  msp_out.payloadSize = 3;
  MSP::sendPacket(&msp_out, &my_ctrl_serial);
}

void handleHandsetAdjustResp(uint8_t * data, int num = -1)
{
  String out = "ELRS_handset_adjust=";
  uint8_t iter;
  if (data) {
    memcpy(gimbals, data, sizeof(gimbals));
  }

  for (iter = 0; iter < ARRAY_SIZE(gimbals); iter++) {
    struct gimbal_limit * limits = &gimbals[iter];
    out += limits->low;
    out += ":";
    out += limits->mid;
    out += ":";
    out += limits->high;
    out += ";";
  }

  if (0 <= num)
    webSocket.sendTXT(num, out);
  else
    webSocket.broadcastTXT(out);
}

void HandsetConfigGet(uint8_t wsnum, uint8_t force=0)
{
  if (!handset_mixer_ok || !handset_adjust_ok || force) {
    // Fill MSP packet
    msp_out.reset();
    msp_out.type = MSP_PACKET_V1_ELRS;
    msp_out.flags = MSP_ELRS_INT;
    msp_out.function = ELRS_HANDSET_CONFIGS_LOAD;
    msp_out.payload[0] = 1;
    msp_out.payload[1] = 1;
    msp_out.payloadSize = 2;
    // Send packet
    MSP::sendPacket(&msp_out, &my_ctrl_serial);
    return;
  }

  delay(5);
  handleHandsetMixerResp(NULL, wsnum);
  delay(5);
  handleHandsetAdjustResp(NULL, wsnum);
}

void HandsetConfigSave(uint8_t wsnum)
{
  // Fill MSP packet
  msp_out.reset();
  msp_out.type = MSP_PACKET_V1_ELRS;
  msp_out.flags = MSP_ELRS_INT;
  msp_out.function = ELRS_HANDSET_CONFIGS_SAVE;
  msp_out.payload[0] = 1;
  msp_out.payload[1] = 1;
  msp_out.payloadSize = 2;
  // Send packet
  MSP::sendPacket(&msp_out, &my_ctrl_serial);
}

void handleHandsetTlmLnkStats(uint8_t * data)
{
  String out = "ELRS_tlm_uldl=";
  LinkStatsLink_t * stats = (LinkStatsLink_t*)data;
  // Uplink
  out += "ULQ:"; out += stats->uplink_Link_quality;
  out += ",UR1:"; out += (int8_t)stats->uplink_RSSI_1;
  out += ",UR2:"; out += (int8_t)stats->uplink_RSSI_2;
  out += ",USN:"; out += (int8_t)stats->uplink_SNR;
  //out += ",PWR:"; out += stats->uplink_TX_Power;
  //out += ",MO:"; out += stats->rf_Mode;
  // Downlink
  out += ",DLQ:"; out += stats->downlink_Link_quality;
  out += ",DR1:"; out += (int8_t)(stats->downlink_RSSI - 120);
  out += ",DSN:"; out += (int8_t)stats->downlink_SNR;
  webSocket.broadcastTXT(out);
}

void handleHandsetTlmBattery(uint8_t * data)
{
  String out = "ELRS_tlm_batt=";
  LinkStatsBatt_t * stats = (LinkStatsBatt_t*)data;
  out += "V:"; out += stats->voltage;
  out += ",A:"; out += stats->current;
  out += ",C:"; out += stats->capacity;
  out += ",R:"; out += stats->remaining;
  webSocket.broadcastTXT(out);
}

void handleHandsetTlmGps(uint8_t * data)
{
  String out = "ELRS_tlm_gps=";
  GpsOta_t * stats = (GpsOta_t*)data;
  out += "lat:"; out += stats->latitude;
  out += ",lon:"; out += stats->longitude;
  out += ",spe:"; out += stats->speed;
  out += ",hea:"; out += stats->heading / 10; // convert to degrees
  out += ",alt:"; out += (int)(stats->altitude - 1000); // 1000m offset
  out += ",sat:"; out += stats->satellites;
  webSocket.broadcastTXT(out);
}
#endif // CONFIG_HANDSET

#ifdef BUZZER_PIN
void beep(int note, int duration, int wait=1)
{
  tone(BUZZER_PIN, note, duration);
  if (wait)
    delay(duration);
}
#else
#define beep(N, T, W)
#endif

#if CONFIG_HANDSET

#define ADC_R1  100000U  //100k ohm
#define ADC_R2  10000U   //10k ohm
//#define ADC_VOLT(X) (((X) * ADC_R2) / (ADC_R1 + ADC_R2))
#define ADC_SCALE (ADC_R1 / ADC_R2)

#define ADC_REF_mV  (1075U * ADC_SCALE)
#define ADC_MAX     1023U
#define ADC_VOLT(X) (((uint32_t)(X) * ADC_REF_mV) / ADC_MAX)

#define BATT_WARN_DEFAULT 79  // = ~3.3V / cell
#define BATT_DEAD_DEFAULT 71  // = ~3V / cell
#define BATT_NOMINAL_mV   8400

static uint32_t batt_voltage;
static uint32_t batt_voltage_scale = 100;
static uint32_t batt_voltage_interval = 5000;
static uint32_t batt_voltage_warning = BATT_WARN_DEFAULT;
static uint32_t batt_voltage_warning_limit =
  ((BATT_WARN_DEFAULT * BATT_NOMINAL_mV) / 100);
static uint32_t batt_voltage_dead_limit =
  ((BATT_DEAD_DEFAULT * BATT_NOMINAL_mV) / 100);

static uint32_t batt_voltage_meas_last_ms;
static uint32_t batt_voltage_warning_last_ms;
static uint32_t batt_voltage_warning_timeout = 5000;
#ifdef WS2812_PIN
static uint8_t batt_voltage_last_bright;
#endif

void battery_voltage_report(int num = -1)
{
  String out = "ELRS_handset_battery=";
  out += batt_voltage;
  out += ",";
  out += batt_voltage_scale;
  out += ",";
  out += batt_voltage_warning;

  if (0 <= num)
    webSocket.sendTXT(num, out);
  else
    webSocket.broadcastTXT(out);
}

void battery_voltage_parse(const char * input, int num = -1)
{
  const char * temp = strstr(input, ",");
  uint32_t scale = char_u8_to_hex(input);
  if (50 <= scale && scale <= 150) {
    batt_voltage_scale = scale;
  }
  if (temp) {
    scale = char_u8_to_hex(&temp[1]);
    if (10 <= scale && scale <= 100) {
      batt_voltage_warning = scale;
      batt_voltage_warning_limit = ((scale * BATT_NOMINAL_mV) / 100);
    }
  }
}

void batt_voltage_measure(void)
{
  uint32_t ms = millis();
  if (batt_voltage_interval <= (uint32_t)(ms - batt_voltage_meas_last_ms)) {
    int adc = analogRead(A0);
    batt_voltage = ADC_VOLT(adc);
    batt_voltage = (batt_voltage * batt_voltage_scale) / 100;

    battery_voltage_report();

    uint32_t brightness = (batt_voltage * 255) / BATT_NOMINAL_mV;
    if (255 < brightness) {
      batt_voltage_warning_timeout = 5000;
      brightness = 255;
    } else if (batt_voltage <= batt_voltage_dead_limit) {
      batt_voltage_warning_timeout = 500;
      brightness = 0;
    } else if (batt_voltage <= batt_voltage_warning_limit) {
      batt_voltage_warning_timeout = 500 +
        5 * (int32_t)(batt_voltage - batt_voltage_warning_limit);
      if ((int32_t)batt_voltage_warning_timeout < 500) {
        batt_voltage_warning_timeout = 500;
      }
    }

#ifdef WS2812_PIN
    led_rgb.setBrightness(brightness);
#else
    (void)brightness;
#endif

    batt_voltage_meas_last_ms = ms;
  }

  if ((batt_voltage <= batt_voltage_warning_limit) &&
      (batt_voltage_warning_timeout <= (ms - batt_voltage_warning_last_ms))) {
#ifdef WS2812_PIN
    if (!batt_voltage_last_bright) {
      batt_voltage_last_bright = led_rgb.getBrightness();
      led_rgb.setBrightness(0);
      led_rgb.show();
    } else {
      led_rgb.setBrightness(batt_voltage_last_bright);
      led_set(led_rgb_state);
      batt_voltage_last_bright = 0;
    }
#endif
    // 400Hz, 50ms
    beep(400, 50, 0);

    batt_voltage_warning_last_ms = ms;
  }
}

#else
#define batt_voltage_measure()
#endif

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  char * temp;
  //Serial.printf("webSocketEvent(%d, %d, ...)\r\n", num, type);

  switch (type)
  {
  case WStype_DISCONNECTED:
    //Serial.printf("[%u] Disconnected!\r\n", num);
    break;
  case WStype_CONNECTED:
  {
    //IPAddress ip = webSocket.remoteIP(num);
    //Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\r\n", num, ip[0], ip[1], ip[2], ip[3], payload);
    //socketNumber = num;

    webSocket.sendTXT(num, my_ipaddress_info_str);
#if ESP_NOW
    webSocket.sendTXT(num, espnow_init_info);
#endif

    // Send settings
    SettingsGet(num);
#if CONFIG_HANDSET
    HandsetConfigGet(num);
    delay(5);
    battery_voltage_report(num);
#endif
  }
  break;
  case WStype_TEXT:
    //Serial.printf("[%u] get Text: %s\r\n", num, payload);
    // send data to all connected clients
    //webSocket.broadcastTXT(payload, length);

    temp = strstr((char*)payload, "stm32_cmd=");
    if (temp) {
      // Command STM32
      if (strstr((char*)&temp[10], "reset")) {
        // Reset STM32
        reset_stm32_to_app_mode();
      }
    } else {

#if CONFIG_HANDSET
      /* Handset specific */
      temp = strstr((char*)payload, "handset_");
      if (temp) {

        // handset_mixer=
        temp = strstr((char*)payload, "mixer=");
        if (temp) {
          temp = &temp[6];
          handleHandsetMixer(temp, (length - ((uintptr_t)temp - (uintptr_t)payload)));
          break;
        }

        // handset_calibrate=
        temp = strstr((char*)payload, "calibrate=");
        if (temp) {
          handleHandsetCalibrate(&temp[10]);
          break;
        }

        // handset_adjust_[axe]_[min|max]=val
        temp = strstr((char*)payload, "_adjust_");
        if (temp) {
          handleHandsetAdjust(&temp[8]);
          break;
        }

        temp = strstr((char*)payload, "_refresh");
        if (temp) {
          HandsetConfigGet(num, 1);
          break;
        }

        temp = strstr((char*)payload, "_save");
        if (temp) {
          HandsetConfigSave(num);
          break;
        }

        temp = strstr((char*)payload, "_battery_config=");
        if (temp) {
          battery_voltage_parse(&temp[16], num);
          break;
        }
      } else
#endif // CONFIG_HANDSET

      {
        // ExLRS setting commands
        temp = strstr((char*)payload, "S_rate");
        if (temp) {
          handleSettingRate(&temp[6], num);
          break;
        }
        temp = strstr((char*)payload, "S_power");
        if (temp) {
          handleSettingPower(&temp[7], num);
          break;
        }
        temp = strstr((char*)payload, "S_telemetry");
        if (temp) {
          handleSettingTlm(&temp[11], num);
          break;
        }
        temp = strstr((char*)payload, "S_vtx_freq");
        if (temp) {
          MspVtxWrite(&temp[10], num);
          break;
        }
        temp = strstr((char*)payload, "S_rf_pwr");
        if (temp) {
          handleSettingRfPwr(&temp[8], num);
          break;
        }
        temp = strstr((char*)payload, "S_rf_module");
        if (temp) {
          handleSettingRfModule(&temp[11], num);
          break;
        }
      }
    }
    break;
  case WStype_BIN:
    //Serial.printf("[%u] get binary length: %u\r\n", num, length);
    hexdump(payload, length);

    // echo data back to browser
    webSocket.sendBIN(num, payload, length);
    break;
  default:
    //Serial.printf("Invalid WStype [%d]\r\n", type);
    //webSocket.broadcastTXT("Invalid WStype: " + type);
    break;
  }
}

/***********************************************************************************/
/*************                   STM32 OTA UPGRADE                     *************/

int8_t flash_stm32(uint32_t flash_addr)
{
  int8_t result = -1;
  webSocket.broadcastTXT("STM32 Firmware Flash Requested!");
  webSocket.broadcastTXT("  the firmware file: '" + uploadedfilename + "'");
  if (uploadedfilename.endsWith("firmware.elrs")) {
    result = stk500_write_file(uploadedfilename.c_str());
  } else if (uploadedfilename.endsWith("firmware.bin")) {
    result = esp8266_spifs_write_file(uploadedfilename.c_str(), flash_addr);
    if (result == 0)
      reset_stm32_to_app_mode(); // boot into app
  } else {
    webSocket.broadcastTXT("Invalid file!");
  }
  Serial.begin(SERIAL_BAUD);
  return result;
}

void handleFileUploadEnd()
{
  uint32_t flash_base = BEGIN_ADDRESS;
  //String message = "\nRequest params:\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    String name = server.argName(i);
    String value = server.arg(i);
      //message += " " + name + ": " + value + "\n";
      if (name == "flash_address") {
        flash_base = strtol(&value.c_str()[2], NULL, 16);
        break;
      }
  }
  //webSocket.broadcastTXT(message);

  int8_t success = flash_stm32(flash_base);

  if (uploadedfilename.length() && FILESYSTEM.exists(uploadedfilename))
    FILESYSTEM.remove(uploadedfilename);

  server.sendHeader("Location", "/return");          // Redirect the client to the success page
  server.send(303);
  webSocket.broadcastTXT(
    (success) ? "Update Successful!": "Update Failure!");
  server.send((success < 0) ? 400 : 200);
}

void handleFileUpload()
{ // upload a new file to the SPIFFS
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START)
  {
    /* Remove old file */
    if (uploadedfilename.length() && FILESYSTEM.exists(uploadedfilename))
      FILESYSTEM.remove(uploadedfilename);

    FSInfo fs_info;
    if (FILESYSTEM.info(fs_info))
    {
      Dir dir = FILESYSTEM.openDir("/");
      while (dir.next()) {
        String file = dir.fileName();
        if (file.endsWith(".bin")) {
          FILESYSTEM.remove(file);
        }
      }

      String output = "Filesystem: used: ";
      output += fs_info.usedBytes;
      output += " / free: ";
      output += fs_info.totalBytes;
      webSocket.broadcastTXT(output);

      if (fs_info.usedBytes > 0) {
        //webSocket.broadcastTXT("formatting filesystem");
        //FILESYSTEM.format();
      }
    }
    else
    {
      webSocket.broadcastTXT("SPIFFs Failed to init!");
      return;
    }
    uploadedfilename = upload.filename;

    webSocket.broadcastTXT("Uploading file: " + uploadedfilename);

    if (!uploadedfilename.startsWith("/"))
    {
      uploadedfilename = "/" + uploadedfilename;
    }
    fsUploadFile = FILESYSTEM.open(uploadedfilename, "w"); // Open the file for writing in SPIFFS (create if it doesn't exist)
  }
  else if (upload.status == UPLOAD_FILE_WRITE)
  {
    if (fsUploadFile)
    {
      fsUploadFile.write(upload.buf, upload.currentSize); // Write the received bytes to the file
      String output = "Uploaded: ";
      output += fsUploadFile.position();
      output += " bytes";
      webSocket.broadcastTXT(output);
    }
  }
  else if (upload.status == UPLOAD_FILE_END)
  {
    if (fsUploadFile)
    {                       // If the file was successfully created
      String totsize = "Total uploaded size ";
      totsize += fsUploadFile.position();
      totsize += " of ";
      totsize += upload.totalSize;
      webSocket.broadcastTXT(totsize);
      server.send(100);
      fsUploadFile.close(); // Close the file again
    }
    else
    {
      server.send(500, "text/plain", "500: couldn't create file");
    }
  }
}

/***********************************************************************************/
/*************                    ESP OTA UPGRADE                      *************/
#define LOCAL_OTA 0
#if LOCAL_OTA
void handle_upgrade_error(void)
{
  server.sendHeader("Location", "/return");          // Redirect the client to the success page
  server.send(303);
  webSocket.broadcastTXT(
    (Update.hasError()) ? "Update Failure!" : "Update Successful!");
  server.send(Update.hasError() ? 200 : 400);
}

void handleEspUpgrade(void)
{
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    //Serial.setDebugOutput(true);
    WiFiUDP::stopAll();
    //Serial.printf("Update: %s\n", upload.filename.c_str());
    webSocket.broadcastTXT("*** ESP OTA ***\n  file: " + upload.filename);

    /*if (upload.name != "backpack_fw") {
      webSocket.broadcastTXT("Invalid upload type!");
      return;
    } else*/ if (!upload.filename.startsWith("backpack.bin")) {
      webSocket.broadcastTXT("Invalid file! Update aborted...");
      handle_upgrade_error();
      return;
    }

    uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
    if (!Update.begin(maxSketchSpace, U_FLASH)) { //start with max available size
      //Update.printError(Serial);
      webSocket.broadcastTXT("File is too big!");
      handle_upgrade_error();
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      //Update.printError(Serial);
      webSocket.broadcastTXT("Junk write failed!");
      handle_upgrade_error();
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) { //true to set the size to the current progress
      //Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      webSocket.broadcastTXT("Update Success! Rebooting...");
    } else {
      //Update.printError(Serial);
      webSocket.broadcastTXT("Update failed!");
    }
    //Serial.setDebugOutput(false);
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.end();
  }
  delay(0); //same as yield();
}

void handleEspUpgradeFinalize(void)
{
  server.sendHeader("Location", "/return");          // Redirect the client to the success page
  server.send(303);
  webSocket.broadcastTXT(
    (Update.hasError()) ? "Update Failure!" : "Update Successful!");
  server.send(Update.hasError() ? 200 : 400);
  //server.sendHeader("Connection", "close");
  //server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
  ESP.restart();
}
#endif // LOCAL_OTA

/***********************************************************************************/

void sendReturn()
{
  server.send_P(200, "text/html", GO_BACK);
}

void handle_recover()
{
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleMacAddress()
{
  String message = "WiFi STA MAC: ";
  message += WiFi.macAddress();
  message += "\n  - channel in use: ";
  message += wifi_get_channel();
  message += "\n  - mode: ";
  message += (uint8_t)WiFi.getMode();
  message += "\n\nWiFi SoftAP MAC: ";
  message += WiFi.softAPmacAddress();
  message += "\n  - IP: ";
  message += WiFi.softAPIP().toString();
  message += "\n";
  server.send(200, "text/plain", message);
}

void handle_fs(void)
{
  FSInfo fs_info;
  FILESYSTEM.info(fs_info);
  String message = "FS ino: used ";
  message += fs_info.usedBytes;
  message += "/";
  message += fs_info.totalBytes;
  message += "\n**** FS files ****\n";

  Dir dir = FILESYSTEM.openDir("/");
  while (dir.next()) {
      message += dir.fileName();
      if(dir.fileSize()) {
          File f = dir.openFile("r");
          message += " - ";
          message += f.size();
          message += "B";
      }
      message += "\n";
  }

  server.send(200, "text/plain", message);
}

String getContentType(String filename)
{
  if(filename.endsWith(".html"))
    return "text/html";
  else if(filename.endsWith(".css"))
    return "text/css";
  else if(filename.endsWith(".js"))
    return "application/javascript";
  else if(filename.endsWith(".ico"))
    return "image/x-icon";
  else if(filename.endsWith(".gz"))
    return "application/x-gzip";
  return "text/plain";
}

bool handleFileRead(String path)
{
  if (path.endsWith("/"))
    // Send the index file if a folder is requested
    path += "index.html";
  // Get the MIME type
  String contentType = getContentType(path);
  uint8_t pathWithGz = FILESYSTEM.exists(path + ".gz");
  if (pathWithGz || FILESYSTEM.exists(path)) {
    if (pathWithGz)
      path += ".gz";
    File file = FILESYSTEM.open(path, "r");
    server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

/******************* ESP-NOW *********************/
#if ESP_NOW

#include <espnow.h>

class CtrlSerialEspNow: public CtrlSerial
{
public:
  size_t available(void) {
    return 0; // not used
  }
  uint8_t read(void) {
    return 0; // not used
  }

  void write(uint8_t * buffer, size_t size) {
    while (size-- && p_iterator < sizeof(p_buffer)) {
      p_buffer[p_iterator++] = *buffer++;
    }
    if (p_iterator >= sizeof(p_buffer))
      p_iterator = UINT8_MAX; // error, not enough space
  }

  void reset(void) {
    p_iterator = 0;
  }
  void send_now(mspPacket_t *msp_in) {
    reset();
    if (MSP::sendPacket(msp_in, this) && p_iterator <= sizeof(p_buffer)) // check overflow
    {
      MSP::sendPacket(msp_in, this);
      esp_now_send(NULL, (uint8_t*)p_buffer, p_iterator);
      //webSocket.broadcastTXT("MSP sent!");
    }
  }

private:
  uint8_t p_buffer[128];
  uint8_t p_iterator = 0;
};

CtrlSerialEspNow esp_now_sender;

void esp_now_recv_cb(uint8_t *mac_addr, uint8_t *data, uint8_t data_len)
{
  /* No data or peer is unknown => ignore */
  if (!data_len || !esp_now_is_peer_exist(mac_addr))
    return;

  webSocket.broadcastTXT("ESP NOW message received!");

  // Pass data to ERLS
  // Note: accepts only correctly formatted MSP packets
  Serial.write((uint8_t*)data, data_len);
}

void esp_now_send_cb(uint8_t *mac_addr, u8 status) {
#if 0
  String temp = "ESPNOW Sent: ";
  temp += (status ? "FAIL" : "SUCCESS");
  webSocket.broadcastTXT(temp);
#endif
}

void init_esp_now(void)
{
  espnow_init_info = "ESP NOW init... ";

  if (esp_now_init() != 0) {
    espnow_init_info += "ESP NOW init failed!";
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_send_cb(esp_now_send_cb);
  esp_now_register_recv_cb(esp_now_recv_cb);

#ifdef ESP_NOW_PEERS
#define ESP_NOW_ETH_ALEN 6
  uint8_t peers[][ESP_NOW_ETH_ALEN] = ESP_NOW_PEERS;
  uint8_t num_peers = sizeof(peers) / ESP_NOW_ETH_ALEN;
  for (uint8_t iter = 0; iter < num_peers; iter++) {
    //esp_now_del_peer(peers[iter]);
    if (esp_now_add_peer(peers[iter], ESP_NOW_ROLE_COMBO, ESP_NOW_CHANNEL, NULL, 0) != 0) {
      espnow_init_info += ", PEER ";
      espnow_init_info += iter;
      espnow_init_info += " FAIL";
    }
  }
#endif // ESP_NOW_PEERS

  espnow_init_info += " - Init DONE!";
}
#endif // ESP_NOW
/*************************************************/

void setup()
{
  IPAddress my_ip;
  uint8_t sta_up = 0;
  rst_info *resetInfo;
  resetInfo = ESP.getResetInfoPtr();
  int reset_reason = resetInfo->reason;

  msp_handler.markPacketFree();

  /* Reset values */
  settings_rate = 1;
  settings_power = 4;
  settings_power_max = 8;
  settings_tlm = 7;
  settings_region = 255;
  settings_valid = 0;
#if CONFIG_HANDSET
  handset_num_switches = 6;
  handset_num_aux = 5;
  handset_mixer_ok = 0;
  handset_adjust_ok = 0;
#endif

  led_init();
  led_set(LED_INIT);

#ifdef BUZZER_PIN
  pinMode(BUZZER_PIN, OUTPUT);
  beep(440, 100);
#endif

  //Serial.setRxBufferSize(256);
#ifdef INVERTED_SERIAL
  // inverted serial
  Serial.begin(SERIAL_BAUD, SERIAL_8N1, SERIAL_FULL, 1, true);
#else
  // non-inverted serial
  Serial.begin(SERIAL_BAUD);
#endif

#if (BOOT0_PIN == 2 || BOOT0_PIN == 0)
  reset_stm32_to_app_mode();
#endif

  FILESYSTEM.begin();
  //FILESYSTEM.format();

  wifi_station_set_hostname("elrs_tx");

#if defined(WIFI_SSID) && defined(WIFI_PSK)
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PSK, WIFI_CHANNEL);
  }
  uint32_t i = 0, led = 0;
#define TIMEOUT (WIFI_TIMEOUT * 10)
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    if (++i > TIMEOUT) {
      break;
    }
    if ((i % 10) == 0) {
      led_set(led ? LED_INIT : LED_OFF);
      led ^= 1;
    }
  }
  sta_up = (WiFi.status() == WL_CONNECTED);

#elif WIFI_MANAGER
  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(WIFI_TIMEOUT);
  if (wifiManager.autoConnect(WIFI_AP_SSID " R9M")) {
    // AP found, connected
    sta_up = 1;
  }
#endif /* WIFI_MANAGER */

  if (!sta_up)
  {
    // WiFi not connected, Start access point
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID " R9M", WIFI_AP_PSK, ESP_NOW_CHANNEL);
  }

  led_set(LED_WIFI_OK);

#if ESP_NOW
  init_esp_now();
#endif // ESP_NOW

  my_ip = (sta_up) ? WiFi.localIP() : WiFi.softAPIP();

  if (mdns.begin("elrs_tx", my_ip))
  {
    mdns.addService("http", "tcp", 80);
    mdns.addService("ws", "tcp", 81);
  }
  my_ipaddress_info_str = "My IP address = ";
  my_ipaddress_info_str += my_ip.toString();
  my_ipaddress_info_str += " (RST: ";
  my_ipaddress_info_str += reset_reason;
  my_ipaddress_info_str += ")";

#if 0 && defined(LATEST_COMMIT)
  my_ipaddress_info_str += "\nCurrent version (SHA): ";
  uint8_t commit_sha[] = {LATEST_COMMIT};
  for (uint8_t iter = 0; iter < sizeof(commit_sha); iter++)

#endif // LATEST_COMMIT

  //Serial.print("Connect to http://elrs_tx.local or http://");
  //Serial.println(my_ip);

  server.on("/fs", handle_fs);
  server.on("/return", sendReturn);
  server.on("/mac", handleMacAddress);
#if LOCAL_OTA
  server.on("/update_tst", HTTP_POST, // ESP OTA upgrade
    handleEspUpgradeFinalize, handleEspUpgrade);
#endif // LOCAL_OTA
  server.on("/upload", HTTP_POST, // STM32 OTA upgrade
    handleFileUploadEnd, handleFileUpload);
  server.onNotFound([]() {
    if (!handleFileRead(server.uri())) {
      // No matching file, respond with a 404 (Not Found) error
      //server.send(404, "text/plain", "404: Not Found");
      handle_recover();
    }
  });

  httpUpdater.setup(&server);
  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

int serialEvent()
{
  int temp, limit = 8;
  uint8_t inChar;
  while (Serial.available() && limit--) {
    temp = Serial.read();
    if (temp < 0)
      break;

    inChar = (uint8_t)temp;
    if (msp_handler.processReceivedByte(inChar)) {
      String info = "MSP received: ";
      // msp fully received
      mspPacket_t &msp_in = msp_handler.getPacket();
      if (msp_in.type == MSP_PACKET_V1_ELRS) {
        uint8_t * payload = (uint8_t*)msp_in.payload;
        switch (msp_in.function) {
          case ELRS_INT_MSP_PARAMS: {
            info += "ELRS params";
            settings_rate = payload[0];
            settings_tlm = payload[1];
            settings_power = payload[2];
            settings_power_max = payload[3];
            settings_region = payload[4];
            settings_valid = 1;

            if (settings_region != 255) {
              if (3 <= settings_region)
                led_set(LED_FREQ_2400);
              else
                led_set(LED_FREQ_900);
            }

            handleSettingDomain(NULL);
            handleSettingRate(NULL);
            handleSettingPower(NULL);
            handleSettingTlm(NULL);
            break;
          }
#if CONFIG_HANDSET
          case ELRS_HANDSET_CALIBRATE: {
            info += "CALIBRATE error";
            handleHandsetCalibrateResp(payload);
            break;
          }
          case ELRS_HANDSET_MIXER: {
            info += "MIXER config";
            handleHandsetMixerResp(payload);
            handset_mixer_ok = 1;
            break;
          }
          case ELRS_HANDSET_ADJUST: {
            info += "ADJUST config";
            handleHandsetAdjustResp(payload);
            handset_adjust_ok = 1;
            break;
          }
          case ELRS_HANDSET_TLM_LINK_STATS: {
            info += "LNK STAT telemetry";
            handleHandsetTlmLnkStats(payload);
            break;
          }
          case ELRS_HANDSET_TLM_BATTERY: {
            info += "BATTERY telemetry";
            handleHandsetTlmBattery(payload);
            break;
          }
          case ELRS_HANDSET_TLM_GPS: {
            info += "GPS telemetry";
            handleHandsetTlmGps(payload);
            break;
          }
#endif /* CONFIG_HANDSET */
          default:
            info += "UNKNOWN";
            break;
        };
      }

      webSocket.broadcastTXT(info);

      //yield();
#if ESP_NOW
      // Send received MSP packet to clients
      esp_now_sender.send_now(&msp_in);
#endif

      msp_handler.markPacketFree();
    } else if (!msp_handler.mspOngoing()) {
      if (inChar == '\r') {
        continue;
      } else if (inChar == '\n' || 64 <= inputString.length()) {
        return 0;
      }
      if (isprint(inChar))
        inputString += (char)inChar;
    }

    //if (msp_handler.error())
    //  msp_handler.markPacketFree();

    //yield();
  }
  return -1;
}

void loop()
{
  if (0 <= serialEvent()) {
    webSocket.broadcastTXT(inputString);
    inputString = "";
  }

  server.handleClient();
  webSocket.loop();
  mdns.update();

  batt_voltage_measure();
}
