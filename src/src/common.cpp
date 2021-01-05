#include "common.h"
#include "platform.h"
#include "utils.h"
#include "FHSS.h"
#include "crc.h"

uint8_t current_rate_config;
const expresslrs_mod_settings_t *ExpressLRS_currAirRate;

//
// https://semtech.my.salesforce.com/sfc/p/#E0000000JelG/a/2R000000HUhK/6T9Vdb3_ldnElA8drIbPYjs1wBbhlWUXej8ZMXtZXOM
//
static expresslrs_mod_settings_t DRAM_FORCE_ATTR ExpressLRS_AirRateConfig[] = {
#if RADIO_SX128x
    // NOTE! Preamble len is calculate MANT*2^EXP when MANT is bits [3:0] and EXP is bits [7:4]
#if RADIO_SX128x_BW800
    /* 500Hz */
    {SX1280_LORA_BW_0800, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_5, 2000, 500, TLM_RATIO_1_128, FHSS_1, OSD_MODE_500Hz, 0b01100 /*12*/, 1000, 1500, 250000u}, // 1.35ms
    /* 250Hz */
    //{SX1280_LORA_BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_LI_4_6, 4000, 250, TLM_RATIO_1_64,  FHSS_1, OSD_MODE_250Hz, 0b01110 /*14*/, 1000, 1500, 250000u}, // 2.86ms
    {SX1280_LORA_BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_LI_4_7, 4000, 250, TLM_RATIO_1_64,  FHSS_1, OSD_MODE_250Hz, 0b01110 /*14*/, 1000, 1500, 250000u}, // 3.33ms
    /* 125Hz */
    {SX1280_LORA_BW_0800, SX1280_LORA_SF7, SX1280_LORA_CR_LI_4_7, 8000, 125, TLM_RATIO_1_32,  FHSS_1, OSD_MODE_125Hz, 0b11001 /*18*/, 1000, 2000, 500000u}, // 6.82ms
    /* 50Hz */
    {SX1280_LORA_BW_0800, SX1280_LORA_SF8, SX1280_LORA_CR_LI_4_7, 20000, 50, TLM_RATIO_1_16,  FHSS_1, OSD_MODE_50Hz,  0b11010 /*20*/, 1000, 2500, 750000u}, // 13.32ms
#else // 1600MHz BW
    /* 800Hz */
    //{SX1280_LORA_BW_1600, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_5, 1250, 800, TLM_RATIO_1_128, FHSS_1, OSD_MODE_800Hz, 0b01100 /*12*/, 1000, 1500, 250000u}, // 0.68ms
    /* 500Hz */
    //{SX1280_LORA_BW_1600, SX1280_LORA_SF5, SX1280_LORA_CR_LI_4_7, 2000, 500, TLM_RATIO_1_128, FHSS_1, OSD_MODE_500Hz, 0b01100 /*12*/, 1000, 1500, 250000u}, // 0.88ms
    {SX1280_LORA_BW_1600, SX1280_LORA_SF6, SX1280_LORA_CR_LI_4_6, 2000, 500, TLM_RATIO_1_128, FHSS_1, OSD_MODE_500Hz, 0b01100 /*12*/, 1000, 1500, 250000u}, // 1.35ms
    /* 250Hz */
    {SX1280_LORA_BW_1600, SX1280_LORA_SF7, SX1280_LORA_CR_LI_4_7, 4000, 250, TLM_RATIO_1_64,  FHSS_1, OSD_MODE_250Hz, 0b01110 /*14*/, 1000, 1500, 250000u}, // 3.10ms
    /* 125Hz */
    {SX1280_LORA_BW_1600, SX1280_LORA_SF8, SX1280_LORA_CR_LI_4_7, 8000, 125, TLM_RATIO_1_32,  FHSS_1, OSD_MODE_125Hz, 0b11010 /*20*/, 1000, 2000, 500000u},
    /* 50Hz */
    {SX1280_LORA_BW_1600, SX1280_LORA_SF9, SX1280_LORA_CR_LI_4_7, 20000, 50, TLM_RATIO_1_16,  FHSS_1, OSD_MODE_50Hz,  0b11010 /*20*/, 1000, 2500, 750000u},
#endif // RADIO_SX128x_BW800
#else // RADIO_SX127x
    /* 200Hz */
    //{BW_500_00_KHZ, SF_6, CR_4_5, 5000, 200, TLM_RATIO_1_64, FHSS_1, OSD_MODE_200Hz, 8, 1000, 1000, 250000u}, // 3.87ms
    {BW_500_00_KHZ, SF_6, CR_4_7, 5000, 200, TLM_RATIO_1_64, FHSS_1, OSD_MODE_200Hz, 8, 1000, 1000, 250000u}, // 4.38ms
    /* 100Hz */
    //{BW_500_00_KHZ, SF_7, CR_4_7, 10000, 100, TLM_RATIO_1_32, FHSS_1, OSD_MODE_100Hz, 8, 1000, 1500, 500000u}, // 8,77ms
    {BW_500_00_KHZ, SF_7, CR_4_8, 10000, 100, TLM_RATIO_1_32, FHSS_1, OSD_MODE_100Hz, 8, 1000, 1500, 500000u}, // 9.28ms
    /* 50Hz */
    {BW_500_00_KHZ, SF_8, CR_4_8, 20000, 50, TLM_RATIO_1_16, FHSS_1, OSD_MODE_50Hz, 8, 1000, 2000, 750000u}, // 18.56ms

#if RATE_ENABLED_25Hz
    {BW_500_00_KHZ, SF_9, CR_4_8, 40000, 25, TLM_RATIO_1_8, FHSS_1, OSD_MODE_25Hz, 10, 6000, 2500, 0},
#if RATE_ENABLED_4Hz
    {BW_500_00_KHZ, SF_12, CR_4_8, 250000, 4, TLM_RATIO_NO_TLM, FHSS_1, OSD_MODE_4Hz, 10, 6000, 2500, 0},
#endif /* RATE_ENABLED_4Hz */
#endif /* RATE_ENABLED_25Hz */
#endif /* RADIO_SX128x */
};

const expresslrs_mod_settings_t *get_elrs_airRateConfig(uint8_t rate)
{
    if (get_elrs_airRateMax() <= rate)
        return NULL;
    return &ExpressLRS_AirRateConfig[rate];
}

uint8_t get_elrs_airRateIndex(void * current)
{
    return ((uintptr_t)current - (uintptr_t)ExpressLRS_AirRateConfig) / sizeof(expresslrs_mod_settings_t);
}

uint8_t get_elrs_airRateMax(void)
{
    return sizeof(ExpressLRS_AirRateConfig) / sizeof(expresslrs_mod_settings_t);
}

#ifndef MY_UID
#error "UID is mandatory!"
#endif

uint8_t getSyncWord(void)
{
    uint8_t UID[6] = {MY_UID};
    return CalcCRC(UID, sizeof(UID));
}
