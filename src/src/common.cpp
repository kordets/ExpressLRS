#include "common.h"
#include "platform.h"
#include "utils.h"
#include "FHSS.h"
#include "crc.h"
#include <Arduino.h>

volatile uint8_t current_rate_config = RATE_DEFAULT;

//
// https://semtech.my.salesforce.com/sfc/p/#E0000000JelG/a/2R000000HUhK/6T9Vdb3_ldnElA8drIbPYjs1wBbhlWUXej8ZMXtZXOM
//
const expresslrs_mod_settings_s DRAM_ATTR ExpressLRS_AirRateConfig[RATE_MAX] = {
#if RADIO_SX128x
    // NOTE! Preamble len is calculate MANT*2^EXP when MANT is bits [3:0] and EXP is bits [7:4]
#if RADIO_SX128x_BW800
    /* 250Hz */
    {SX1280_LORA_BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_LI_4_6, 4000, 250, TLM_RATIO_1_64, FHSS_1, 0b01110 /*14*/, RATE_200HZ, 1000, 1500, 250000u},
    /* 125Hz */
    {SX1280_LORA_BW_0800, SX1280_LORA_SF7, SX1280_LORA_CR_LI_4_7, 8000, 125, TLM_RATIO_1_32, FHSS_1, 0b11001 /*18*/, RATE_100HZ, 1000, 2000, 500000u},
    /* 50Hz */
    {SX1280_LORA_BW_0800, SX1280_LORA_SF8, SX1280_LORA_CR_LI_4_7, 20000, 50, TLM_RATIO_1_16, FHSS_1, 0b11010 /*20*/, RATE_50HZ, 1000, 2500, 750000u},
#else
    /* 250Hz */
    {SX1280_LORA_BW_1600, SX1280_LORA_SF7, SX1280_LORA_CR_LI_4_7, 4000, 250, TLM_RATIO_1_64, FHSS_1, 0b01100 /*12*/, RATE_200HZ, 1000, 1500, 250000u},
    /* 125Hz */
    {SX1280_LORA_BW_1600, SX1280_LORA_SF8, SX1280_LORA_CR_LI_4_7, 8000, 125, TLM_RATIO_1_32, FHSS_1, 0b11010 /*20*/, RATE_100HZ, 1000, 2000, 500000u},
    /* 50Hz */
    {SX1280_LORA_BW_1600, SX1280_LORA_SF9, SX1280_LORA_CR_LI_4_7, 20000, 50, TLM_RATIO_1_16, FHSS_1, 0b11010 /*20*/, RATE_50HZ, 1000, 2500, 750000u},
#endif
#else
    /* 200Hz */
    {BW_500_00_KHZ, SF_6, CR_4_5, 5000, 200, TLM_RATIO_1_64, FHSS_1, 8, RATE_200HZ, 1000, 1500, 250000u}, // 8B
    //{BW_500_00_KHZ, SF_6, CR_4_7, 5000, 200, TLM_RATIO_1_64, FHSS_1, 8, RATE_200HZ, 1000, 1500, 250000u},
    /* 100Hz */
    {BW_500_00_KHZ, SF_7, CR_4_7, 10000, 100, TLM_RATIO_1_32, FHSS_1, 8, RATE_100HZ, 1000, 2000, 500000u}, // 9B
    /* 50Hz */
    {BW_500_00_KHZ, SF_8, CR_4_8, 20000, 50, TLM_RATIO_1_16, FHSS_1, 8, RATE_50HZ, 1000, 2500, 750000u}, // 11B

#if RATE_MAX > RATE_50HZ
    {BW_500_00_KHZ, SF_9, CR_4_8, 40000, 25, TLM_RATIO_1_8, FHSS_1, 10, RATE_25HZ, 6000, 2500, 0},
#if RATE_MAX > (RATE_50HZ + 1)
    {BW_500_00_KHZ, SF_12, CR_4_8, 250000, 4, TLM_RATIO_NO_TLM, FHSS_1, 10, RATE_4HZ, 6000, 2500, 0},
#endif
#endif
#endif
};

const expresslrs_mod_settings_s *get_elrs_airRateConfig(uint8_t rate)
{
    if (RATE_MAX <= rate)
        return NULL;
    return &ExpressLRS_AirRateConfig[rate];
}

volatile const expresslrs_mod_settings_s *ExpressLRS_currAirRate = NULL;

#ifndef MY_UID
#error "UID is mandatory!"
#else
uint8_t const DRAM_ATTR UID[6] = {MY_UID};
#endif

uint8_t getSyncWord(void)
{
#if RADIO_SX128x
    return 0;
#else // RADIO_SX128x
#if 1
    // TX: 0x1d
    // RX: 0x61 -> NOK
    // RX: 0x1E, 0x1F, 0x2d, 0x3d, 0x4d, 0x6d -> OK
    // RX: 0x20, 0x30, 0x40 -> NOK
    uint8_t syncw = CalcCRC(UID, sizeof(UID));
    if (syncw == SX127X_SYNC_WORD_LORAWAN)
        syncw += 0x1;
    return syncw;
#else
    return SX127X_SYNC_WORD;
#endif
#endif // RADIO_SX128x
}
