#ifndef _RX_LINK_QUALITY_H__
#define _RX_LINK_QUALITY_H__

#include <string.h>

#define NEW_LQ_CALC 0

uint8_t linkQualityArray[100];
uint_fast8_t linkQualityArrayIndex;
#if NEW_LQ_CALC
uint_fast8_t linkQuality;
#endif

void FAST_CODE_1 LQ_nextPacket(void)
{
    uint_fast8_t index = linkQualityArrayIndex;
#if NEW_LQ_CALC
    linkQuality -= linkQualityArray[index];
#endif
    index = (index + 1) % sizeof(linkQualityArray);
    linkQualityArray[index] = 0;
    linkQualityArrayIndex = index;
}

void FAST_CODE_1 LQ_packetAck(void)
{
    linkQualityArray[linkQualityArrayIndex] = 1;
#if NEW_LQ_CALC
    linkQuality++;
#endif
}

uint_fast8_t FAST_CODE_1 LQ_getlinkQuality()
{
#if !NEW_LQ_CALC
    uint_fast8_t LQ = 0;
    int_fast8_t size = sizeof(linkQualityArray);
    while (0 <= (--size)) {
        LQ += linkQualityArray[size];
    }
    return LQ;
#else
    return linkQuality;
#endif
}

void FAST_CODE_1 LQ_reset()
{
    memset(linkQualityArray, 1, sizeof(linkQualityArray));
    linkQualityArrayIndex = 0;
#if NEW_LQ_CALC
    linkQuality = sizeof(linkQualityArray);
#endif
}

#endif // _RX_LINK_QUALITY_H__
