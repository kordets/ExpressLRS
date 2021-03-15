#ifndef _RX_LINK_QUALITY_H__
#define _RX_LINK_QUALITY_H__

#include <string.h>

uint8_t linkQualityArray[100];
uint8_t linkQualityArrayIndex;

void FAST_CODE_1 LQ_nextPacket(void)
{
    uint_fast8_t index = linkQualityArrayIndex;
    index = (index + 1) % sizeof(linkQualityArray);
    linkQualityArray[index] = 0;
    linkQualityArrayIndex = index;
}

void FAST_CODE_1 LQ_packetAck(void)
{
    linkQualityArray[linkQualityArrayIndex] = 1;
}

uint_fast8_t FAST_CODE_1 LQ_getlinkQuality()
{
    uint_fast8_t LQ = 0;
    int_fast8_t size = sizeof(linkQualityArray);
    while (0 <= (--size)) {
        LQ += linkQualityArray[size];
    }
    return LQ;
}

void FAST_CODE_1 LQ_reset()
{
    memset(linkQualityArray, 1, sizeof(linkQualityArray));
    linkQualityArrayIndex = 0;
}

#endif // _RX_LINK_QUALITY_H__
