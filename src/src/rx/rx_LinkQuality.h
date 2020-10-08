#ifndef _RX_LINK_QUALITY_H__
#define _RX_LINK_QUALITY_H__

#include <string.h>

uint8_t linkQualityArray[100];
uint8_t linkQualityArrayIndex;

void ICACHE_RAM_ATTR LQ_nextPacket()
{
    uint_fast8_t index = linkQualityArrayIndex;
    index = (index + 1) % sizeof(linkQualityArray);
    linkQualityArray[index] = 0;
    linkQualityArrayIndex = index;
}

void ICACHE_RAM_ATTR LQ_packetAck(void)
{
    linkQualityArray[linkQualityArrayIndex] = 1;
}

uint_fast8_t ICACHE_RAM_ATTR LQ_getlinkQuality()
{
    uint_fast8_t LQ = 0;
    int_fast8_t size = sizeof(linkQualityArray);
    while (0 <= (--size)) {
        LQ += linkQualityArray[size];
    }
    return LQ;
}

void ICACHE_RAM_ATTR LQ_reset()
{
    memset(linkQualityArray, 1, sizeof(linkQualityArray));
    linkQualityArrayIndex = 0;
}

#endif // _RX_LINK_QUALITY_H__
