#ifndef __RC_CHANNELS_H
#define __RC_CHANNELS_H

#include "platform.h"
#include "CRSF.h" // N_SWITCHES
#include "msp.h"
#include <stdint.h>

#define OTA_PACKET_DATA     6
#define OTA_PACKET_CRC      2
#define OTA_PACKET_SIZE     (OTA_PACKET_DATA+OTA_PACKET_CRC)

// current and sent switch values
#define N_CONTROLS 4
#define N_SWITCHES 8
//#define N_CHANNELS 16
#define N_CHANNELS (N_CONTROLS + N_SWITCHES)

// expresslrs packet header types
// 00 -> standard 4 channel data packet
// 01 -> switch data packet
// 10 -> sync packet with hop data
// 11 -> tlm packet (MSP)
enum
{
    UL_PACKET_UNKNOWN = 0b00,
    UL_PACKET_RC_DATA = 0b01,
    UL_PACKET_SYNC = 0b10,
    UL_PACKET_MSP = 0b11,
};

enum
{
    DL_PACKET_FREE1 = 0b00,
    DL_PACKET_TLM_MSP = 0b01,
    DL_PACKET_FREE2 = 0b10,
    DL_PACKET_TLM_LINK = 0b11,
};

typedef struct ElrsSyncPacket_s {
    uint16_t CRCCaesarCipher;
    uint8_t fhssIndex;
    uint8_t rxtx_counter;
#if RX_UPDATE_AIR_RATE
    uint8_t air_rate: 4;
    uint8_t tlm_interval : 4;
#else
    uint8_t tlm_interval;
#endif
    uint8_t pkt_type;
} ElrsSyncPacket_s;

#if (OTA_PACKET_DATA < 6)
#error "Min OTA size is 6 bytes!"
#endif

class RcChannels
{
public:
    RcChannels() {}

    uint8_t packetTypeGet(volatile uint8_t const *const input) {
        return input[OTA_PACKET_DATA-1] & 0b11;
    }
    void packetTypeSet(uint8_t *const output, uint8_t type) {
        uint8_t val = output[OTA_PACKET_DATA-1];
        val = (val & 0xFC) + (type & 0b11);
        output[OTA_PACKET_DATA-1] = val;
    }

    // TX related
    void processChannels(crsf_channels_t const *const channels);
    void ICACHE_RAM_ATTR get_packed_data(uint8_t *const output)
    {
        for (uint8_t i = 0; i < sizeof(packed_buffer); i++)
            output[i] = packed_buffer[i];
    }

    // RX related
    void ICACHE_RAM_ATTR channels_extract(uint8_t const *const input,
                                          crsf_channels_t &output);

    // TLM pkt
    uint8_t ICACHE_RAM_ATTR tlm_send(uint8_t *const output,
                                     mspPacket_t &packet,
                                     uint8_t tx=1);
    uint8_t ICACHE_RAM_ATTR tlm_receive(volatile uint8_t const *const input,
                                        mspPacket_t &packet);

private:
    // Pack channels into OTA TX buffer
    void channels_pack(void);
    // Switches / AUX channel handling
    uint8_t getNextSwitchIndex(void);

    // Channel processing data
    volatile uint16_t ChannelDataIn[N_CHANNELS] = {0};  // range: 0...2048
    volatile uint8_t currentSwitches[N_SWITCHES] = {0}; // range: 0,1,2

    // esp requires aligned buffer
    volatile uint8_t WORD_ALIGNED_ATTR packed_buffer[OTA_PACKET_SIZE];

    // bitmap of changed switches
    volatile uint16_t p_auxChannelsChanged = 0;
    // which switch should be sent in the next rc packet
    volatile uint8_t p_nextSwitchIndex = 0;
};

#endif /* __RC_CHANNELS_H */
