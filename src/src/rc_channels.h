#ifndef __RC_CHANNELS_H
#define __RC_CHANNELS_H

#include "platform.h"
#include "CRSF.h" // crsf_channels_t
#include "msp.h"
#include <stdint.h>

#ifndef OTA_PACKET_10B
#define OTA_PACKET_10B      0
#endif

#define OTA_PACKET_DATA     6
#define OTA_PACKET_CRC      2
#define OTA_PACKET_PAYLOAD  (OTA_PACKET_DATA + (OTA_PACKET_10B * 2))
#define OTA_PACKET_SIZE     (OTA_PACKET_PAYLOAD + OTA_PACKET_CRC)

// current and sent switch values
#define N_CONTROLS 4
#ifndef N_SWITCHES
//#define N_SWITCHES 8
#define N_SWITCHES 5
#endif
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
    DL_PACKET_GPS = 0b10,
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

#if SERVO_OUTPUTS_ENABLED
#define EXTRACT_VOLATILE volatile
#else
#define EXTRACT_VOLATILE
#endif


inline __attribute__((always_inline)) uint8_t
RcChannels_packetTypeGet(volatile uint8_t const *const input)
{
    return input[OTA_PACKET_DATA-1] & 0b11;
}

inline __attribute__((always_inline)) void
RcChannels_packetTypeSet(uint8_t *const output, uint8_t type)
{
    uint8_t val = output[OTA_PACKET_DATA-1];
    val = (val & 0xFC) + (type & 0b11);
    output[OTA_PACKET_DATA-1] = val;
}


// TX related
void
RcChannels_processChannels(crsf_channels_t const *const channels);
void ICACHE_RAM_ATTR
RcChannels_get_packed_data(uint8_t *const output);

// RX related
void ICACHE_RAM_ATTR
RcChannels_channels_extract(uint8_t const *const input,
                            EXTRACT_VOLATILE crsf_channels_t &output);

// TLM pkt
uint8_t ICACHE_RAM_ATTR
RcChannels_tlm_ota_send(uint8_t *const output,
                        mspPacket_t &packet,
                        uint8_t tx=1);
uint8_t ICACHE_RAM_ATTR
RcChannels_tlm_uplink_receive(volatile uint8_t *const input);

uint8_t ICACHE_RAM_ATTR
RcChannels_tlm_downlink_receive(volatile uint8_t const *const input,
                                mspPacket_t &packet);

#endif /* __RC_CHANNELS_H */
