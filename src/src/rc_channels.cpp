#include "rc_channels.h"
#include "common.h"
#include "helpers.h"
#include "FHSS.h"
#include "debug_elrs.h"

#include <assert.h>

/*************************************************************************************
 * RC OTA PACKET
 *************************************************************************************/

#if (16 < N_CHANNELS)
#error "CRSF Channels Config is not OK"
#endif

typedef struct
{
    // The analog channels
    unsigned rc1 : 10;
    unsigned rc2 : 10;
    unsigned rc3 : 10;
    unsigned rc4 : 10;
    // Packet type
    unsigned pkt_type : 2;
    // The round-robin switch
#if N_SWITCHES <= 8
    unsigned aux_n_idx : 3;
    unsigned aux_n : 3;
#else
    unsigned aux_n_idx : 4;
    unsigned aux_n : 2;
#endif
} PACKED RcDataPacket_s;

//#if (OTA_PACKET_DATA < sizeof(RcDataPacket_s))
//#error "Min OTA size is 6 bytes!"
//#endif

static_assert(sizeof(RcDataPacket_s) <= OTA_PACKET_DATA,
              "OTA pkt size is not correct");


// Holds last state of the switches
uint8_t DRAM_ATTR currentSwitches[N_SWITCHES] = {0};

// Packed OTA packet buffer
// NOTE! ESP32 requires aligned buffer
volatile uint8_t WORD_ALIGNED_ATTR DRAM_ATTR packed_buffer[OTA_PACKET_SIZE];

// bitmap of changed switches
uint16_t DRAM_ATTR p_auxChannelsChanged = 0;
// which switch should be sent in the next rc packet
uint8_t DRAM_ATTR p_nextSwitchIndex = 0;


/**
 * Determine which switch to send next.
 *
 * If any switch has changed since last sent, it sends the lowest index changed switch.
 * If no switches have changed then this sends p_nextSwitchIndex and increment the value.
 */
inline __attribute__((always_inline)) uint8_t
getNextSwitchIndex(void)
{
    /* Check if channel is changed and send it immediately,
     * otherwise send next sequential switch */
    int8_t index = __builtin_ffs(p_auxChannelsChanged) - 1;
    if (index < 0) {
        index = p_nextSwitchIndex++;
        p_nextSwitchIndex %= N_SWITCHES;
    } else {
        p_auxChannelsChanged &= ~(0x1 << index);
    }
    return index;
}

/**
 * Sequential switches packet
 *
 * Cycle through N_SWITCHES switches on successive packets. If any switches have
 * changed takes the lowest indexed one and send that, hence lower indexed switches have
 * higher priority in the event that several are changed at once.
 */
inline __attribute__((always_inline)) void
channels_pack(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4)
{
    // find the next switch to send
    uint8_t ch_idx = getNextSwitchIndex() & 0b111;

    RcDataPacket_s *rcdata = (RcDataPacket_s *)&packed_buffer[0];
    // The analog channels, scale down to 10bits
    rcdata->rc1 = (ch1 >> 1);
    rcdata->rc2 = (ch2 >> 1);
    rcdata->rc3 = (ch3 >> 1);
    rcdata->rc4 = (ch4 >> 1);
    // The round-robin switch
    rcdata->aux_n_idx = ch_idx;
    rcdata->aux_n = currentSwitches[ch_idx];
    // Set type
    rcdata->pkt_type = UL_PACKET_RC_DATA;
}

/**
 * Convert received OTA packet data to CRSF packet for FC
 */
void ICACHE_RAM_ATTR RcChannels_channels_extract(uint8_t const *const input,
                                                  EXTRACT_VOLATILE crsf_channels_t &PackedRCdataOut)
{
    uint16_t switchValue;
    uint8_t switchIndex;

    RcDataPacket_s *rcdata = (RcDataPacket_s *)&input[0];
    // The analog channels, scale packet to 11bit
    PackedRCdataOut.ch0 = ((uint16_t)rcdata->rc1 << 1);
    PackedRCdataOut.ch1 = ((uint16_t)rcdata->rc2 << 1);
    PackedRCdataOut.ch2 = ((uint16_t)rcdata->rc3 << 1);
    PackedRCdataOut.ch3 = ((uint16_t)rcdata->rc4 << 1);
    // The round-robin switch
    switchIndex = rcdata->aux_n_idx;
#if N_SWITCHES <= 8
    switchValue = SWITCH3b_to_CRSF(rcdata->aux_n);
#else
    switchValue = SWITCH2b_to_CRSF(rcdata->aux_n);
#endif

    switch (switchIndex) {
        case 0:
            PackedRCdataOut.ch4 = switchValue;
            break;
        case 1:
            PackedRCdataOut.ch5 = switchValue;
            break;
        case 2:
            PackedRCdataOut.ch6 = switchValue;
            break;
        case 3:
            PackedRCdataOut.ch7 = switchValue;
            break;
        case 4:
            PackedRCdataOut.ch8 = switchValue;
            break;
        case 5:
            PackedRCdataOut.ch9 = switchValue;
            break;
        case 6:
            PackedRCdataOut.ch10 = switchValue;
            break;
        case 7:
            PackedRCdataOut.ch11 = switchValue;
            break;
#if 8 < N_SWITCHES
        case 8:
            PackedRCdataOut.ch12 = switchValue;
            break;
#if 9 < N_SWITCHES
        case 9:
            PackedRCdataOut.ch13 = switchValue;
            break;
#if 10 < N_SWITCHES
        case 10:
            PackedRCdataOut.ch14 = switchValue;
            break;
#if 11 < N_SWITCHES
        case 11:
            PackedRCdataOut.ch15 = switchValue;
            break;
#endif // 11 < N_SWITCHES
#endif // 10 < N_SWITCHES
#endif // 9 < N_SWITCHES
#endif // 8 < N_SWITCHES
    }
}

/**
 * Convert received CRSF serial packet from handset and
 * store it to local buffers for OTA packet
 */
void RcChannels_processChannels(crsf_channels_t const *const rcChannels)
{
    // channels input range: 0...2048
    uint16_t ChannelDataIn[N_SWITCHES] = {
        (uint16_t)rcChannels->ch4, (uint16_t)rcChannels->ch5,
        (uint16_t)rcChannels->ch6, (uint16_t)rcChannels->ch7,
        (uint16_t)rcChannels->ch8, (uint16_t)rcChannels->ch9,
        (uint16_t)rcChannels->ch10, (uint16_t)rcChannels->ch11
#if 8 < N_SWITCHES
    , (uint16_t)rcChannels->ch12
#if 9 < N_SWITCHES
    , (uint16_t)rcChannels->ch13
#if 10 < N_SWITCHES
    , (uint16_t)rcChannels->ch14
#if 11 < N_SWITCHES
    , (uint16_t)rcChannels->ch15
#endif // 11 < N_SWITCHES
#endif // 10 < N_SWITCHES
#endif // 9 < N_SWITCHES
#endif // 8 < N_SWITCHES
    };
    uint8_t switch_state;

    /**
     * Convert the rc data corresponding to switches to 2 bit values.
     *
     * I'm defining channels 4 through 11 inclusive as representing switches
     * Take the input values and convert them to the range 0 - 2.
     * (not 0-3 because most people use 3 way switches and expect the middle
     *  position to be represented by a middle numeric value)
     */
    for (uint8_t idx = 0; idx < N_SWITCHES; idx++) {
#if N_SWITCHES <= 8
        // input is 0 - 2048, output is 0 - 7
        switch_state = CRSF_to_SWITCH3b(ChannelDataIn[idx]) & 0b111;
#else
        // input is 0 - 2048, output is 0 - 2
        switch_state = CRSF_to_SWITCH2b(ChannelDataIn[idx]) & 0b11;
#endif
        // Check if state is changed
        if (switch_state != currentSwitches[idx])
            p_auxChannelsChanged |= (0x1 << idx);
        currentSwitches[idx] = switch_state;
    }

    channels_pack(rcChannels->ch0, rcChannels->ch1, rcChannels->ch2, rcChannels->ch3);
}

void ICACHE_RAM_ATTR RcChannels_get_packed_data(uint8_t *const output)
{
    uint8_t iter = sizeof(RcDataPacket_s);
    while (iter--)
        output[iter] = packed_buffer[iter];
}

/*************************************************************************************
 * TELEMETRY OTA PACKET
 *************************************************************************************/

typedef struct {
    union {
        struct
        {
            uint16_t func;
            uint16_t payloadSize;
            uint8_t flags;
        } PACKED hdr;
        struct
        {
            uint8_t data[5];
        } PACKED payload;
    };
    union {
        uint8_t pkt_type : 2, seq : 4, ver : 2;
        uint8_t flags;
    };
} PACKED TlmDataPacket_s;

static_assert(sizeof(TlmDataPacket_s) <= OTA_PACKET_DATA,
              "OTA pkt size is not correct");

uint8_t ICACHE_RAM_ATTR
RcChannels_tlm_ota_send(uint8_t *const output,
                        mspPacket_t &packet,
                        uint8_t tx)
{
    TlmDataPacket_s *tlm_ptr = (TlmDataPacket_s *)output;
    uint8_t iter = 0;
    tlm_ptr->pkt_type = tx ? (uint8_t)UL_PACKET_MSP : (uint8_t)DL_PACKET_TLM_MSP;
    tlm_ptr->seq = packet.sequence_nbr++;
    tlm_ptr->ver = (MSP_VERSION >> 4);

    if (!tlm_ptr->seq) {
        /* Start MSP packet */
        tlm_ptr->ver |= (MSP_STARTFLAG >> 4);
    }

    for (iter = 0; iter < sizeof(tlm_ptr->payload.data); iter++)
        tlm_ptr->payload.data[iter] = packet.readByte();

    return packet.iterated();
}

uint8_t ICACHE_RAM_ATTR
RcChannels_tlm_uplink_receive(volatile uint8_t *const input)
{
    TlmDataPacket_s *tlm_ptr = (TlmDataPacket_s *)input;
    tlm_ptr->flags >>= 2; // remove pkt_type
    return (tlm_ptr->flags & MSP_VERSION) ? sizeof(TlmDataPacket_s) : 0; // return data len
}

uint8_t ICACHE_RAM_ATTR
RcChannels_tlm_downlink_receive(volatile uint8_t const *const input,
                                mspPacket_t &packet)
{
    if (packet.iterated())
        return 1;

    TlmDataPacket_s *tlm_ptr = (TlmDataPacket_s *)input;
    tlm_ptr->flags >>= 2; // remove pkt_type
    if (tlm_ptr->flags & MSP_VERSION) {
        if (tlm_ptr->flags & MSP_STARTFLAG) {
            // first junk, reset packet and start reception
            packet.reset();
            packet.type = MSP_PACKET_TLM_OTA;
            packet.payloadSize = input[0];
            packet.function = input[1];
        } else if ((tlm_ptr->flags & 0xf) == packet.sequence_nbr) {
            // next junk...
        } else {
            // error...
        }

        packet.sequence_nbr++;

        for (uint8_t iter = 0; iter < sizeof(tlm_ptr->payload.data); iter++)
            packet.addByte(tlm_ptr->payload.data[iter]);
    }

    return packet.iterated();
}
