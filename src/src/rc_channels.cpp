#include "rc_channels.h"
#include "common.h"
#include "helpers.h"
#include "FHSS.h"
#include "debug_elrs.h"

#if (N_SWITCHES > (N_CHANNELS - N_CONTROLS))
#error "CRSF Channels Config is not OK"
#endif

#define USE_3b_SWITCH 0

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
    unsigned aux_n_idx : 3;
    unsigned aux_n : 3;
} PACKED RcDataPacket_s;

/**
 * Sequential switches packet
 * Replaces Generate4ChannelData_11bit
 * Channel 3 is reduced to 10 bits to allow a 3 bit switch index and 2 bit value
 * We cycle through 8 switches on successive packets. If any switches have changed
 * we take the lowest indexed one and send that, hence lower indexed switches have
 * higher priority in the event that several are changed at once.
 */
void RcChannels::channels_pack()
{
    // find the next switch to send
    uint8_t ch_idx = getNextSwitchIndex() & 0b111;

    RcDataPacket_s *rcdata = (RcDataPacket_s *)&packed_buffer[0];
    // The analog channels, scale down to 10bits
    rcdata->rc1 = (ChannelDataIn[0] >> 1);
    rcdata->rc2 = (ChannelDataIn[1] >> 1);
    rcdata->rc3 = (ChannelDataIn[2] >> 1);
    rcdata->rc4 = (ChannelDataIn[3] >> 1);
    // The round-robin switch
    rcdata->aux_n_idx = ch_idx;
    rcdata->aux_n = currentSwitches[ch_idx] & 0b111;
    // Set type
    rcdata->pkt_type = UL_PACKET_RC_DATA;
}

/**
 * Seq switches uses 10 bits for ch3, 3 bits for the switch index and 2 bits for the switch value
 */
void ICACHE_RAM_ATTR RcChannels::channels_extract(uint8_t const *const input,
                                                  crsf_channels_t &PackedRCdataOut)
{
    uint16_t switchValue;
    uint8_t switchIndex;

    RcDataPacket_s *rcdata = (RcDataPacket_s *)&input[0];
    // The analog channels
    PackedRCdataOut.ch0 = ((uint16_t)rcdata->rc1 << 1);
    PackedRCdataOut.ch1 = ((uint16_t)rcdata->rc2 << 1);
    PackedRCdataOut.ch2 = ((uint16_t)rcdata->rc3 << 1);
    PackedRCdataOut.ch3 = ((uint16_t)rcdata->rc4 << 1);
    // The round-robin switch
    switchIndex = rcdata->aux_n_idx;
#if USE_3b_SWITCH
    switchValue = SWITCH3b_to_CRSF(rcdata->aux_n);
#else
    switchValue = SWITCH2b_to_CRSF(rcdata->aux_n);
#endif

    switch (switchIndex)
    {
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
    }
}

void RcChannels::processChannels(crsf_channels_t const *const rcChannels)
{
    uint8_t switch_state;

    // TODO: loop N_CHANNELS times
    ChannelDataIn[0] = (rcChannels->ch0);
    ChannelDataIn[1] = (rcChannels->ch1);
    ChannelDataIn[2] = (rcChannels->ch2);
    ChannelDataIn[3] = (rcChannels->ch3);
    ChannelDataIn[4] = (rcChannels->ch4);
    ChannelDataIn[5] = (rcChannels->ch5);
    ChannelDataIn[6] = (rcChannels->ch6);
    ChannelDataIn[7] = (rcChannels->ch7);
    ChannelDataIn[8] = (rcChannels->ch8);
    ChannelDataIn[9] = (rcChannels->ch9);
    ChannelDataIn[10] = (rcChannels->ch10);
    ChannelDataIn[11] = (rcChannels->ch11);
#if 12 < N_CHANNELS
    ChannelDataIn[12] = (rcChannels->ch12);
#if 13 < N_CHANNELS
    ChannelDataIn[13] = (rcChannels->ch13);
#if 14 < N_CHANNELS
    ChannelDataIn[14] = (rcChannels->ch14);
#if 15 < N_CHANNELS
    ChannelDataIn[15] = (rcChannels->ch15);
#endif
#endif
#endif
#endif

    /**
     * Convert the rc data corresponding to switches to 2 bit values.
     *
     * I'm defining channels 4 through 11 inclusive as representing switches
     * Take the input values and convert them to the range 0 - 2.
     * (not 0-3 because most people use 3 way switches and expect the middle
     *  position to be represented by a middle numeric value)
     */
    for (uint8_t idx = 0; idx < N_SWITCHES; idx++)
    {
        // input is 0 - 2048, output is 0 - 2
#if USE_3b_SWITCH
        switch_state = CRSF_to_SWITCH3b(ChannelDataIn[idx + N_CONTROLS]) & 0b111;
#else
        switch_state = CRSF_to_SWITCH2b(ChannelDataIn[idx + N_CONTROLS]) & 0b11;
#endif
        // Check if state is changed
        if (switch_state != currentSwitches[idx])
            p_auxChannelsChanged |= (0x1 << idx);
        currentSwitches[idx] = switch_state;
    }

    channels_pack();
}

/**
 * Determine which switch to send next.
 * If any switch has changed since last sent, we send the lowest index changed switch
 * and set nextSwitchIndex to that value + 1.
 * If no switches have changed then we send nextSwitchIndex and increment the value.
 * For pure sequential switches, all 8 switches are part of the round-robin sequence.
 * For hybrid switches, switch 0 is sent with every packet and the rest of the switches
 * are in the round-robin.
 */
uint8_t RcChannels::getNextSwitchIndex()
{
    int8_t index;
#ifdef HYBRID_SWITCHES_8
    // for hydrid switches 0 is sent on every packet, so ignore it
    p_auxChannelsChanged &= 0xfffe;
#endif
    /* Check if channel is changed and send it immediately,
     *  send next sequential switch if not changed */
    index = __builtin_ffs(p_auxChannelsChanged) - 1;
    if (index < 0)
    {
        index = p_nextSwitchIndex++;
        p_nextSwitchIndex %= N_SWITCHES;
    }
    else
    {
        p_auxChannelsChanged &= ~(0x1 << index);
    }

#ifdef HYBRID_SWITCHES_8
    // for hydrid switches 0 is sent on every packet, so we can skip
    // that value for the round-robin
    if (p_nextSwitchIndex == 0)
        p_nextSwitchIndex++;
#endif

    return index;
}

typedef union {
    struct
    {
        uint16_t func;
        uint16_t payloadSize;
        uint8_t flags;
    } hdr;
    struct
    {
        uint8_t data[5];
    } payload;
    uint8_t pkt_type;
} PACKED TlmDataPacket_s;

uint8_t ICACHE_RAM_ATTR RcChannels::tlm_send(uint8_t *const output,
                                             mspPacket_t &packet,
                                             uint8_t tx)
{
    TlmDataPacket_s *tlm_ptr = (TlmDataPacket_s *)output;

    /* Ignore invalid packets */
    if (packet.type != MSP_PACKET_TLM_OTA)
        return 0;

    tlm_ptr->pkt_type = tx ? (uint8_t)UL_PACKET_MSP : (uint8_t)DL_PACKET_TLM_MSP;

    if (!packet.header_sent_or_rcvd)
    {
        /* Send header and first byte */
        tlm_ptr->hdr.flags = packet.flags;
        tlm_ptr->hdr.func = packet.function;
        tlm_ptr->hdr.payloadSize = packet.payloadSize;
        packet.header_sent_or_rcvd = true;
    }
    else
    {
        for (uint8_t iter = 0; iter < sizeof(tlm_ptr->payload.data); iter++)
            tlm_ptr->payload.data[iter] = packet.readByte();
    }
    return packet.iterated();
}

uint8_t ICACHE_RAM_ATTR RcChannels::tlm_receive(volatile uint8_t const *const input,
                                                mspPacket_t &packet)
{
    if (packet.iterated())
        return 1;

    TlmDataPacket_s *tlm_ptr = (TlmDataPacket_s *)input;
    if (packet.header_sent_or_rcvd && packet.type == MSP_PACKET_TLM_OTA)
    {
        for (uint8_t iter = 0; iter < sizeof(tlm_ptr->payload.data); iter++)
            packet.addByte(tlm_ptr->payload.data[iter]);
    }
    else if (packet.type == MSP_PACKET_UNKNOWN && !packet.header_sent_or_rcvd)
    {
        // buffer free, fill header
        packet.type = MSP_PACKET_TLM_OTA;
        packet.flags = tlm_ptr->hdr.flags;
        packet.function = tlm_ptr->hdr.func;
        packet.payloadSize = tlm_ptr->hdr.payloadSize;
        packet.header_sent_or_rcvd = true;
    }
    else
    {
        packet.error = true;
        return 0;
    }
    return packet.iterated();
}
