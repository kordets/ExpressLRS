#include "CRSF_RX.h"
#include "utils.h"
#include "debug_elrs.h"
#include <string.h>

crsf_channels_msg_s DMA_ATTR p_crsf_channels;
crsf_msp_packet_fc_t DMA_ATTR msp_packet;
crsfLinkStatisticsMsg_t DMA_ATTR link_stat_packet;

void CRSF_RX::Begin(void)
{
    link_stat_packet.header.device_addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    link_stat_packet.header.frame_size = sizeof(link_stat_packet) - CRSF_FRAME_START_BYTES;
    link_stat_packet.header.type = CRSF_FRAMETYPE_LINK_STATISTICS;

#if 0
    TLMGPSsensor.header.device_addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    TLMGPSsensor.header.frame_size = sizeof(TLMGPSsensor) - CRSF_FRAME_START_BYTES;
    TLMGPSsensor.header.type;
#endif

    msp_packet.header.device_addr = CRSF_ADDRESS_BROADCAST;
    msp_packet.header.frame_size = sizeof(msp_packet) - CRSF_FRAME_START_BYTES;
    msp_packet.header.type = CRSF_FRAMETYPE_MSP_WRITE;
    msp_packet.header.dest_addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    msp_packet.header.orig_addr = CRSF_ADDRESS_RADIO_TRANSMITTER;

    p_crsf_channels.header.device_addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    p_crsf_channels.header.frame_size = sizeof(p_crsf_channels) - CRSF_FRAME_START_BYTES;
    p_crsf_channels.header.type = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;

    CRSF::Begin();
}

void ICACHE_RAM_ATTR CRSF_RX::sendFrameToFC(uint8_t *buff, uint8_t size) const
{
    buff[size - 1] = CalcCRC(&buff[2], (buff[1] - 1));
#if !NO_DATA_TO_FC
    uint32_t irq = _SAVE_IRQ();
    _dev->write(buff, size);
    _RESTORE_IRQ(irq);
#endif
}

void CRSF_RX::LinkStatisticsSend(LinkStats_t & stats) const
{
    memcpy(&link_stat_packet.stats, &stats.link, sizeof(link_stat_packet.stats));
    sendFrameToFC((uint8_t*)&link_stat_packet, sizeof(link_stat_packet));
}

void ICACHE_RAM_ATTR CRSF_RX::sendRCFrameToFC(rc_channels_t * channels) const
{
    memcpy(&p_crsf_channels.data, (void*)channels, sizeof(crsf_channels_t));
    sendFrameToFC((uint8_t*)&p_crsf_channels, sizeof(p_crsf_channels));
}

void ICACHE_RAM_ATTR CRSF_RX::sendMSPFrameToFC(mspPacket_t & msp) const
{
    uint8_t i;
    msp_packet.msp.flags = MSP_VERSION + msp.sequence_nbr;
    if (msp.payloadIterator == 0 && msp.sequence_nbr == 0) {
        msp_packet.msp.flags |= MSP_STARTFLAG;
    }
    msp.sequence_nbr++;
    for (i = 0; i < sizeof(msp_packet.msp.payload); i++) {
        msp_packet.msp.payload[i] = msp.readByte();
    }
    sendFrameToFC((uint8_t*)&msp_packet, sizeof(msp_packet));
}

void CRSF_RX::processPacket(uint8_t const *data)
{
    switch (data[0])
    {
        case CRSF_FRAMETYPE_COMMAND:
        {
            if (data[1] == 0x62 && data[2] == 0x6c)
            {
                DEBUG_PRINTF("Jumping to Bootloader...\n");
                delay(200);
                platform_restart();
            }
            break;
        }

        case CRSF_FRAMETYPE_BATTERY_SENSOR:
        {
            uint32_t voltage, current, capacity;
            voltage = data[1];
            voltage <<= 8;
            voltage += data[2];

            current = data[3];
            current <<= 8;
            current += data[4];

            capacity = data[5];
            capacity <<= 8;
            capacity += data[6];
            capacity <<= 8;
            capacity += data[7];
            //remaining = data[8];

            if (BattInfoCallback)
                BattInfoCallback(voltage, current, capacity);
            break;
        }

        case CRSF_FRAMETYPE_GPS:
        {
            TLMGPSsensor.latitude = data[1];
            TLMGPSsensor.latitude <<= 8;
            TLMGPSsensor.latitude += data[2];
            TLMGPSsensor.latitude <<= 8;
            TLMGPSsensor.latitude += data[3];
            TLMGPSsensor.latitude <<= 8;
            TLMGPSsensor.latitude += data[4];

            TLMGPSsensor.longitude = data[5];
            TLMGPSsensor.longitude <<= 8;
            TLMGPSsensor.longitude += data[6];
            TLMGPSsensor.longitude <<= 8;
            TLMGPSsensor.longitude += data[7];
            TLMGPSsensor.longitude <<= 8;
            TLMGPSsensor.longitude += data[8];

            TLMGPSsensor.speed = data[9];
            TLMGPSsensor.speed <<= 8;
            TLMGPSsensor.speed += data[10];

            TLMGPSsensor.heading = data[11];
            TLMGPSsensor.heading <<= 8;
            TLMGPSsensor.heading += data[12];

            TLMGPSsensor.altitude = data[13];
            TLMGPSsensor.altitude <<= 8;
            TLMGPSsensor.altitude += data[14];

            TLMGPSsensor.satellites = data[15];

            tlm_gps_valid = 3; // Slipt into 3 tlm pkts
            break;
        }

        case CRSF_FRAMETYPE_MSP_RESP:
        {
            if (data[1] == CRSF_ADDRESS_RADIO_TRANSMITTER &&
                data[2] == CRSF_ADDRESS_FLIGHT_CONTROLLER)
            {
                if (MspCallback)
                    MspCallback(&data[3]); // pointer to MSP packet
            }
            break;
        }

        default:
            break;
    }
}

void CRSF_RX::handleUartIn(void)
{
    int available = _dev->available();
    uint8_t *ptr;

    if (16 < available) available = 16;
    else if (available < 0) available = 0;

    while (available--) {
        ptr = ParseInByte(_dev->read());
        if (ptr)
            processPacket(ptr);
    }
}
