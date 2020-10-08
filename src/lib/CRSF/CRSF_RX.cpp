#include "CRSF_RX.h"
#include "utils.h"
#include "debug_elrs.h"
#include <string.h>

crsf_channels_msg_s DMA_ATTR p_crsf_channels;
crsf_msp_packet_fc_t DMA_ATTR msp_packet;

void CRSF_RX::Begin(void)
{
    LinkStatistics.header.device_addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    LinkStatistics.header.frame_size = sizeof(LinkStatistics) - CRSF_FRAME_START_BYTES;
    LinkStatistics.header.type = CRSF_FRAMETYPE_LINK_STATISTICS;

#if 0
    TLMbattSensor.header.device_addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    TLMbattSensor.header.frame_size = sizeof(TLMbattSensor) - CRSF_FRAME_START_BYTES;
    TLMbattSensor.header.type;

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

void CRSF_RX::LinkStatisticsSend(void) const
{
    sendFrameToFC((uint8_t*)&LinkStatistics, sizeof(LinkStatistics));
}

void ICACHE_RAM_ATTR CRSF_RX::sendRCFrameToFC(crsf_channels_t * channels) const
{
    memcpy(&p_crsf_channels.data, (void*)channels, sizeof(crsf_channels_t));
    sendFrameToFC((uint8_t*)&p_crsf_channels, sizeof(p_crsf_channels));
}

void ICACHE_RAM_ATTR CRSF_RX::sendMSPFrameToFC(uint8_t const *const packet, uint8_t len) const
{
    uint8_t i;

    if (!len || len > (sizeof(msp_packet.buffer) + 1))
        return;

    msp_packet.flags = packet[len-1];     // flags, last byte in packet buffer
    for (i = 0; i < (len - 1); i++) {
        msp_packet.buffer[i] = packet[i]; // payload
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
            TLMbattSensor.voltage = data[1];
            TLMbattSensor.voltage <<= 8;
            TLMbattSensor.voltage += data[2];

            TLMbattSensor.current = data[3];
            TLMbattSensor.current <<= 8;
            TLMbattSensor.current += data[4];

            TLMbattSensor.capacity = data[5];
            TLMbattSensor.capacity <<= 8;
            TLMbattSensor.capacity += data[6];
            TLMbattSensor.capacity <<= 8;
            TLMbattSensor.capacity += data[7];

            TLMbattSensor.remaining = data[8];
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

            TLMGPSsensor.valid = 3; // Slipt into 3 tlm pkts
            break;
        }

        case CRSF_FRAMETYPE_MSP_RESP:
        {
            if (data[1] == CRSF_ADDRESS_RADIO_TRANSMITTER &&
                data[2] == CRSF_ADDRESS_FLIGHT_CONTROLLER)
            {
                MspCallback(&data[3]); // pointer to MSP packet
            }
            break;
        }

        default:
            break;
    }
}

void CRSF_RX::handleUartIn(volatile uint8_t &rx_data_rcvd)
{
    uint8_t split_cnt = 0;
    for (split_cnt = 0; (rx_data_rcvd == 0) && _dev->available() && (split_cnt < 16); split_cnt++)
    {
        uint8_t *ptr = ParseInByte(_dev->read());
        if (ptr)
            processPacket(ptr);
    }
}
