#include "CRSF_RX.h"
#include "utils.h"
#include "debug_elrs.h"
#include <string.h>

void ICACHE_RAM_ATTR CRSF_RX::sendFrameToFC(uint8_t *buff, uint8_t size)
{
    buff[size - 1] = CalcCRC(&buff[2], (buff[1] - 1));
#if !NO_DATA_TO_FC
    uint32_t irq = _SAVE_IRQ();
    _dev->write(buff, size);
    _RESTORE_IRQ(irq);
#endif
}

void CRSF_RX::LinkStatisticsSend()
{
    uint8_t out[CRSF_EXT_FRAME_SIZE(LinkStatisticsFrameLength)]; // 10 + 2 + 2 bytes

    out[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    out[1] = CRSF_FRAME_SIZE(LinkStatisticsFrameLength);
    out[2] = CRSF_FRAMETYPE_LINK_STATISTICS;

    memcpy(&out[3], (void *)&LinkStatistics, LinkStatisticsFrameLength);

    sendFrameToFC(out, sizeof(out));
}

void ICACHE_RAM_ATTR CRSF_RX::sendRCFrameToFC(crsf_channels_t * channels)
{
    uint8_t out_rc_data[CRSF_EXT_FRAME_SIZE(RCframeLength)];

    out_rc_data[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    out_rc_data[1] = CRSF_FRAME_SIZE(RCframeLength);
    out_rc_data[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;

    memcpy(&out_rc_data[3], channels, RCframeLength);

    sendFrameToFC(out_rc_data, sizeof(out_rc_data));
}

void ICACHE_RAM_ATTR CRSF_RX::sendMSPFrameToFC(mspPacket_t &packet)
{
    // TODO: This currently only supports single MSP packets per cmd
    // To support longer packets we need to re-write this to allow packet splitting
#if 0
    uint8_t i;
    const uint8_t len = ENCAPSULATED_MSP_FRAME_LEN + CRSF_FRAME_LENGTH_EXT_TYPE_CRC + CRSF_FRAME_NOT_COUNTED_BYTES; // 8 + 4 + 2
    if (len > CRSF_PAYLOAD_SIZE_MAX || !(packet.flags & MSP_VERSION))
        return;

    // CRSF extended frame header
    outBuffer[0] = CRSF_ADDRESS_BROADCAST;                                      // address
    outBuffer[1] = ENCAPSULATED_MSP_FRAME_LEN + CRSF_FRAME_LENGTH_EXT_TYPE_CRC; // length
    outBuffer[2] = CRSF_FRAMETYPE_MSP_WRITE;                                    // packet type
    outBuffer[3] = CRSF_ADDRESS_FLIGHT_CONTROLLER;                              // destination
    outBuffer[4] = CRSF_ADDRESS_RADIO_TRANSMITTER;                              // origin

    // Encapsulated MSP payload
    outBuffer[5] = packet.flags;       // header byte (bits: hhhhssss)
    outBuffer[6] = packet.payloadSize; // mspPayloadSize
    outBuffer[7] = packet.function;    // packet->cmd
    for (uint8_t i = 0; i < ENCAPSULATED_MSP_PAYLOAD_SIZE; ++i)
    {
        // copy packet payload into outBuffer and pad with zeros where required
        outBuffer[8 + i] = i < packet->payloadSize ? packet->payload[i] : 0;
    }
    // Encapsulated MSP crc
    outBuffer[len - 2] = CalcCRCxor(&outBuffer[6], ENCAPSULATED_MSP_FRAME_LEN - 2);
#else
    uint8_t i;
    uint8_t msp_len = CRSF_MSP_FRAME_SIZE(packet.payloadSize);
    uint8_t len = CRSF_EXT_FRAME_SIZE(msp_len); // total len
    if (len > CRSF_EXT_FRAME_SIZE(CRSF_PAYLOAD_SIZE_MAX))
        return;

    // CRSF extended frame header
    outBuffer[0] = CRSF_ADDRESS_BROADCAST;         // address
    outBuffer[1] = CRSF_FRAME_SIZE(msp_len);       // CRSF frame len
    outBuffer[2] = CRSF_FRAMETYPE_MSP_WRITE;       // packet type
    outBuffer[3] = CRSF_ADDRESS_FLIGHT_CONTROLLER; // destination
    outBuffer[4] = CRSF_ADDRESS_RADIO_TRANSMITTER; // origin

    // Encapsulated MSP payload
    outBuffer[5] = packet.flags;
    outBuffer[6] = packet.payloadSize - 1;
    outBuffer[7] = packet.function;
    for (i = 0; i < packet.payloadSize; i++)
    {
        // copy packet payload into outBuffer
        outBuffer[i+8] = packet.payload[i];
    }
    // Encapsulated MSP crc - bypass CRC to received to protect possible transfer failures
    //outBuffer[i] = CalcCRCxor(&outBuffer[6], (packet.payloadSize + 1)); // was out[12]
#endif
    sendFrameToFC(outBuffer, len);
}

void ICACHE_RAM_ATTR CRSF_RX::sendMSPFrameToFC(uint8_t const *const packet, uint8_t len)
{
    uint8_t i;
    uint8_t msp_len = CRSF_FRAME_NOT_COUNTED_BYTES + len; // len + dest + orig
    uint8_t frame_len = CRSF_EXT_FRAME_SIZE(msp_len); // total len = msp_len + address & crc
    if (CRSF_EXT_FRAME_SIZE(CRSF_PAYLOAD_SIZE_MAX) < frame_len || !len)
        return;

    // CRSF frame header
    outBuffer[0] = CRSF_ADDRESS_BROADCAST;         // address
    outBuffer[1] = CRSF_FRAME_SIZE(msp_len);       // CRSF frame len
    outBuffer[2] = CRSF_FRAMETYPE_MSP_WRITE;       // packet type
    // Encapsulated MSP
    outBuffer[3] = CRSF_ADDRESS_FLIGHT_CONTROLLER; // destination
    outBuffer[4] = CRSF_ADDRESS_RADIO_TRANSMITTER; // origin
    outBuffer[5] = packet[len-1];                  // flags, last byte in packet buffer
    for (i = 0; i < (len - 1); i++) {
        outBuffer[i + 6] = packet[i];              // payload
    }

    sendFrameToFC(outBuffer, frame_len);
}

void CRSF_RX::processPacket(uint8_t const *data)
{
    switch (data[0])
    {
        case CRSF_FRAMETYPE_COMMAND:
        {
            if (data[1] == 0x62 && data[2] == 0x6c)
            {
                DEBUG_PRINTLN("Jumping to Bootloader...");
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

            TLMbattSensor.remaining = 0;
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
