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

void ICACHE_RAM_ATTR CRSF_RX::sendMSPFrameToFC(uint8_t const *const packet, uint8_t len)
{
#if 0
    uint8_t i;
    uint8_t msp_len = 2 + len; // dest, orig + len
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
#else
    uint8_t msp_len = 8 + 2;
    uint8_t frame_len = CRSF_EXT_FRAME_SIZE(msp_len); // total len = msp_len + address & crc
    uint16_t freq = 5840;
    //uint16_t freq = (3 << 3) + 5;

    // CRSF frame header
    outBuffer[0] = CRSF_ADDRESS_BROADCAST;         // address
    outBuffer[1] = CRSF_FRAME_SIZE(msp_len);       // CRSF frame len
    outBuffer[2] = CRSF_FRAMETYPE_MSP_WRITE;       // packet type
    // Encapsulated MSP
    outBuffer[3] = CRSF_ADDRESS_FLIGHT_CONTROLLER; // destination
    outBuffer[4] = CRSF_ADDRESS_RADIO_TRANSMITTER; // origin
    outBuffer[5] = 0x30;                           // flags, last byte in packet buffer
    outBuffer[6] = 4; // len
    outBuffer[7] = 89; // write
    outBuffer[8] = freq & 0xff;
    outBuffer[9] = (freq >> 8) & 0xff;
    outBuffer[10] = 1; // pwr
    outBuffer[11] = 0; // pitmode
    outBuffer[12 /*frame_len-2*/] = CalcCRCxor(&outBuffer[6], 6);
#endif

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
