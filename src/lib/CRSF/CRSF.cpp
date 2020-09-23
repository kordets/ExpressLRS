#include "CRSF.h"
#include "debug_elrs.h"

void rcNullCb(crsf_channels_t const *const) {}
void (*CRSF::RCdataCallback1)(crsf_channels_t const *const) = &rcNullCb; // function is called whenever there is new RC data.

void MspNullCallback(uint8_t const *const){};
void (*CRSF::MspCallback)(uint8_t const *const input) = MspNullCallback;

void nullCallback(void){};
void (*CRSF::disconnected)() = &nullCallback; // called when CRSF stream is lost
void (*CRSF::connected)() = &nullCallback;    // called when CRSF stream is regained

uint8_t DMA_ATTR SerialInBuffer[CRSF_EXT_FRAME_SIZE(CRSF_PAYLOAD_SIZE_MAX)];

//#define DBF_PIN_CRSF_PACKET 2

void CRSF::Begin()
{
#ifdef DBF_PIN_CRSF_PACKET
    pinMode(DBF_PIN_CRSF_PACKET, OUTPUT);
    digitalWrite(DBF_PIN_CRSF_PACKET, 0);
#endif

    GoodPktsCount = 0;
    BadPktsCount = 0;
    SerialInPacketStart = 0;
    SerialInPacketLen = 0;
    SerialInPacketPtr = 0;
    CRSFframeActive = false;

    TLMbattSensor.capacity = 0;
    TLMbattSensor.current = 0;
    TLMbattSensor.voltage = 0;
    //TLMbattSensor.remaining = 100;

    TLMGPSsensor.valid = 0;

    _dev->flush_read();
}

void CRSF::LinkStatisticsExtract(volatile uint8_t const *const input,
                                 int8_t snr,
                                 uint8_t rssi)
{
    // NOTE: input is only 5 bytes + 6bits (MSB)!!

    LinkStatistics.downlink_SNR = snr * 10;
    LinkStatistics.downlink_RSSI = 120 + rssi;

    if ((input[5] >> 2) == CRSF_FRAMETYPE_LINK_STATISTICS)
    {
        LinkStatistics.uplink_RSSI_1 = input[0];
        LinkStatistics.uplink_RSSI_2 = 0;
        LinkStatistics.uplink_SNR = input[2];
        LinkStatistics.uplink_Link_quality = input[3];

        TLMbattSensor.voltage = ((uint16_t)input[1] << 8) + input[4];
    }
}

void ICACHE_RAM_ATTR CRSF::LinkStatisticsPack(uint8_t *const output)
{
    // NOTE: output is only 5 bytes + 6bits (MSB)!!

    // OpenTX hard codes "rssi" warnings to the LQ sensor for crossfire, so the
    // rssi we send is for display only.
    // OpenTX treats the rssi values as signed.
    uint8_t openTxRSSI = LinkStatistics.uplink_RSSI_1;
    // truncate the range to fit into OpenTX's 8 bit signed value
    if (openTxRSSI > 127)
        openTxRSSI = 127;
    // convert to 8 bit signed value in the negative range (-128 to 0)
    openTxRSSI = 255 - openTxRSSI;
    output[0] = openTxRSSI;
    output[1] = (TLMbattSensor.voltage & 0xFF00) >> 8;
    output[2] = LinkStatistics.uplink_SNR;
    output[3] = LinkStatistics.uplink_Link_quality;
    output[4] = (TLMbattSensor.voltage & 0x00FF);
    output[5] = CRSF_FRAMETYPE_LINK_STATISTICS << 2;
}

void ICACHE_RAM_ATTR CRSF::GpsStatsExtract(volatile uint8_t const *const input)
{
    uint8_t type = input[5] >> 2;
    if ((type & 0xf) != CRSF_FRAMETYPE_GPS)
        return;
    switch (type >> 4) {
        case 3:
            TLMGPSsensor.latitude = input[0];
            TLMGPSsensor.latitude <<= 8;
            TLMGPSsensor.latitude += input[1];
            TLMGPSsensor.latitude <<= 8;
            TLMGPSsensor.latitude += input[2];
            TLMGPSsensor.latitude <<= 8;
            TLMGPSsensor.latitude += input[3];
            TLMGPSsensor.speed = input[4];
            TLMGPSsensor.speed <<= 8;
            break;
        case 2:
            TLMGPSsensor.longitude = input[0];
            TLMGPSsensor.longitude <<= 8;
            TLMGPSsensor.longitude += input[1];
            TLMGPSsensor.longitude <<= 8;
            TLMGPSsensor.longitude += input[2];
            TLMGPSsensor.longitude <<= 8;
            TLMGPSsensor.longitude += input[3];
            TLMGPSsensor.speed += input[4];
            break;
        case 1:
            TLMGPSsensor.heading = input[0];
            TLMGPSsensor.heading <<= 8;
            TLMGPSsensor.heading += input[1];
            TLMGPSsensor.altitude = input[2];
            TLMGPSsensor.altitude <<= 8;
            TLMGPSsensor.altitude += input[3];
            TLMGPSsensor.satellites = input[4];
            TLMGPSsensor.valid = true;
            break;
    }
}

uint8_t ICACHE_RAM_ATTR CRSF::GpsStatsPack(uint8_t *const output)
{
    uint8_t type = (TLMGPSsensor.valid << 4) + CRSF_FRAMETYPE_GPS;
    if (!TLMGPSsensor.valid)
        return 0;
    // GPS block is split into pieces
    switch (TLMGPSsensor.valid--) {
        case 3:
            output[0] = (uint8_t)(TLMGPSsensor.latitude >> 24);
            output[1] = (uint8_t)(TLMGPSsensor.latitude >> 16);
            output[2] = (uint8_t)(TLMGPSsensor.latitude >> 8);
            output[3] = (uint8_t)TLMGPSsensor.latitude;
            output[4] = (uint8_t)(TLMGPSsensor.speed >> 8);
            break;
        case 2:
            output[0] = (uint8_t)(TLMGPSsensor.longitude >> 24);
            output[1] = (uint8_t)(TLMGPSsensor.longitude >> 16);
            output[2] = (uint8_t)(TLMGPSsensor.longitude >> 8);
            output[3] = (uint8_t)TLMGPSsensor.longitude;
            output[4] = (uint8_t)(TLMGPSsensor.speed);
            break;
        case 1:
            output[0] = (uint8_t)(TLMGPSsensor.heading >> 8);
            output[1] = (uint8_t)(TLMGPSsensor.heading);
            output[2] = (uint8_t)(TLMGPSsensor.altitude >> 8);
            output[3] = (uint8_t)(TLMGPSsensor.altitude);
            output[4] = TLMGPSsensor.satellites;
            break;
    }
    output[5] = type << 2;
    return 1;
}

uint8_t *CRSF::ParseInByte(uint8_t inChar)
{
    uint8_t *packet_ptr = NULL;

    if (SerialInPacketPtr >= sizeof(SerialInBuffer))
    {
        // we reached the maximum allowable packet length,
        // so start again because shit fucked up hey.
        SerialInPacketPtr = 0;
        CRSFframeActive = false;
        BadPktsCount++;
    }

    // store byte
    SerialInBuffer[SerialInPacketPtr++] = inChar;

    // CRSF Frame:
    // | address | payload_len | payload* | crc |

    if (CRSFframeActive == false)
    {
        if (inChar == CRSF_ADDRESS_CRSF_RECEIVER ||
            inChar == CRSF_ADDRESS_CRSF_TRANSMITTER ||
            inChar == CRSF_SYNC_BYTE)
        {
            CRSFframeActive = true;
            SerialInPacketLen = 0;
#ifdef DBF_PIN_CRSF_PACKET
            digitalWrite(DBF_PIN_CRSF_PACKET, 1);
#endif
        }
        else
        {
            SerialInPacketPtr = 0;
        }
    }
    else
    {
        if (SerialInPacketLen == 0) // we read the packet length and save it
        {
            SerialInCrc = 0;
            SerialInPacketLen = inChar;
            SerialInPacketStart = SerialInPacketPtr;
            if ((SerialInPacketLen < 2) || (CRSF_FRAME_SIZE_MAX < SerialInPacketLen))
            {
                // failure -> discard
                CRSFframeActive = false;
                SerialInPacketPtr = 0;
                BadPktsCount++;
            }
        }
        else
        {
            if ((SerialInPacketPtr - SerialInPacketStart) >= (SerialInPacketLen))
            {
                /* Check packet CRC */
                if (SerialInCrc == inChar)
                {
                    packet_ptr = &SerialInBuffer[SerialInPacketStart];
                    GoodPktsCount++;
                }
                else
                {
#if 0
                    // https://crccalc.com/
                    // CRC algorithm: CRC-8/DVB-S2
                    DEBUG_PRINT("UART CRC failure ");
                    DEBUG_PRINT(SerialInCrc, HEX);
                    DEBUG_PRINT("!=");
                    DEBUG_PRINT(inChar, HEX);
                    for (int i = (SerialInPacketStart-2); i < (SerialInPacketLen + 2); i++)
                    {
                        DEBUG_PRINT(" 0x");
                        DEBUG_PRINT(SerialInBuffer[i], HEX);
                    }
                    DEBUG_PRINTLN();
#elif defined(DEBUG_SERIAL)
                    DEBUG_SERIAL.print("!C");
                    //DEBUG_SERIAL.write((uint8_t *)&SerialInBuffer[SerialInPacketStart - 2], SerialInPacketLen + 2);
#endif
                    BadPktsCount++;
                }

#ifdef DBF_PIN_CRSF_PACKET
                digitalWrite(DBF_PIN_CRSF_PACKET, 0);
#endif

                // packet handled, start next
                CRSFframeActive = false;
                SerialInPacketPtr = 0;
                SerialInPacketLen = 0;
            }
            else
            {
                // Calc crc on the fly
                SerialInCrc = CalcCRC(inChar, SerialInCrc);
            }
        }
    }

    return packet_ptr;
}
