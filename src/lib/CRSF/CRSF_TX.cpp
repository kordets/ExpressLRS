#include "CRSF_TX.h"
#include "debug_elrs.h"
#include <string.h>

//#define DBF_PIN_CRSF_BYTES_IN 4

void paramNullCallback(uint8_t const *, uint16_t){};
void (*CRSF_TX::ParamWriteCallback)(uint8_t const *msg, uint16_t len) = &paramNullCallback;

enum {
    SEND_NA = 0,
    SEND_LNK_STAT = 1 << 0,
    SEND_BATT = 1 << 1,
    SEND_LUA = 1 << 2,
    SEND_MSP = 1 << 3,
    SEND_SYNC = 1 << 4,
    SEND_GPS = 1 << 5,
};
static volatile uint_fast8_t DMA_ATTR send_buffers = SEND_NA;

void CRSF_TX::Begin(void)
{
#ifdef DBF_PIN_CRSF_BYTES_IN
    pinMode(DBF_PIN_CRSF_BYTES_IN, OUTPUT);
    digitalWrite(DBF_PIN_CRSF_BYTES_IN, 0);
#endif

    CRSF::Begin();
    p_UartNextCheck = millis(); //  +UARTwdtInterval * 2;
}

void ICACHE_RAM_ATTR CRSF_TX::CrsfFramePushToFifo(uint8_t *buff, uint8_t size)
{
    buff[size - 1] = CalcCRC(&buff[2], (buff[1] - 1));
    _dev->write(buff, size);
    platform_wd_feed();
}

void CRSF_TX::LinkStatisticsSend(void)
{
    send_buffers |= SEND_LNK_STAT;
}
void CRSF_TX::LinkStatisticsProcess(void)
{
    send_buffers &= ~SEND_LNK_STAT;

    uint8_t len = CRSF_EXT_FRAME_SIZE(LinkStatisticsFrameLength);

    outBuffer[0] = CRSF_ADDRESS_RADIO_TRANSMITTER;
    outBuffer[1] = CRSF_FRAME_SIZE(LinkStatisticsFrameLength);
    outBuffer[2] = CRSF_FRAMETYPE_LINK_STATISTICS;

    // this is nok with volatile
    memcpy(&outBuffer[3], (void *)&LinkStatistics, LinkStatisticsFrameLength);

    CrsfFramePushToFifo(outBuffer, len);
}

void CRSF_TX::GpsSensorSend(void)
{
    if (TLMGPSsensor.valid)
        send_buffers |= SEND_GPS;
}
void CRSF_TX::GpsSensorProcess(void)
{
    send_buffers &= ~SEND_GPS;

    uint8_t frame_len = sizeof(crsf_sensor_gps_t) - 1;
    uint8_t len = CRSF_EXT_FRAME_SIZE(frame_len);

    outBuffer[0] = CRSF_ADDRESS_RADIO_TRANSMITTER;
    outBuffer[1] = CRSF_FRAME_SIZE(frame_len);
    outBuffer[2] = CRSF_FRAMETYPE_GPS;

    // this is nok with volatile
    memcpy(&outBuffer[3], (void *)&TLMGPSsensor, frame_len);

    TLMGPSsensor.valid = false;

    CrsfFramePushToFifo(outBuffer, len);
}

void CRSF_TX::sendLUAresponseToRadio(uint8_t *data, uint8_t size)
{
    memcpy(lua_buff, data, size);
    send_buffers |= SEND_LUA;
}

void CRSF_TX::LuaResponseProcess(void)
{
    send_buffers &= ~SEND_LUA;

    uint8_t const msp_len = CRSF_FRAME_NOT_COUNTED_BYTES + sizeof(lua_buff); // dest, orig
    uint8_t const len = CRSF_EXT_FRAME_SIZE(msp_len);

    outBuffer[0] = CRSF_ADDRESS_RADIO_TRANSMITTER;
    outBuffer[1] = CRSF_FRAME_SIZE(msp_len);
    outBuffer[2] = CRSF_FRAMETYPE_PARAMETER_WRITE;

    // Encapsulated MSP payload
    outBuffer[3] = CRSF_ADDRESS_RADIO_TRANSMITTER;
    outBuffer[4] = CRSF_ADDRESS_CRSF_TRANSMITTER;

    for (uint8_t i = 0; i < sizeof(lua_buff); i++)
        outBuffer[5 + i] = lua_buff[i];

    CrsfFramePushToFifo(outBuffer, len);
}

void CRSF_TX::sendMspPacketToRadio(mspPacket_t &msp)
{
    if (!p_RadioConnected)
        return;

    // CRC included in payloadSize
    uint8_t const msp_len = 3 + msp.payloadSize; // dest, orig, flags
    uint8_t const len = CRSF_EXT_FRAME_SIZE(msp_len);

    if (CRSF_EXT_FRAME_SIZE(CRSF_PAYLOAD_SIZE_MAX) < len)
        /* TODO: split into junks... */
        // just ignore if too big
        return;

    // CRSF packet
    outBuffer[0] = CRSF_ADDRESS_RADIO_TRANSMITTER;
    outBuffer[1] = CRSF_FRAME_SIZE(msp_len);
    outBuffer[2] = CRSF_FRAMETYPE_MSP_RESP;

    // Encapsulated MSP payload
    outBuffer[3] = CRSF_ADDRESS_RADIO_TRANSMITTER;
    outBuffer[4] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    outBuffer[5] = MSP_VERSION | MSP_STARTFLAG; // 0x30, header
    for (uint8_t i = 0; i < msp.payloadSize; i++) {
        outBuffer[6 + i] = msp.payload[i];
    }

    CrsfFramePushToFifo(outBuffer, len);
}

void CRSF_TX::BatterySensorSend(void)
{
    send_buffers |= SEND_BATT;
}
void CRSF_TX::BatteryStatisticsProcess(void)
{
    send_buffers &= ~SEND_BATT;

    uint8_t len = CRSF_EXT_FRAME_SIZE(BattSensorFrameLength);
    memset(outBuffer, 0, len);

    outBuffer[0] = CRSF_ADDRESS_RADIO_TRANSMITTER;
    outBuffer[1] = CRSF_FRAME_SIZE(BattSensorFrameLength);
    outBuffer[2] = CRSF_FRAMETYPE_BATTERY_SENSOR;

    outBuffer[3] = (TLMbattSensor.voltage >> 8) & 0xff;
    outBuffer[4] = TLMbattSensor.voltage & 0xff;
    //outBuffer[5] = (TLMbattSensor.current >> 8) & 0xff;
    //outBuffer[6] = TLMbattSensor.current & 0xff;
    //outBuffer[7] = (TLMbattSensor.capacity >> 16) & 0xff;
    //outBuffer[9] = (TLMbattSensor.capacity >> 8) & 0xff;
    //outBuffer[10] = TLMbattSensor.capacity & 0xff;
    //outBuffer[11] = TLMbattSensor.remaining;

    CrsfFramePushToFifo(outBuffer, len);
}

int CRSF_TX::sendSyncPacketToRadio()
{
#if (FEATURE_OPENTX_SYNC)
    if (RCdataLastRecv && p_RadioConnected)
    {
        uint32_t current = millis();
        int32_t offset = (int32_t)(OpenTXsyncOffset - RequestedRCpacketAdvance);

        // Adjust radio timing if not in requested window or not sent within 200ms
        if (/*(100 < offset) || // too early
            (-50 > offset) || // too late*/
            (OpenTXsyncPakcetInterval <= (current - OpenTXsynNextSend)))
        {
            OpenTXsynNextSend = current;

            /*DEBUG_PRINT("rate: ");
            DEBUG_PRINT(RequestedRCpacketInterval);
            DEBUG_PRINT(" offset: ");
            DEBUG_PRINTLN(offset);*/

            uint32_t packetRate = RequestedRCpacketInterval;
            packetRate *= 10; //convert from us to right format

            offset *= 10;

            uint8_t len = CRSF_EXT_FRAME_SIZE(OpenTXsyncFrameLength);

            outBuffer[0] = CRSF_ADDRESS_RADIO_TRANSMITTER;         // 0xEA
            outBuffer[1] = CRSF_FRAME_SIZE(OpenTXsyncFrameLength); // 13
            outBuffer[2] = CRSF_FRAMETYPE_RADIO_ID;                // 0x3A

            outBuffer[3] = CRSF_ADDRESS_RADIO_TRANSMITTER; //0XEA
            outBuffer[4] = 0x00;                           //??? not sure doesn't seem to matter
            outBuffer[5] = CRSF_FRAMETYPE_OPENTX_SYNC;     //0X10

            outBuffer[6] = (packetRate & 0xFF000000) >> 24;
            outBuffer[7] = (packetRate & 0x00FF0000) >> 16;
            outBuffer[8] = (packetRate & 0x0000FF00) >> 8;
            outBuffer[9] = (packetRate & 0x000000FF) >> 0;

            outBuffer[10] = (offset & 0xFF000000) >> 24;
            outBuffer[11] = (offset & 0x00FF0000) >> 16;
            outBuffer[12] = (offset & 0x0000FF00) >> 8;
            outBuffer[13] = (offset & 0x000000FF) >> 0;

            CrsfFramePushToFifo(outBuffer, len);
            return 0;
        }
    }
#endif /* FEATURE_OPENTX_SYNC */
    return -1;
}

void CRSF_TX::processPacket(uint8_t const *input)
{
    if (p_RadioConnected == false)
    {
        p_RadioConnected = true;
#if (FEATURE_OPENTX_SYNC)
        RCdataLastRecv = 0;
        OpenTXsynNextSend = millis(); //+60;
#endif
        DEBUG_PRINT("CRSF Connected. Baud ");
        if (p_slowBaudrate)
            DEBUG_PRINTLN("115k");
        else
            DEBUG_PRINTLN("400k");
        connected();
    }

    switch (input[0]) // check CRSF command
    {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
        {
#if (FEATURE_OPENTX_SYNC)
            RCdataLastRecv = micros();
#endif
            (RCdataCallback1)((crsf_channels_t *)&input[1]); // run new RC data callback
            break;
        }
        case CRSF_FRAMETYPE_PARAMETER_WRITE:
        {
            if (input[1] == CRSF_ADDRESS_CRSF_TRANSMITTER &&
                input[2] == CRSF_ADDRESS_RADIO_TRANSMITTER)
            {
                ParamWriteCallback(&input[3], 2);
            }
        }
        case CRSF_FRAMETYPE_MSP_REQ:
        case CRSF_FRAMETYPE_MSP_WRITE:
        {
            if (input[1] == CRSF_ADDRESS_FLIGHT_CONTROLLER &&
                input[2] == CRSF_ADDRESS_RADIO_TRANSMITTER)
            {
                MspCallback(&input[3]); // pointer to MSP packet
            }
            break;
        }
        default:
            break;
    };
}

uint8_t CRSF_TX::handleUartIn(volatile uint8_t &rx_data_rcvd) // Merge with RX version...
{
    uint8_t can_send = 0;
    uint8_t split_cnt = 0;

    for (split_cnt = 0; (rx_data_rcvd == 0) && _dev->available() && (split_cnt < 16); split_cnt++)
    {
#ifdef DBF_PIN_CRSF_BYTES_IN
        digitalWrite(DBF_PIN_CRSF_BYTES_IN, 1);
#endif
        uint8_t *ptr = ParseInByte(_dev->read());
        if (ptr)
        {
            processPacket(ptr);
            platform_wd_feed();

            if (rx_data_rcvd == 0)
            {
                /* Can write right after successful package reception */
                sendSyncPacketToRadio();

                if (!rx_data_rcvd)
                {
                    if (send_buffers & SEND_LNK_STAT)
                        LinkStatisticsProcess();
                    else if (send_buffers & SEND_BATT)
                        BatteryStatisticsProcess();
                    else if (send_buffers & SEND_LUA)
                        LuaResponseProcess();
                    else if (send_buffers & SEND_GPS)
                        GpsSensorProcess();
                    else
                        can_send = 1;
                }
            }
        }

#ifdef DBF_PIN_CRSF_BYTES_IN
        digitalWrite(DBF_PIN_CRSF_BYTES_IN, 0);
#endif
    }

    if (rx_data_rcvd == 0)
        uart_wdt();

    return can_send;
}

void CRSF_TX::uart_wdt(void)
{
    uint32_t now = millis();
    if (UARTwdtInterval <= (now - p_UartNextCheck))
    {
        DEBUG_PRINT("CRSF Bad:Good ");
        DEBUG_PRINT(BadPktsCount);
        DEBUG_PRINT(":");
        DEBUG_PRINTLN(GoodPktsCount);

        if (BadPktsCount >= GoodPktsCount)
        {
            if (p_RadioConnected == true)
            {
                DEBUG_PRINT("CRSF UART Disconnect. ");
                disconnected();
                p_RadioConnected = false;
#if (FEATURE_OPENTX_SYNC)
                OpenTXsynNextSend = 0;
                OpenTXsyncOffset = 0;
#endif
            }

            _dev->end();
            platform_wd_feed();

            if (p_slowBaudrate)
            {
                _dev->Begin(CRSF_TX_BAUDRATE_FAST);
                DEBUG_PRINTLN("Switch to 400000 baud");
            }
            else
            {
                _dev->Begin(CRSF_TX_BAUDRATE_SLOW);
                DEBUG_PRINTLN("Switch to 115000 baud");
            }
            p_slowBaudrate = !p_slowBaudrate;
        }

        p_UartNextCheck = now;
        BadPktsCount = 0;
        GoodPktsCount = 0;
    }
}
