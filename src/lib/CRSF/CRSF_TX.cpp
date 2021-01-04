#include "CRSF_TX.h"
#include "debug_elrs.h"
#include <string.h>

//#define DBF_PIN_CRSF_BYTES_IN 4
#ifdef DBF_PIN_CRSF_BYTES_IN
struct gpio_out crsf_byte_in;
#endif

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
volatile uint_fast8_t DMA_ATTR send_buffers;

elrs_lua_packet_t DMA_ATTR p_lua_packet;
OpenTxSyncPacket_s DMA_ATTR p_otx_sync_packet;
crsf_msp_packet_radio_s DMA_ATTR p_msp_packet;

void CRSF_TX::Begin(void)
{
#ifdef DBF_PIN_CRSF_BYTES_IN
    crsf_byte_in = gpio_out_setup(DBF_PIN_CRSF_BYTES_IN, 0);
#endif

    /* Initialize used messages */

    LinkStatistics.header.device_addr = CRSF_ADDRESS_RADIO_TRANSMITTER;
    LinkStatistics.header.frame_size = sizeof(LinkStatistics) - CRSF_FRAME_START_BYTES;
    LinkStatistics.header.type = CRSF_FRAMETYPE_LINK_STATISTICS;

    TLMbattSensor.header.device_addr = CRSF_ADDRESS_RADIO_TRANSMITTER;
    TLMbattSensor.header.frame_size = sizeof(TLMbattSensor) - CRSF_FRAME_START_BYTES;
    TLMbattSensor.header.type = CRSF_FRAMETYPE_BATTERY_SENSOR;

    TLMGPSsensor.header.device_addr = CRSF_ADDRESS_RADIO_TRANSMITTER;
    TLMGPSsensor.header.frame_size = sizeof(TLMGPSsensor) - CRSF_FRAME_START_BYTES;
    TLMGPSsensor.header.type = CRSF_FRAMETYPE_GPS;

    p_lua_packet.header.device_addr = CRSF_ADDRESS_RADIO_TRANSMITTER;
    p_lua_packet.header.frame_size = sizeof(p_lua_packet) - CRSF_FRAME_START_BYTES;
    p_lua_packet.header.type = CRSF_FRAMETYPE_PARAMETER_WRITE;
    p_lua_packet.header.dest_addr = CRSF_ADDRESS_RADIO_TRANSMITTER;
    p_lua_packet.header.orig_addr = CRSF_ADDRESS_CRSF_TRANSMITTER;

    p_otx_sync_packet.header.device_addr = CRSF_ADDRESS_RADIO_TRANSMITTER;
    p_otx_sync_packet.header.frame_size = sizeof(p_otx_sync_packet) - CRSF_FRAME_START_BYTES;
    p_otx_sync_packet.header.type = CRSF_FRAMETYPE_RADIO_ID;
    p_otx_sync_packet.header.dest_addr = CRSF_ADDRESS_RADIO_TRANSMITTER;
    p_otx_sync_packet.header.orig_addr = CRSF_ADDRESS_CRSF_TRANSMITTER;
    p_otx_sync_packet.otx_id = CRSF_FRAMETYPE_OPENTX_SYNC;

    p_msp_packet.header.device_addr = CRSF_ADDRESS_RADIO_TRANSMITTER;
    p_msp_packet.header.frame_size = 0;
    p_msp_packet.header.type = CRSF_FRAMETYPE_MSP_RESP;
    p_msp_packet.header.dest_addr = CRSF_ADDRESS_RADIO_TRANSMITTER;
    p_msp_packet.header.orig_addr = CRSF_ADDRESS_FLIGHT_CONTROLLER;

    CRSF::Begin();
    p_UartNextCheck = millis(); //  +UARTwdtInterval * 2;
}

void ICACHE_RAM_ATTR CRSF_TX::CrsfFramePushToFifo(uint8_t *buff, uint8_t size) const
{
    buff[size - 1] = CalcCRC(&buff[2], (buff[1] - 1));
    _dev->write(buff, size);
    platform_wd_feed();
#if defined(BT_SERIAL)
    // Send to BT serial
    BT_SERIAL.write(buff, size);
    platform_wd_feed();
#endif // BT_SERIAL
}

void CRSF_TX::LinkStatisticsSend(void) const
{
    send_buffers |= SEND_LNK_STAT;
}
void CRSF_TX::LinkStatisticsProcess(void) const
{
    send_buffers &= ~SEND_LNK_STAT;
    CrsfFramePushToFifo((uint8_t*)&LinkStatistics, sizeof(LinkStatistics));
}

void CRSF_TX::BatterySensorSend(void) const
{
    send_buffers |= SEND_BATT;
}
void CRSF_TX::BatteryStatisticsProcess(void) const
{
    send_buffers &= ~SEND_BATT;
    CrsfFramePushToFifo((uint8_t*)&TLMbattSensor, sizeof(TLMbattSensor));
}

void CRSF_TX::GpsSensorSend(void) const
{
    if (tlm_gps_valid)
        send_buffers |= SEND_GPS;
}
void CRSF_TX::GpsSensorProcess(void)
{
    send_buffers &= ~SEND_GPS;
    CrsfFramePushToFifo((uint8_t *)&TLMGPSsensor, sizeof(TLMGPSsensor));
    tlm_gps_valid = 0;
}

void CRSF_TX::sendLUAresponseToRadio(uint8_t * const data, uint8_t const size) const
{
    memcpy(p_lua_packet.buffer, data, size);
    send_buffers |= SEND_LUA;
}
void CRSF_TX::LuaResponseProcess(void) const
{
    send_buffers &= ~SEND_LUA;
    CrsfFramePushToFifo((uint8_t*)&p_lua_packet, sizeof(p_lua_packet));
}

void CRSF_TX::sendMspPacketToRadio(mspPacket_t &msp) const
{
    if (!p_RadioConnected)
        return;

    // CRC included in payloadSize
    uint8_t const msp_len = 3 + msp.payloadSize; // dest, orig, flags
    uint8_t const len = CRSF_EXT_FRAME_SIZE(msp_len);

    if (sizeof(p_msp_packet.buffer) < msp.payloadSize)
        /* TODO: split into junks... */
        // just ignore if too big
        return;

    p_msp_packet.header.frame_size = CRSF_FRAME_SIZE(msp_len);
    // Fill encapsulated MSP payload
    p_msp_packet.flags = MSP_VERSION | MSP_STARTFLAG;
    for (uint8_t i = 0; i < msp.payloadSize; i++) {
        p_msp_packet.buffer[i] = msp.payload[i];
    }

    CrsfFramePushToFifo((uint8_t*)&p_msp_packet, len);
}

int CRSF_TX::sendSyncPacketToRadio()
{
#if (FEATURE_OPENTX_SYNC)
    if (RCdataLastRecv && p_RadioConnected)
    {
        uint32_t const current = millis();
        // Adjust radio timing if not in requested window or not sent within 200ms
        if (OTX_SYNC_INTERVAL <= (current - OpenTXsynNextSend))
        {
            int32_t offset = (int32_t)(OpenTXsyncOffset - OTX_SYNC_ADVANCE);
            OpenTXsynNextSend = current;

            //DEBUG_PRINTF("Sync: int=%u,off=%d\n", RequestedRCpacketInterval, offset);

            p_otx_sync_packet.packetRate =
                __builtin_bswap32(RequestedRCpacketInterval);
            p_otx_sync_packet.offset = __builtin_bswap32(offset * 10);
            CrsfFramePushToFifo((uint8_t*)&p_otx_sync_packet, sizeof(p_otx_sync_packet));
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
        write_u32(&RCdataLastRecv, 0);
        OpenTXsynNextSend = millis();
#endif
        DEBUG_PRINTF("CRSF Connected. Baud %uk\n", (p_slowBaudrate ? 115 : 400));
        connected();
    }

    switch (input[0]) // check CRSF command
    {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
        {
#if (FEATURE_OPENTX_SYNC)
            write_u32(&RCdataLastRecv, micros());
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

uint8_t CRSF_TX::handleUartIn(void)
{
    int available = _dev->available();
    uint8_t *ptr;
    uint8_t can_send = 0;

    if (16 < available) available = 16;
    else if (available < 0) available = 0;

    while (available--) {
#ifdef DBF_PIN_CRSF_BYTES_IN
        gpio_out_write(crsf_byte_in, 1);
#endif
        ptr = ParseInByte(_dev->read());
        if (ptr) {
            processPacket(ptr);
            platform_wd_feed();

            /* Can write right after successful package reception */
            sendSyncPacketToRadio();

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

#ifdef DBF_PIN_CRSF_BYTES_IN
        gpio_out_write(crsf_byte_in, 0);
#endif
    }

    if (!can_send)
        uart_wdt();

    return can_send;
}

void CRSF_TX::uart_wdt(void)
{
    uint32_t now = millis();
    if (UARTwdtInterval <= (now - p_UartNextCheck))
    {
        DEBUG_PRINTF("CRSF Bad:Good %u:%u\n", BadPktsCount, GoodPktsCount);

        if (BadPktsCount >= GoodPktsCount)
        {
            if (p_RadioConnected == true)
            {
                DEBUG_PRINTF("CRSF UART Disconnect. ");
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
                DEBUG_PRINTF("Switch to 400000 baud\n");
            }
            else
            {
                _dev->Begin(CRSF_TX_BAUDRATE_SLOW);
                DEBUG_PRINTF("Switch to 115000 baud\n");
            }
            p_slowBaudrate = !p_slowBaudrate;
        }

        p_UartNextCheck = now;
        BadPktsCount = 0;
        GoodPktsCount = 0;
    }
}
