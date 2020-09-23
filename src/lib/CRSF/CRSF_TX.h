#ifndef CRSF_TX_H_
#define CRSF_TX_H_

#include "CRSF.h"

/* OpenTX timer control sync packet */
typedef struct OpenTxSyncPacket_s {
    crsf_ext_header_s   header;
    uint8_t             otx_id;
    uint32_t            packetRate;
    uint32_t            offset;
    uint8_t             crc;
} PACKED OpenTxSyncPacket_t;

/* LUA packet is used only on TX */
typedef struct elrs_lua_packet_s
{
    crsf_ext_header_t header;
    uint8_t buffer[5];
    uint8_t crc;
} PACKED elrs_lua_packet_t;


class CRSF_TX : public CRSF
{
public:
    CRSF_TX(HwSerial *dev) : CRSF(dev) {}
    CRSF_TX(HwSerial &dev) : CRSF(dev) {}

    void Begin(void);

    // Handle incoming data
    uint8_t handleUartIn(volatile uint8_t &rx_data_rcvd);

    // Send to RADIO
    void LinkStatisticsSend(void);
    void BatterySensorSend(void);
    void GpsSensorSend(void);
    void sendLUAresponseToRadio(uint8_t *data, uint8_t len);
    void sendMspPacketToRadio(mspPacket_t &msp);

    // OpenTX Syncing
    void ICACHE_RAM_ATTR setRcPacketRate(uint32_t interval)
    {
#if (FEATURE_OPENTX_SYNC)
        // Scale value to correct format
        RequestedRCpacketInterval = interval * 10;
#endif
    }

    void ICACHE_RAM_ATTR UpdateOpenTxSyncOffset(uint32_t current_us) // called from timer
    {
#if (FEATURE_OPENTX_SYNC)
        OpenTXsyncOffset = current_us - RCdataLastRecv;
#endif
    }

    ///// Callbacks /////
    static void (*ParamWriteCallback)(uint8_t const *msg, uint16_t len);

    ///// Variables /////

private:
    void uart_wdt(void);
    void processPacket(uint8_t const *input);
    void ICACHE_RAM_ATTR CrsfFramePushToFifo(uint8_t *buff, uint8_t size);
    void LinkStatisticsProcess(void);
    void BatteryStatisticsProcess(void);
    void GpsSensorProcess(void);
    void LuaResponseProcess(void);

#if (FEATURE_OPENTX_SYNC)

#define OpenTXsyncPakcetInterval 200 // in ms
#define RequestedRCpacketAdvance 500 // 800 timing adcance in us

    volatile uint32_t RCdataLastRecv;
    volatile int32_t OpenTXsyncOffset;
    uint32_t RequestedRCpacketInterval;
    uint32_t OpenTXsynNextSend;
#endif /* FEATURE_OPENTX_SYNC */
    int sendSyncPacketToRadio();

    // for the UART wdt, every 1000ms we change bauds when connect is lost
#define UARTwdtInterval 1000
    uint32_t p_UartNextCheck;

    bool p_slowBaudrate;
    bool p_RadioConnected; // connected state
};

#endif /* CRSF_TX_H_ */
