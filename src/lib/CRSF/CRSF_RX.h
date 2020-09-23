#ifndef CRSF_RX_H_
#define CRSF_RX_H_

#include "CRSF.h"

class CRSF_RX : public CRSF
{
public:
    CRSF_RX(HwSerial *dev) : CRSF(dev) {}
    CRSF_RX(HwSerial &dev) : CRSF(&dev) {}

    void Begin(void);

    void handleUartIn(volatile uint8_t &rx_data_rcvd);

    void ICACHE_RAM_ATTR sendRCFrameToFC(crsf_channels_t * channels);
    void LinkStatisticsSend();
    void ICACHE_RAM_ATTR sendMSPFrameToFC(uint8_t const *const packet, uint8_t len);

    crsf_msp_packet_fc_t msp_packet;

private:
    void ICACHE_RAM_ATTR sendFrameToFC(uint8_t *buff, uint8_t size);
    void processPacket(uint8_t const *data);

    crsf_channels_msg_s p_crsf_channels;
};

#endif /* CRSF_RX_H_ */
