#ifndef CRSF_RX_H_
#define CRSF_RX_H_

#include "CRSF.h"

class CRSF_RX : public CRSF
{
public:
    CRSF_RX(HwSerial *dev) : CRSF(dev) {}
    CRSF_RX(HwSerial &dev) : CRSF(&dev) {}

    void handleUartIn(volatile uint8_t &rx_data_rcvd);

    void ICACHE_RAM_ATTR sendRCFrameToFC(crsf_channels_t * channels);
    void LinkStatisticsSend();
    void ICACHE_RAM_ATTR sendMSPFrameToFC(mspPacket_t& packet);
    void ICACHE_RAM_ATTR sendMSPFrameToFC(uint8_t const *const packet, uint8_t len);

private:
    void ICACHE_RAM_ATTR sendFrameToFC(uint8_t *buff, uint8_t size);
    void processPacket(uint8_t const *data);
};

#endif /* CRSF_RX_H_ */
