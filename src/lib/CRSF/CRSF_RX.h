#ifndef CRSF_RX_H_
#define CRSF_RX_H_

#include "CRSF.h"

class CRSF_RX : public CRSF
{
public:
    CRSF_RX(HwSerial &dev) : CRSF(&dev) {}

    void Begin(void);

    void handleUartIn(volatile uint8_t &rx_data_rcvd);

    void sendRCFrameToFC(crsf_channels_t * channels) const;
    void LinkStatisticsSend(void) const;
    void sendMSPFrameToFC(uint8_t const *const packet, uint8_t len) const;

private:
    void sendFrameToFC(uint8_t *buff, uint8_t size) const;
    void processPacket(uint8_t const *data);
};

#endif /* CRSF_RX_H_ */
