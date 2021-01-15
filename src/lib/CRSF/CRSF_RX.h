#ifndef CRSF_RX_H_
#define CRSF_RX_H_

#include "CRSF.h"

class CRSF_RX : public CRSF
{
public:
    CRSF_RX(HwSerial &dev) : CRSF(&dev) {}

    void Begin(void);

    void handleUartIn(void);

    void sendRCFrameToFC(rc_channels_t * channels) const;
    void LinkStatisticsSend(LinkStats_t & stats) const;
    void sendMSPFrameToFC(mspPacket_t & msp) const;

private:
    void sendFrameToFC(uint8_t *buff, uint8_t size) const;
    void processPacket(uint8_t const *data);
};

#endif /* CRSF_RX_H_ */
