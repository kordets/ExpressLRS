#include "GHST.h"
#include "CRSF.h" // CRSF_FRAMETYPE_LINK_STATISTICS
#include "crc.h"
#include "platform.h"
#include "utils.h"
#include "debug_elrs.h"

void MspNullCallback(uint8_t const *const) {}
void (*GHST::MspCallback)(uint8_t const *const input) = MspNullCallback;

static ghstRcFrame_t rc_data;
static volatile uint8_t stats_updated;

void GHST::Begin(void)
{
    rc_data.hdr.addr = GHST_ADDR_FC;
    rc_data.hdr.len = sizeof(rc_data) - offsetof(ghstRcFrame_t, type) - 1; /* CRC is not included */
    rc_data.type = GHST_UL_RC_CHANS_HS4_5TO8;

    TLMbattSensor.capacity = 0;
    TLMbattSensor.current = 0;
    TLMbattSensor.voltage = 0;

    stats_updated = 0;

    _dev->flush_read();
}

void GHST::handleUartIn(void)
{
    int available = _dev->available();
    uint8_t *ptr;

    if (16 < available) available = 16;
    else if (available < 0) available = 0;

    while (available--) {
        ptr = ParseInByte(_dev->read());
        if (ptr)
            processPacket(ptr);
    }
}

void GHST::sendRCFrameToFC(struct crsf_channels_s * channels) const
{
    rc_data.channels.ch1to4.ch1 = channels->ch0 << 1;
    rc_data.channels.ch1to4.ch2 = channels->ch1 << 1;
    rc_data.channels.ch1to4.ch3 = channels->ch2 << 1;
    rc_data.channels.ch1to4.ch4 = channels->ch3 << 1;

    if (stats_updated) {
        rc_data.type = GHST_UL_RC_CHANS_HS4_RSSI;
        rc_data.channels.stat.lq = LinkStatistics.uplink_Link_quality;
        rc_data.channels.stat.rssi = -1 * (int8_t)LinkStatistics.uplink_RSSI_1;
        stats_updated = 0;
    } else {
        rc_data.type = GHST_UL_RC_CHANS_HS4_5TO8;
        rc_data.channels.aux.cha = channels->ch4 >> 3;
        rc_data.channels.aux.chb = channels->ch5 >> 3;
        rc_data.channels.aux.chc = channels->ch6 >> 3;
        rc_data.channels.aux.chd = channels->ch7 >> 3;
    }
    sendFrameToFC((uint8_t*)&rc_data, sizeof(rc_data));
}

void GHST::LinkStatisticsPack(uint8_t *const output,
                              uint_fast8_t ul_lq) const
{
    // NOTE: output is only 5 bytes + 6bits (MSB)!!
    /* THIS IS COPY FROM CRSF!!! KEEP ALIGNED FOR NOW */

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
    output[3] = ul_lq;
    output[4] = (TLMbattSensor.voltage & 0x00FF);
    output[5] = CRSF_FRAMETYPE_LINK_STATISTICS << 2;
}

void GHST::LinkStatisticsSend(void) const
{
    stats_updated = 1;
}

uint8_t GHST::GpsStatsPack(uint8_t *const output)
{
    (void)output;
    // TODO: implement GHST GPS pack
    return 0;
}

void GHST::sendMSPFrameToFC(uint8_t const *const packet, uint8_t len) const
{
    // TODO: not support atm
}

void GHST::sendFrameToFC(uint8_t *buff, uint8_t size) const
{
    buff[size - 1] = crc8_dvb_s2(&buff[sizeof(ghstHeader_t)], (size - sizeof(ghstHeader_t) - 1));
#if !NO_DATA_TO_FC
    uint32_t irq = _SAVE_IRQ();
    _dev->write(buff, size);
    _RESTORE_IRQ(irq);
#endif
}

uint8_t *GHST::ParseInByte(uint8_t inChar)
{
    static uint8_t SerialInBuffer[GHST_FRAME_SIZE_MAX*2];
    uint8_t *packet_ptr = NULL;

    if (SerialInPacketPtr >= sizeof(SerialInBuffer))
    {
        // we reached the maximum allowable packet length,
        // so start again because shit fucked up hey.
        SerialInPacketPtr = 0;
        frameActive = false;
    }

    // store byte
    SerialInBuffer[SerialInPacketPtr++] = inChar;

    // GHST Frame:
    // | address | payload_len | payload* | crc |

    if (frameActive == false) {
        if (inChar == GHST_ADDR_RX) {
            frameActive = true;
            SerialInPacketLen = 0;
        } else {
            SerialInPacketPtr = 0;
        }
    } else {
        if (SerialInPacketLen == 0) {
            // we read the packet length and save it
            SerialInCrc = 0;
            SerialInPacketLen = inChar;
            SerialInPacketStart = SerialInPacketPtr;
            if ((SerialInPacketLen < 2) || (GHST_FRAME_SIZE_MAX < SerialInPacketLen)) {
                // failure -> discard
                frameActive = false;
                SerialInPacketPtr = 0;
            }
        } else {
            if ((SerialInPacketPtr - SerialInPacketStart) >= (SerialInPacketLen)) {
                /* Check packet CRC */
                if (SerialInCrc == inChar) {
                    packet_ptr = &SerialInBuffer[SerialInPacketStart];
                } else {
#if 0
                    // https://crccalc.com/
                    // CRC algorithm: CRC-8/DVB-S2
                    DEBUG_PRINTF("UART CRC %X != %X\n", SerialInCrc, inChar);
                    for (int i = (SerialInPacketStart-2); i < (SerialInPacketLen + 2); i++)
                    {
                        DEBUG_PRINTF(" 0x%X", SerialInBuffer[i]);
                    }
                    DEBUG_PRINTF("\n");
#elif defined(DEBUG_SERIAL)
                    DEBUG_PRINTF("!C");
#endif
                }

                // packet handled, start next
                frameActive = false;
                SerialInPacketPtr = 0;
                SerialInPacketLen = 0;
            } else {
                // Calc crc on the fly
                SerialInCrc = crc8_dvb_s2(inChar, SerialInCrc);
            }
        }
    }

    return packet_ptr;
}

void GHST::processPacket(uint8_t const *data)
{
    switch (data[0]) {
        case CRSF_FRAMETYPE_COMMAND: {
            if (data[1] == 0x62 && data[2] == 0x6c) {
                DEBUG_PRINTF("Jumping to Bootloader...\n");
                delay(200);
                platform_restart();
            }
            break;
        }
        case GHST_DL_PACK_STAT: {
            memcpy(&TLMbattSensor , data, sizeof(TLMbattSensor));
            break;
        }
    }
}
