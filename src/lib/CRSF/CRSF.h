#ifndef H_CRSF
#define H_CRSF

#include "HwSerial.h"
#include "helpers.h"
#include "crc.h"
#include "platform.h"
#include "utils.h"
#include "msp.h"
#include "rc_channels.h"

#if PROTOCOL_ELRS_TO_FC
#define CRSF_RX_BAUDRATE      691200
#else // !PROTOCOL_ELRS_TO_FC
#define CRSF_RX_BAUDRATE      420000
#define CRSF_RX_BAUDRATE_V3   921600
#endif // PROTOCOL_ELRS_TO_FC
#define CRSF_TX_BAUDRATE_FAST 400000
#define CRSF_TX_BAUDRATE_SLOW 115200
#define CRSF_NUM_CHANNELS     16         // Number of input channels

#define CRSF_SYNC_BYTE 0xC8

#define CRSF_PAYLOAD_SIZE_MAX 62
#define CRSF_FRAME_START_BYTES 2 // address + len (start of the CRSF frame, not counted to frame len)
#define CRSF_FRAME_HEADER_BYTES 2 // type + crc
#define CRSF_FRAME_SIZE(payload_size) ((payload_size) + CRSF_FRAME_HEADER_BYTES) // See crsf_header_t.frame_size
#define CRSF_EXT_FRAME_SIZE(payload_size) (CRSF_FRAME_SIZE(payload_size) + CRSF_FRAME_START_BYTES)
#define CRSF_FRAME_SIZE_MAX (CRSF_PAYLOAD_SIZE_MAX + CRSF_FRAME_START_BYTES)

#define CRSF_MSP_FRAME_HEADER_BYTES 4 // type, dest, orig, crc
#define CRSF_MSP_FRAME_SIZE(payload_size) (CRSF_FRAME_SIZE(payload_size) + CRSF_MSP_FRAME_HEADER_BYTES)


#define CRSF_GEN_POLY 0xD5
#define CRSF_CMD_POLY 0xBA

//////////////////////////////////////////////////////////////

enum crsf_frame_type_e
{
    CRSF_FRAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
    CRSF_FRAMETYPE_LINK_STATISTICS_ELRS = 0x15,
    CRSF_FRAMETYPE_OPENTX_SYNC = 0x10,
    CRSF_FRAMETYPE_RADIO_ID = 0x3A,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED_ELRS = 0x17,
    CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED = 0x17,
    CRSF_FRAMETYPE_LINK_STATISTICS_RX = 0x1C,
    CRSF_FRAMETYPE_LINK_STATISTICS_TX = 0x1D,
    CRSF_FRAMETYPE_ATTITUDE = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    CRSF_FRAMETYPE_DEVICE_PING = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,
    CRSF_FRAMETYPE_COMMAND = 0x32,
    // MSP commands
    CRSF_FRAMETYPE_MSP_REQ = 0x7A,   // response request using msp sequence as command
    CRSF_FRAMETYPE_MSP_RESP = 0x7B,  // reply with 58 byte chunked binary
    CRSF_FRAMETYPE_MSP_WRITE = 0x7C, // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
};

enum
{
    CRSF_COMMAND_SUBCMD_GENERAL = 0x0A,    // general command
};

enum
{
    CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_PROPOSAL = 0x70,    // proposed new CRSF port speed
    CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_RESPONSE = 0x71,    // response to the proposed CRSF port speed
};

enum crsf_addr_e
{
    CRSF_ADDRESS_BROADCAST = 0x00,
    CRSF_ADDRESS_USB = 0x10,
    CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
    CRSF_ADDRESS_RESERVED1 = 0x8A,
    CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
    CRSF_ADDRESS_GPS = 0xC2,
    CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
    CRSF_ADDRESS_RESERVED2 = 0xCA,
    CRSF_ADDRESS_RACE_TAG = 0xCC,
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
    CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
    CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE,
};

typedef enum
{
    CRSF_UINT8 = 0,
    CRSF_INT8 = 1,
    CRSF_UINT16 = 2,
    CRSF_INT16 = 3,
    CRSF_UINT32 = 4,
    CRSF_INT32 = 5,
    CRSF_UINT64 = 6,
    CRSF_INT64 = 7,
    CRSF_FLOAT = 8,
    CRSF_TEXT_SELECTION = 9,
    CRSF_STRING = 10,
    CRSF_FOLDER = 11,
    CRSF_INFO = 12,
    CRSF_COMMAND = 13,
    CRSF_VTX = 15,
    CRSF_OUT_OF_RANGE = 127,
} crsf_value_type_e;

typedef struct crsf_header_s
{
    uint8_t device_addr; // from crsf_addr_e
    uint8_t frame_size;  // counts size after this byte, so it must be the payload size + 2 (type and crc)
    uint8_t type;        // from crsf_frame_type_e
} PACKED crsf_header_t;

// Used by extended header frames (type in range 0x28 to 0x96)
typedef struct crsf_ext_header_s
{
    // Common header fields, see crsf_header_t
    uint8_t device_addr;
    uint8_t frame_size;
    uint8_t type;
    // Extended fields
    uint8_t dest_addr;
    uint8_t orig_addr;
} PACKED crsf_ext_header_t;

// Used by command frames (type in range 0x28 to 0x96)
typedef struct crsf_command_header_s
{
    // Common header fields, see crsf_header_t
    uint8_t device_addr;
    uint8_t frame_size;
    uint8_t type;
    // Extended fields
    uint8_t dest_addr;
    uint8_t orig_addr;
    // Command fields
    uint8_t command;
    uint8_t sub_command;
} PACKED crsf_command_header_t;

typedef union crsf_buffer_u
{
    uint8_t type;
    struct {
        uint8_t payload[1];
    } normal;
    struct {
        // Extended fields
        uint8_t dest_addr;
        uint8_t orig_addr;
        uint8_t payload[1];
    } extended;
    struct {
        // Extended fields
        uint8_t dest_addr;
        uint8_t orig_addr;
        // Command fields
        uint8_t command;
        uint8_t sub_command;
        uint8_t payload[1];
    } command;
} crsf_buffer_t;

// RC data frame
typedef struct crsf_channels_msg_s
{
    crsf_header_t header;
    rc_channels_rx_t data; // see rc_channels.h
    uint8_t crc;
} PACKED crsf_channels_msg_t;

// Used by extended header frames (type in range 0x28 to 0x96)
typedef struct crsf_sensor_battery_s
{
    crsf_header_t header;
    uint16_t voltage;  // mv * 100
    uint16_t current;  // ma * 100
    uint32_t capacity : 24; // mah
    uint32_t remaining : 8; // %
    uint8_t crc;
} PACKED crsf_sensor_battery_t;

/*
 * 0x14 Link statistics
 * Payload:
 *
 * uint8_t Uplink RSSI Ant. 1 ( dBm * -1 )
 * uint8_t Uplink RSSI Ant. 2 ( dBm * -1 )
 * uint8_t Uplink Package success rate / Link quality ( % )
 * int8_t Uplink SNR ( db )
 * uint8_t Diversity active antenna ( enum ant. 1 = 0, ant. 2 )
 * uint8_t RF Mode ( enum 4fps = 0 , 50fps, 150hz)
 * uint8_t Downlink TX Power ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW, 250mW )
 * uint8_t Downlink RSSI ( dBm * -1 )
 * uint8_t Downlink package success rate / Link quality ( % )
 * int8_t Downlink SNR ( db )
 *
 * Uplink is the connection from the ground to the UAV and downlink the opposite direction.
 * Uplink:   PILOT => UAV
 * Downlink: UAV   => PILOT
 */
// Compact version of uplink link statistics
typedef struct elrsPayloadLinkstatistics_s {
    uint8_t uplink_RSSI;
    uint8_t uplink_Link_quality;
    int8_t  uplink_SNR;
    uint8_t rf_Mode;
} PACKED elrsLinkStatistics_t;

typedef struct crsfLinkStatisticsMsg_elrs_s
{
    crsf_header_t header;
    elrsLinkStatistics_t stats;
    uint8_t crc;
} PACKED crsfLinkStatisticsMsg_elrs_t;

typedef struct crsfLinkStatistics_s
{
    uint8_t uplink_RSSI_1;
    uint8_t uplink_RSSI_2;
    uint8_t uplink_Link_quality; // this goes to opentx rssi
    int8_t uplink_SNR;
    uint8_t active_antenna;
    uint8_t rf_Mode;
    uint8_t uplink_TX_Power;
    uint8_t downlink_RSSI;
    uint8_t downlink_Link_quality;
    int8_t downlink_SNR;
} PACKED crsfLinkStatistics_t;

typedef struct crsfLinkStatisticsMsg_s
{
    crsf_header_t header;
    crsfLinkStatistics_t stats;
    uint8_t crc;
} PACKED crsfLinkStatisticsMsg_t;

/*
 * 0x1C Link statistics RX (CRSF_FRAMETYPE_LINK_STATISTICS_RX)
 * Payload:
 *
 * uint8_t Downlink RSSI ( dBm * -1 )
 * uint8_t Downlink RSSI ( % )
 * uint8_t Downlink Package success rate / Link quality ( % )
 * int8_t Downlink SNR ( db )
 * uint8_t Uplink RF Power ( db )
 */
typedef struct crsfPayloadLinkstatisticsRx_s {
    uint8_t downlink_RSSI_1;
    uint8_t downlink_RSSI_1_percentage;
    uint8_t downlink_Link_quality;
    int8_t  downlink_SNR;
    uint8_t uplink_power;
} crsfLinkStatisticsRx_t;

typedef struct crsfLinkStatisticsRxMsg_s
{
    crsf_header_t header;
    crsfLinkStatisticsRx_t stats;
    uint8_t crc;
} PACKED crsfLinkStatisticsRxMsg_t;

/*
 * 0x1D Link statistics TX (CRSF_FRAMETYPE_LINK_STATISTICS_TX)
 * Payload:
 *
 * uint8_t Uplink RSSI ( dBm * -1 )
 * uint8_t Uplink RSSI ( % )
 * uint8_t Uplink Package success rate / Link quality ( % )
 * int8_t Uplink SNR ( db )
 * uint8_t Downlink RF Power ( db )
 * uint8_t Uplink FPS ( FPS / 10 )
 */
typedef struct crsfPayloadLinkstatisticsTx_s {
    uint8_t uplink_RSSI;
    uint8_t uplink_RSSI_percentage;
    uint8_t uplink_Link_quality;
    int8_t  uplink_SNR;
    uint8_t downlink_power;
    uint8_t uplink_FPS;
} crsfLinkStatisticsTx_t;

typedef struct crsfLinkStatisticsTxMsg_s
{
    crsf_header_t header;
    crsfLinkStatisticsTx_t stats;
    uint8_t crc;
} PACKED crsfLinkStatisticsTxMsg_t;


typedef struct crsf_sensor_gps_s
{
    crsf_header_t header;
    int32_t latitude;
    int32_t longitude;
    uint16_t speed;
    uint16_t heading;
    uint16_t altitude;
    uint8_t satellites;
    uint8_t crc;
} PACKED crsf_sensor_gps_t;


/* MSP from radio to FC */
#define CRSF_FRAME_RX_MSP_FRAME_SIZE 8
typedef struct
{
    uint8_t flags;
    union {
        struct {
            uint8_t payloadSize;
            uint8_t function;
            uint8_t payload[CRSF_FRAME_RX_MSP_FRAME_SIZE-3];
        } hdr;
        uint8_t payload[CRSF_FRAME_RX_MSP_FRAME_SIZE-1];
    };
} PACKED mspHeaderV1_RX_t;

/* MSP from FC to radio */
#define CRSF_FRAME_TX_MSP_FRAME_SIZE 58
typedef struct
{
    uint8_t flags;
    union {
        struct {
            uint8_t payloadSize;
            uint8_t function;
            uint8_t payload[CRSF_FRAME_TX_MSP_FRAME_SIZE-4];
        } hdr;
        uint8_t payload[CRSF_FRAME_TX_MSP_FRAME_SIZE-2];
    };
    uint8_t crc_msp;
} PACKED mspHeaderV1_TX_t;

typedef struct crsf_msp_packet_fc_s
{
    crsf_ext_header_t header;
    mspHeaderV1_RX_t msp;
    uint8_t crc;
} PACKED crsf_msp_packet_fc_t;

typedef struct crsf_msp_packet_radio_s
{
    crsf_ext_header_t header;
    mspHeaderV1_TX_t msp;
    uint8_t crc;
} PACKED crsf_msp_packet_radio_t;


typedef struct {
    uint8_t command;
    uint8_t sub_command;
    uint8_t portID;
    uint32_t baudrate;
} PACKED crsf_v3_speed_control_proposal_t;

typedef struct {
    uint8_t portID;
    uint8_t found; // true / false if baud ok
} crsf_v3_speed_control_resp_t;

struct crsf_speed_req {
    crsf_ext_header_t header;
    crsf_v3_speed_control_proposal_t proposal;
    uint8_t crc; // POLY = 0xBA
};

/////inline and utility functions//////


class CRSF
{
public:
    CRSF(HwSerial *dev) : _dev(dev) {}

    void Begin();

    ///// Callbacks /////
    MspCallback_t MspCallback;
    BattInfoCallback_t BattInfoCallback;
    GpsCallback_t GpsCallback;

protected:
    uint8_t *ParseInByte(uint8_t inChar);
    virtual void LinkStatisticsSend(LinkStatsLink_t & stats) const = 0;

    HwSerial * const _dev;

    /// UART validity check ///
    uint32_t GoodPktsCount;
    uint32_t BadPktsCount;

private:
    bool CRSFframeActive;
    uint8_t crsf_cmd_ongoing;
    uint8_t SerialInCrc;
    uint8_t SerialInPacketStart;
    uint8_t SerialInPacketLen;      // length of the CRSF packet as measured
    uint8_t SerialInPacketPtr;      // index where we are reading/writing
};

#endif
