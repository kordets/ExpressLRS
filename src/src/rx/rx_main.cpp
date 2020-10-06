#include <Arduino.h>
#include "targets.h"
#include "utils.h"
#include "common.h"
#include "LowPassFilter.h"
#if RADIO_SX128x
#include "SX1280.h"
#else
#include "LoRa_SX127x.h"
#endif
#include "CRSF_RX.h"
#include "FHSS.h"
#include "rx_LinkQuality.h"
#include "HwTimer.h"
#include "HwSerial.h"
#include "debug_elrs.h"
#include "helpers.h"
#include "rc_channels.h"
#include "rx_servo_out.h"

static void SetRFLinkRate(uint8_t rate);
void ICACHE_RAM_ATTR LostConnection();

//// CONSTANTS ////
#define SEND_LINK_STATS_TO_FC_INTERVAL 100
#define PRINT_FREQ_ERROR               0
//#define NUM_FAILS_TO_RESYNC            100

///////////////////

#if RADIO_SX128x
SX1280Driver DRAM_FORCE_ATTR Radio(RadioSpi, OTA_PACKET_SIZE);
#else
SX127xDriver DRAM_FORCE_ATTR Radio(RadioSpi, OTA_PACKET_SIZE);
#endif
CRSF_RX DRAM_FORCE_ATTR crsf(CrsfSerial); //pass a serial port object to the class for it to use

volatile connectionState_e DRAM_ATTR connectionState;
static volatile uint8_t DRAM_ATTR NonceRXlocal; // nonce that we THINK we are up to.
static volatile uint8_t DRAM_ATTR TLMinterval;
static volatile uint32_t DRAM_ATTR tlm_check_ratio;
static volatile uint32_t DRAM_ATTR rx_last_valid_us; //Time the last valid packet was recv
static volatile int32_t DRAM_ATTR rx_freqerror;
#if NUM_FAILS_TO_RESYNC
static volatile uint32_t rx_lost_packages;
#endif
static volatile int32_t DRAM_ATTR rx_hw_isr_running;

static uint16_t DRAM_ATTR CRCCaesarCipher;

#if SERVO_OUTPUTS_ENABLED
#if !SERVO_WRITE_FROM_ISR
static volatile uint8_t DRAM_ATTR update_servos;
#endif
#endif
static EXTRACT_VOLATILE crsf_channels_t DRAM_ATTR CrsfChannels;

///////////////////////////////////////////////
////////////////  Filters  ////////////////////
static LPF DRAM_FORCE_ATTR LPF_FreqError(5);
static LPF DRAM_FORCE_ATTR LPF_UplinkRSSI(5);

//////////////////////////////////////////////////////////////
////// Variables for Telemetry and Link Quality //////////////

static volatile uint32_t DRAM_ATTR LastValidPacket; //Time the last valid packet was recv
#if !SERVO_OUTPUTS_ENABLED
static uint32_t DRAM_ATTR SendLinkStatstoFCintervalNextSend;
#endif
static mspPacket_t DRAM_FORCE_ATTR msp_packet_tx;
static volatile uint_fast8_t DRAM_ATTR tlm_msp_send;
static volatile uint_fast8_t DRAM_ATTR uplink_Link_quality;

///////////////////////////////////////////////////////////////
///////////// Variables for Sync Behaviour ////////////////////
static volatile uint32_t DRAM_ATTR RFmodeNextCycle; // set from isr
static uint32_t DRAM_ATTR RFmodeCycleDelay;
static uint8_t DRAM_ATTR scanIndex;
static uint8_t DRAM_ATTR tentative_cnt;
#if RX_UPDATE_AIR_RATE
static volatile uint32_t DRAM_ATTR updatedAirRate = 0xff;
#endif

///////////////////////////////////////
#if (DBG_PIN_TMR_ISR != UNDEF_PIN)
struct gpio_out dbg_pin_tmr;
#endif
#if (DBG_PIN_RX_ISR != UNDEF_PIN)
struct gpio_out dbg_pin_rx;
#endif

///////////////////////////////////////

static bool DRAM_ATTR ledState;
inline void led_set_state(bool state)
{
    ledState = state;
    platform_set_led(state);
}

inline void led_toggle(void)
{
    led_set_state(!ledState);
}

static void ICACHE_RAM_ATTR handle_tlm_ratio(uint8_t interval)
{
    if (TLMinterval == interval) return;

    if ((TLM_RATIO_NO_TLM < interval) && (TLM_RATIO_MAX > interval))
    {
        tlm_check_ratio = TLMratioEnumToValue(interval) - 1;
    }
    else
    {
        tlm_check_ratio = 0;
    }
    TLMinterval = interval;
    DEBUG_PRINTF("TLM: %u\n", interval);
}
///////////////////////////////////////

void ICACHE_RAM_ATTR FillLinkStats()
{
    int8_t LastRSSI = Radio.LastPacketRSSI;
    //CrsfChannels.ch15 = UINT10_to_CRSF(MAP(LastRSSI, -100, -50, 0, 1023));
    //CrsfChannels.ch14 = UINT10_to_CRSF(MAP_U16(crsf.LinkStatistics.uplink_Link_quality, 0, 100, 0, 1023));
    int32_t rssiDBM = LPF_UplinkRSSI.update(LastRSSI);
    // our rssiDBM is currently in the range -128 to 98, but BF wants a value in the range
    // 0 to 255 that maps to -1 * the negative part of the rssiDBM, so cap at 0.
    if (rssiDBM > 0)
        rssiDBM = 0;
    crsf.LinkStatistics.uplink_RSSI_1 = -1 * rssiDBM; // to match BF
    crsf.LinkStatistics.uplink_SNR = Radio.LastPacketSNR * 10;
}

uint8_t ICACHE_RAM_ATTR RadioFreqErrorCorr(void)
{
    // Do freq correction before FHSS
    /* Freq tunin ~84us */
    int32_t freqerror = rx_freqerror;
    uint8_t retval = 0;
    rx_freqerror = 0;
    if (!freqerror)
        return 0;

#if PRINT_FREQ_ERROR
    DEBUG_PRINTF(" > freqerror:%u", freqerror);
#endif

#define MAX_ERROR 0 //3000
#if MAX_ERROR
    if (freqerror < -MAX_ERROR)
        freqerror = -MAX_ERROR;
    else if (freqerror > MAX_ERROR)
        freqerror = MAX_ERROR;
#endif
    freqerror = LPF_FreqError.update(freqerror);

#if PRINT_FREQ_ERROR
    DEBUG_PRINTF(" smooth:%u", freqerror);
#endif

    if (abs(freqerror) > 100) // 120
    {
        FreqCorrection += freqerror;
        Radio.setPPMoffsetReg(freqerror, 0);
        retval = 1;
    }
#if PRINT_FREQ_ERROR
    //DEBUG_PRINTF(" local:%u", FreqCorrection);
#endif

    return retval;
}

uint8_t ICACHE_RAM_ATTR HandleFHSS()
{
    uint8_t fhss = 0, nonce = NonceRXlocal;
    if ((nonce % ExpressLRS_currAirRate->FHSShopInterval) == 0)
    {
        FHSSincCurrIndex();
        fhss = 1;
    }
    NonceRXlocal = ++nonce;
    return fhss;
}

void ICACHE_RAM_ATTR HandleSendTelemetryResponse(uint_fast8_t lq) // total ~79us
{
    DEBUG_PRINTF(" X");
    // esp requires word aligned buffer
    uint32_t __tx_buffer[(OTA_PACKET_SIZE + sizeof(uint32_t) - 1) / sizeof(uint32_t)];
    uint8_t *tx_buffer = (uint8_t *)__tx_buffer;
    uint8_t index = OTA_PACKET_DATA;

    if ((tlm_msp_send == 1) && (msp_packet_tx.type == MSP_PACKET_TLM_OTA))
    {
        if (RcChannels_tlm_ota_send(tx_buffer, msp_packet_tx, 0) || msp_packet_tx.error)
        {
            msp_packet_tx.reset();
            tlm_msp_send = 0;
        }
    }
    else if (crsf.GpsStatsPack(tx_buffer))
    {
        RcChannels_packetTypeSet(tx_buffer, DL_PACKET_GPS);
    }
    else
    {
        crsf.LinkStatistics.uplink_Link_quality = uplink_Link_quality;
        crsf.LinkStatisticsPack(tx_buffer);
        RcChannels_packetTypeSet(tx_buffer, DL_PACKET_TLM_LINK);
    }

#if OTA_PACKET_10B
    tx_buffer[index++] = 0;
    tx_buffer[index++] = 0;
#endif // OTA_PACKET_10B

    uint16_t crc = CalcCRC16(tx_buffer, index, CRCCaesarCipher);
    tx_buffer[index++] = (crc >> 8);
    tx_buffer[index++] = (crc & 0xFF);
    Radio.TXnb(tx_buffer, index, FHSSgetCurrFreq());
}

void tx_done_cb(void)
{
    // Configure RX only next is not hopping time
    if (((NonceRXlocal + 1) % ExpressLRS_currAirRate->FHSShopInterval) != 0)
        Radio.RXnb(FHSSgetCurrFreq());
}

void ICACHE_RAM_ATTR HWtimerCallback(uint32_t us)
{
    //DEBUG_PRINTF("H");
#if (DBG_PIN_TMR_ISR != UNDEF_PIN)
    gpio_out_write(dbg_pin_tmr, 1);
#endif
    rx_hw_isr_running = 1;
    uint_fast8_t fhss_config_rx = 0, tlm_send, tlm_ratio = tlm_check_ratio;
    uint32_t __rx_last_valid_us = rx_last_valid_us;
    rx_last_valid_us = 0;

#if PRINT_TIMER && PRINT_HW_ISR
    DEBUG_PRINTF("HW us %u", us);
#endif

    /* do adjustment */
    if (unlikely(__rx_last_valid_us != 0))
    {
#if !USE_TIMER_KICK
        int32_t diff_us = (int32_t)(us - __rx_last_valid_us);
#if PRINT_TIMER && PRINT_HW_ISR
        DEBUG_PRINTF(" - rx %u = %u", __rx_last_valid_us, diff_us);
#endif

        int32_t const interval = ExpressLRS_currAirRate->interval;
        if (diff_us > (interval >> 1))
        {
            /* the timer was called too early */
            diff_us %= interval;
            diff_us -= interval;
        }

        /* Adjust the timer */
        if ((TIMER_OFFSET_LIMIT < diff_us) || (diff_us < 0))
            TxTimer.reset(diff_us - TIMER_OFFSET);
#else // USE_TIMER_KICK
        //TxTimer.setTime(interval+200 /*TIMER_OFFSET*/);
        TxTimer.reset(-TIMER_OFFSET);
#endif // USE_TIMER_KICK
    }
    else
    {
#if !USE_TIMER_KICK
        TxTimer.setTime(); // Reset timer interval
#else
        TxTimer.reset(0); // Reset timer interval
#endif
    }

    fhss_config_rx |= RadioFreqErrorCorr();
    fhss_config_rx |= HandleFHSS();

#if PRINT_TIMER && PRINT_HW_ISR
    //DEBUG_PRINTF(" nonce %u", NonceRXlocal);
#endif

    tlm_send = ((0 < tlm_ratio) && ((NonceRXlocal & tlm_ratio) == 0));
    uint_fast8_t lq = LQ_getlinkQuality();
    uplink_Link_quality = lq;
    LQ_nextPacket();

    if (tlm_send)
    {
#if (DBG_PIN_TMR_ISR != UNDEF_PIN)
    gpio_out_write(dbg_pin_tmr, 0);
#endif
        // Adds packet to LQ otherwise an artificial drop in LQ is seen due to sending TLM.
        LQ_packetAck();

        HandleSendTelemetryResponse(lq);
#if (DBG_PIN_TMR_ISR != UNDEF_PIN)
        gpio_out_write(dbg_pin_tmr, 1);
#endif
        fhss_config_rx = 0;
    }
#if NUM_FAILS_TO_RESYNC
    else if (!__rx_last_valid_us)
    {
        if (NUM_FAILS_TO_RESYNC < (++rx_lost_packages)) {
            // consecutive losts => trigger connection lost to resync
            LostConnection();
            DEBUG_PRINTF("RESYNC!");
        }
    }
#endif

    if (fhss_config_rx) {
        Radio.RXnb(FHSSgetCurrFreq()); // 260us => 148us => ~67us
    }

#if (PRINT_TIMER && PRINT_HW_ISR) || PRINT_FREQ_ERROR
    DEBUG_PRINTF(" took %u\n", (micros() - us));
#endif
    rx_hw_isr_running = 0;
#if (DBG_PIN_TMR_ISR != UNDEF_PIN)
    gpio_out_write(dbg_pin_tmr, 0);
#endif
}

void ICACHE_RAM_ATTR LostConnection()
{
    if (connectionState <= STATE_lost)
    {
        return; // Already disconnected
    }

#if SERVO_OUTPUTS_ENABLED
    servo_out_fail_safe();
#endif

    uint32_t irq = _SAVE_IRQ();
    TxTimer.stop(); // Stop sync timer
    _RESTORE_IRQ(irq);

    // Reset FHSS
    FHSSresetFreqCorrection();
    FHSSsetCurrIndex(0);
    LPF_FreqError.init(0);

    connectionState = STATE_lost; //set lost connection

    led_set_state(1);             // turn off led
    Radio.RXnb(GetInitialFreq()); // in conn lost state we always want to listen on freq index 0
    DEBUG_PRINTF("lost conn\n");

    platform_connection_state(connectionState);
    RFmodeNextCycle = millis(); // make sure to stay on same freq during next sync search phase
}

void ICACHE_RAM_ATTR TentativeConnection(int32_t freqerror)
{
    /* Do initial freq correction */
    FreqCorrection += freqerror;
    Radio.setPPMoffsetReg(freqerror, 0);
    LPF_FreqError.init(freqerror);
    rx_freqerror = 0;
    rx_last_valid_us = 0;

    tentative_cnt = 0;
    connectionState = STATE_tentative;
    DEBUG_PRINTF("tentative\n");
    TxTimer.start();     // Start local sync timer
#if !USE_TIMER_KICK
    TxTimer.setTime(80); // Trigger isr right after reception
#endif
    led_set_state(0); // turn on led
}

void ICACHE_RAM_ATTR GotConnection()
{
    connectionState = STATE_connected; //we got a packet, therefore no lost connection

    //led_set_state(0); // turn on led
    DEBUG_PRINTF("connected\n");

    platform_connection_state(connectionState);
}

void ICACHE_RAM_ATTR ProcessRFPacketCallback(uint8_t *rx_buffer, const uint32_t current_us)
{
    /* Processing takes:
        R9MM: ~160us
    */
    if (rx_hw_isr_running)
        // Skip if hw isr is triggered already (e.g. TX has some weird latency)
        return;

#if (DBG_PIN_RX_ISR != UNDEF_PIN)
    gpio_out_write(dbg_pin_rx, 1);
#endif

    //DEBUG_PRINTF("I");
    const connectionState_e _conn_state = connectionState;
    const uint16_t crc = CalcCRC16(rx_buffer, OTA_PACKET_PAYLOAD, CRCCaesarCipher);
    const uint16_t crc_in = ((uint16_t)rx_buffer[OTA_PACKET_PAYLOAD] << 8) + rx_buffer[OTA_PACKET_PAYLOAD+1];
    const uint8_t type = RcChannels_packetTypeGet(rx_buffer);

#if PRINT_TIMER && PRINT_RX_ISR
    DEBUG_PRINTF("RX us %u", current_us);
#endif

    if (crc_in != crc)
    {
#if (DBG_PIN_RX_ISR != UNDEF_PIN)
        gpio_out_write(dbg_pin_rx, 0);
#endif
        DEBUG_PRINTF(" !");
        return;
    }

    rx_freqerror = Radio.GetFrequencyError();

    rx_last_valid_us = current_us;
    LastValidPacket = millis();

    switch (type)
    {
        case UL_PACKET_SYNC:
        {
            DEBUG_PRINTF(" S");
            ElrsSyncPacket_s const * const sync = (ElrsSyncPacket_s*)rx_buffer;

            if (sync->CRCCaesarCipher == CRCCaesarCipher)
            {
                if (_conn_state == STATE_disconnected || _conn_state == STATE_lost)
                {
                    TentativeConnection(rx_freqerror);
                }
                else if (_conn_state == STATE_tentative)
                {
                    if (NonceRXlocal == sync->rxtx_counter &&
                        FHSSgetCurrIndex() == sync->fhssIndex)
                    {
                        GotConnection();
                    }
                    else if (2 < (tentative_cnt++))
                    {
                        LostConnection();
                    }
                }

#if RX_UPDATE_AIR_RATE
                if (current_rate_config != sync->air_rate)
                {
                    updatedAirRate = sync->air_rate;
                }
#endif

                handle_tlm_ratio(sync->tlm_interval);
                FHSSsetCurrIndex(sync->fhssIndex);
                NonceRXlocal = sync->rxtx_counter;
            }
            break;
        }
        case UL_PACKET_RC_DATA: //Standard RC Data Packet
            DEBUG_PRINTF(" R");
            if (STATE_lost < _conn_state)
            {
                RcChannels_channels_extract(rx_buffer, CrsfChannels);
#if SERVO_OUTPUTS_ENABLED
#if SERVO_WRITE_FROM_ISR
                servo_out_write(&CrsfChannels);
#else
                update_servos = 1;
#endif
#else // !SERVO_OUTPUTS_ENABLED
#if (DBG_PIN_RX_ISR != UNDEF_PIN)
                gpio_out_write(dbg_pin_rx, 0);
#endif
                crsf.sendRCFrameToFC(&CrsfChannels);
#if (DBG_PIN_RX_ISR != UNDEF_PIN)
                gpio_out_write(dbg_pin_rx, 1);
#endif
#endif // SERVO_OUTPUTS_ENABLED
            }
            break;

        case UL_PACKET_MSP: {
#if !SERVO_OUTPUTS_ENABLED
            DEBUG_PRINTF(" M");
            uint8_t len = RcChannels_tlm_uplink_receive(rx_buffer);
            crsf.sendMSPFrameToFC(rx_buffer, len);
#endif
            break;
        }

        case UL_PACKET_UNKNOWN:
        default:
            /* Not a valid packet, ignore it */
            rx_last_valid_us = 0;
            rx_freqerror = 0;
            return;
            //break;
    }

#if OTA_PACKET_10B
    if (STATE_lost < _conn_state) {
        // Received in every packet to maintain sync all the time
        FHSSsetCurrIndex(rx_buffer[OTA_PACKET_DATA]);
        NonceRXlocal = rx_buffer[OTA_PACKET_DATA+1];
    }
#endif // OTA_PACKET_10B

#if NUM_FAILS_TO_RESYNC
    rx_lost_packages = 0;
#endif
    LQ_packetAck();
    FillLinkStats();

#if PRINT_TIMER && PRINT_RX_ISR
    DEBUG_PRINTF(" took %u\n", (micros() - current_us));
#endif
#if (DBG_PIN_RX_ISR != UNDEF_PIN)
    gpio_out_write(dbg_pin_rx, 0);
#endif

#if USE_TIMER_KICK
    TxTimer.triggerSoon(); // Trigger FHSS ISR
#endif
}

void forced_stop(void)
{
    uint32_t irq = _SAVE_IRQ();
    TxTimer.stop();
    Radio.StopContRX();
    _RESTORE_IRQ(irq);
}

static void SetRFLinkRate(uint8_t rate) // Set speed of RF link (hz)
{
    const expresslrs_mod_settings_t *const config = get_elrs_airRateConfig(rate);
    if (config == NULL || config == ExpressLRS_currAirRate)
        return; // No need to modify, rate is same

    // Stop ongoing actions before configuring next rate
    forced_stop();

    ExpressLRS_currAirRate = config;
    current_rate_config = rate;

    DEBUG_PRINTF("Set RF rate: %u\n", config->rate);

    // Init CRC aka LQ array
    LQ_reset();
    // Reset FHSS
    FHSSresetFreqCorrection();
    FHSSsetCurrIndex(0);

    RFmodeCycleDelay = config->syncSearchTimeout +
                       config->connectionLostTimeout;

    handle_tlm_ratio(config->TLMinterval);

    Radio.Config(config->bw, config->sf, config->cr, GetInitialFreq(),
                 config->PreambleLen, (OTA_PACKET_CRC == 0));

    // Measure RF noise
#ifdef DEBUG_SERIAL // TODO: Enable this when noize floor is used!
    int RFnoiseFloor = Radio.MeasureNoiseFloor(10, GetInitialFreq());
    DEBUG_PRINTF("RF noise floor: %d dBm\n", RFnoiseFloor);
    (void)RFnoiseFloor;
#endif

    TxTimer.updateInterval(config->interval);
    LPF_FreqError.init(0);
    LPF_UplinkRSSI.init(0);
#if !SERVO_OUTPUTS_ENABLED
    crsf.LinkStatistics.uplink_RSSI_2 = 0;
    crsf.LinkStatistics.rf_Mode = config->rate_osd_num;
#endif
    //TxTimer.setTime();
    Radio.RXnb();
}

/* FC sends v1 MSPs */
void msp_data_cb(uint8_t const *const input)
{
    if ((tlm_msp_send != 0) || (tlm_check_ratio == 0))
        return;

    /* process MSP packet from flight controller
     *  [0] header: seq&0xF,
     *  [1] payload size
     *  [2] function
     *  [3...] payload + crc
     */
    mspHeaderV1_t *hdr = (mspHeaderV1_t *)input;

    if (sizeof(msp_packet_tx.payload) < (hdr->payloadSize + 3U))
        /* too big, ignore */
        return;

    msp_packet_tx.reset(hdr);
    msp_packet_tx.type = MSP_PACKET_TLM_OTA;
    msp_packet_tx.payloadSize = hdr->payloadSize + 3;

    volatile_memcpy(msp_packet_tx.payload,
                    (void*)&input[1],       // skip flags
                    hdr->payloadSize + 3);  // include size, func and crc

    tlm_msp_send = 1; // rdy for sending
}

void setup()
{
    uint8_t UID[6] = {MY_UID};

#if (DBG_PIN_TMR_ISR != UNDEF_PIN)
    dbg_pin_tmr = gpio_out_setup(DBG_PIN_TMR_ISR, 0);
#endif
#if (DBG_PIN_RX_ISR != UNDEF_PIN)
    dbg_pin_rx = gpio_out_setup(DBG_PIN_RX_ISR, 0);
#endif

    connectionState = STATE_disconnected;
    tentative_cnt = 0;
    CRCCaesarCipher = CalcCRC16(UID, sizeof(UID), 0);

#if !SERVO_OUTPUTS_ENABLED
    CrsfSerial.Begin(CRSF_RX_BAUDRATE);
#endif

    msp_packet_tx.reset();

    DEBUG_PRINTF("ExpressLRS RX Module...\n");
    platform_setup();

    //FHSSrandomiseFHSSsequence();

    // Prepare radio
#if defined(Regulatory_Domain_AU_433) || defined(Regulatory_Domain_EU_433)
    Radio.RFmodule = RFMOD_SX1278;
#elif !RADIO_SX128x
    Radio.RFmodule = RFMOD_SX1276;
#endif
    Radio.RXdoneCallback1 = ProcessRFPacketCallback;
    Radio.TXdoneCallback1 = tx_done_cb;
    Radio.SetPins(GPIO_PIN_RST, GPIO_PIN_DIO0, GPIO_PIN_DIO1, GPIO_PIN_DIO2,
                  GPIO_PIN_BUSY, GPIO_PIN_TX_ENABLE, GPIO_PIN_RX_ENABLE);
    Radio.SetSyncWord(getSyncWord());
    Radio.Begin(GPIO_PIN_SCK, GPIO_PIN_MISO, GPIO_PIN_MOSI, GPIO_PIN_NSS);
    Radio.SetOutputPower(0b1111); // default RX to max power for tlm

    // Set call back for timer ISR
    TxTimer.callbackTock = &HWtimerCallback;
    TxTimer.init();
    // Init first scan index
    scanIndex = RATE_DEFAULT;
#if SERVO_OUTPUTS_ENABLED
    servo_out_init();
#else
    // Initialize CRSF protocol handler
    crsf.MspCallback = msp_data_cb;
    crsf.Begin();
#endif
}

static uint32_t led_toggle_ms = 0;
void loop()
{
    uint32_t now = millis();
    const connectionState_e _conn_state = connectionState;
#if !SERVO_OUTPUTS_ENABLED
    uint8_t rx_buffer_handle = 0;
#endif

#if RX_UPDATE_AIR_RATE
    /* update air rate config, timed to FHSS index 0 */
    if (updatedAirRate < get_elrs_airRateMax() && updatedAirRate != current_rate_config)
    {
        //connectionState = STATE_disconnected; // Force resync
        connectionState = STATE_lost; // Mark to lost to stay on received rate and force resync.
        SetRFLinkRate(updatedAirRate); // configure air rate
        RFmodeNextCycle = now;
        return;
    }
#endif /* RX_UPDATE_AIR_RATE */

    if (STATE_lost < _conn_state)
    {
        // check if connection is lost
        if (ExpressLRS_currAirRate->connectionLostTimeout <= (int32_t)(now - LastValidPacket))
        {
            LostConnection();
        }
        else if (_conn_state == STATE_connected)
        {
#if SERVO_OUTPUTS_ENABLED
#if !SERVO_WRITE_FROM_ISR
            if (update_servos)
                servo_out_write(&CrsfChannels);
#endif
#else
            if (SEND_LINK_STATS_TO_FC_INTERVAL <= (uint32_t)(now - SendLinkStatstoFCintervalNextSend))
            {
                crsf.LinkStatistics.uplink_Link_quality = uplink_Link_quality;
                crsf.LinkStatisticsSend();
                SendLinkStatstoFCintervalNextSend = now;
            }
#endif
        }
    }
    else if (_conn_state == STATE_disconnected)
    { /* Cycle only if initial connection search */

        if (RFmodeCycleDelay < (uint32_t)(now - RFmodeNextCycle))
        {
            uint8_t max_rate = get_elrs_airRateMax();
            SetRFLinkRate((scanIndex % max_rate)); //switch between rates
            scanIndex++;
            if (max_rate <= scanIndex)
                platform_connection_state(STATE_search_iteration_done);

            RFmodeNextCycle = now;
        }
        else if (150 <= (uint32_t)(now - led_toggle_ms))
        {
            led_toggle();
            led_toggle_ms = now;
        }
    }
    else if (_conn_state == STATE_lost)
    { /* Just blink a led if connections is lost */
        if (300 <= (uint32_t)(now - led_toggle_ms))
        {
            led_toggle();
            led_toggle_ms = now;
        }
    }

#if !SERVO_OUTPUTS_ENABLED
    crsf.handleUartIn(rx_buffer_handle);
#endif

    platform_loop(_conn_state);

    platform_wd_feed();
}
