#include <Arduino.h>
#include "utils.h"
#include "common.h"
#include "targets.h"
#include "POWERMGNT.h"
#include "debug_elrs.h"
#include "tx_common.h"
#include "HwTimer.h"
#include "gimbals.h"
#include "switches.h"
#include <stdlib.h>


static uint32_t DRAM_ATTR TlmSentToRadioTime;
static rc_channels_t DRAM_ATTR rc_data;
///////////////////////////////////////

static void ICACHE_RAM_ATTR
rc_data_collect(uint32_t const current_us)
{
    uint16_t gimbals[4];
    gimbals_timer_adjust(current_us);
    gimbals_get(gimbals[ANALOG_CH0], gimbals[ANALOG_CH1],
                gimbals[ANALOG_CH2], gimbals[ANALOG_CH3]);
    rc_data.ch0 = gimbals[ANALOG_CH0];
    rc_data.ch1 = gimbals[ANALOG_CH1];
    rc_data.ch2 = gimbals[ANALOG_CH2];
    rc_data.ch3 = gimbals[ANALOG_CH3];

    RcChannels_processChannels(&rc_data);
    switches_collect(&rc_data);
}

///////////////////////////////////////

void setup()
{
    tx_common_init_globals();
    platform_setup();
    DEBUG_PRINTF("ExpressLRS TX Module...\n");

    gimbals_init();

    TxTimer.callbackTockPre = rc_data_collect;
    TxTimer.updateInterval(5000);
    TxTimer.init();
    TxTimer.start();

    while (1) {
        DEBUG_PRINTF("RC: %u, %u, %u, %u -- %u, %u, %u\n",
            rc_data.ch0, rc_data.ch1, rc_data.ch2, rc_data.ch3,
            rc_data.ch4, rc_data.ch5, rc_data.ch6);
        delay(500);
    }

    tx_common_init();
}

void loop()
{
    tx_common_handle_rx_buffer();

    if (0 <= tx_common_has_telemetry())
    {
        uint32_t current_ms = millis();
        if (0 <= tx_common_check_connection() &&
            connectionState == STATE_connected &&
            TLM_REPORT_INTERVAL <= (uint32_t)(current_ms - TlmSentToRadioTime))
        {
            TlmSentToRadioTime = current_ms;
            tx_common_update_link_stats();

            // TODO: Send TLM data to CTRL_SERIAL (MSP)
        }
    }

    // Send MSP resp if allowed and packet ready
    if (tlm_msp_rcvd)
    {
        DEBUG_PRINTF("DL MSP rcvd. func: %x, size: %u\n",
            msp_packet_rx.function, msp_packet_rx.payloadSize);

        // TODO: Send received MSP packet to CTRL_SERIAL (MSP)
        // msp_packet_rx;

        msp_packet_rx.reset();
        tlm_msp_rcvd = 0;
    } else {
        tx_common_handle_ctrl_serial();
    }

    platform_loop(connectionState);
    platform_wd_feed();
}
