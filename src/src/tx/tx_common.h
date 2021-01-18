#pragma once

#include "rc_channels.h"
#include "msp.h"
#include <stdint.h>


//// CONSTANTS ////
#ifndef TLM_REPORT_INTERVAL
#define TLM_REPORT_INTERVAL               300u
#endif


//////////// TELEMETRY /////////
extern mspPacket_t msp_packet_tx;
extern mspPacket_t msp_packet_rx;
extern uint8_t tlm_msp_send, tlm_msp_rcvd;

extern LinkStats_t LinkStatistics;
extern GpsOta_t GpsTlm;


void tx_common_init_globals(void);
void tx_common_init(void);
void tx_common_handle_rx_buffer(void);
int  tx_common_has_telemetry(void);
int  tx_common_check_connection(void);
void tx_common_handle_ctrl_serial(void);
void tx_common_update_link_stats(void);

int8_t SettingsCommandHandle(uint8_t const *in, uint8_t *out,
                             uint8_t const inlen, uint8_t & outlen);

void hw_timer_init(void);
void hw_timer_stop(void);
