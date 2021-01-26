#pragma once

#include <stdint.h>

#define TX_NUM_ANALOGS  4
#define TX_NUM_MIXER    16

enum {
    GIMBAL_IDX_L1 = 0,  // L1
    GIMBAL_IDX_L2,      // L2
    GIMBAL_IDX_R1,      // R1
    GIMBAL_IDX_R2,      // R2
    GIMBAL_IDX_MAX
};

enum {
    GIMBAL_CALIB_THR = 0x10,
    GIMBAL_CALIB_YAW,
    GIMBAL_CALIB_PITCH,
    GIMBAL_CALIB_ROLL,
};

struct gimbal_limit {
    uint16_t low;
    uint16_t mid;
    uint16_t high;
};

struct mixer {
    uint8_t index;
    uint8_t inv;
};
