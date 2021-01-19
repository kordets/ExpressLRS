#pragma once

#include <stdint.h>


#define NUM_ANALOGS   4

void gimbals_init(void);
void gimbals_timer_adjust(uint32_t us);
void gimbals_get(uint16_t * const out);
