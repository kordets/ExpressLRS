#pragma once

#include <stdint.h>

void gimbals_init(void);
void gimbals_timer_adjust(int32_t off);
void gimbals_get(uint16_t &l1, uint16_t &l2, uint16_t &r1, uint16_t &r2);