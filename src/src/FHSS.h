#ifndef __FHSS_H
#define __FHSS_H

#include "platform.h"
#include <stdint.h>

#define FreqCorrectionMax 100000
#define FreqCorrectionMin -100000
#define FreqCorrectionStep 61 //min freq step is ~ 61hz

void FHSSfreqCorrectionReset(void);
void FHSSfreqCorrectionSet(int32_t error);

void FAST_CODE_1 FHSS_init(uint8_t mode);

void FAST_CODE_1 FHSSsetCurrIndex(uint32_t value);
uint32_t FAST_CODE_1 FHSSgetCurrIndex();
void FAST_CODE_1 FHSSincCurrIndex();
uint8_t FAST_CODE_1 FHSSgetCurrSequenceIndex();

uint32_t FAST_CODE_1 GetInitialFreq();
uint32_t FAST_CODE_1 FHSSgetCurrFreq();
uint32_t FAST_CODE_1 FHSSgetNextFreq();

void FHSSrandomiseFHSSsequence(uint8_t mode);

#endif // __FHSS_H
