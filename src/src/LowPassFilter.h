#pragma once
#include "stdint.h"
#include "platform.h"

/////////// Super Simple Fixed Point Lowpass ////////////////

class LPF
{
public:
    LPF(int Beta_ = 3, int FP_Shift_ = 3)
    {
        Beta = Beta_;
        FP_Shift = FP_Shift_;
        init(0);
    }

    int32_t ICACHE_RAM_ATTR value(void)
    {
        return SmoothDataINT;
    }

    int32_t ICACHE_RAM_ATTR update(int32_t Indata)
    {
        int RawData;
        RawData = Indata;
        RawData <<= FP_Shift; // Shift to fixed point
        SmoothDataFP = (SmoothDataFP << Beta) - SmoothDataFP;
        SmoothDataFP += RawData;
        SmoothDataFP >>= Beta;
        // Don't do the following shift if you want to do further
        // calculations in fixed-point using SmoothData
        SmoothDataINT = SmoothDataFP >> FP_Shift;
        return SmoothDataINT;
    }

    void ICACHE_RAM_ATTR init(int32_t Indata)
    {
        SmoothDataINT = Indata;
        SmoothDataFP = Indata << FP_Shift;
    }

private:
    int32_t SmoothDataINT;
    int32_t SmoothDataFP;
    int Beta;     // Length = 16
    int FP_Shift; //Number of fractional bits
};
