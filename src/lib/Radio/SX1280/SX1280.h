#pragma once

#include "platform.h"
#include "SX1280_Regs.h"
#include "RadioInterface.h"

// speed up to 10 MHz, SX1280 can handle up to 18MHz
#define SX128X_SPI_SPEED (10000000)

#define EFE_NO_DOUBLE 1

#if EFE_NO_DOUBLE
typedef uint32_t efe_scaler_t;
#else // !EFE_NO_DOUBLE
typedef double efe_scaler_t;
#endif // EFE_NO_DOUBLE

// SX127x compatible typedef
typedef SX1280_RadioLoRaBandwidths_t Bandwidth;
typedef SX1280_RadioLoRaSpreadingFactors_t SpreadingFactor;
typedef SX1280_RadioLoRaCodingRates_t CodingRate;

class SX1280Driver: public RadioInterface
{
public:
    /////////////Packet Stats//////////
    //uint8_t LastRadioStatus = 0;

    ////////////////Configuration Functions/////////////
    SX1280Driver(uint8_t payload_len = RX_BUFFER_LEN);
    void Begin(int sck, int miso, int mosi, int ss);
    void End(void);
    int16_t MeasureNoiseFloor(uint32_t num_meas, uint32_t freq);
    void SetMode(SX1280_RadioOperatingModes_t OPmode);
    void Config(SX1280_RadioLoRaBandwidths_t bw,
                SX1280_RadioLoRaSpreadingFactors_t sf,
                SX1280_RadioLoRaCodingRates_t cr,
                uint32_t freq, uint16_t PreambleLength,
                uint8_t crc = 0);
    void SetSyncWord(uint8_t syncWord) {};
    void SetOutputPower(int8_t power, uint8_t init=0);
    void ICACHE_RAM_ATTR SetFrequency(uint32_t freq);
    int32_t ICACHE_RAM_ATTR GetFrequencyError();
    void ICACHE_RAM_ATTR setPPMoffsetReg(int32_t error_hz, uint32_t frf = 0);

    void ICACHE_RAM_ATTR TXnb(const uint8_t *data, uint8_t length, uint32_t freq = 0);
    void ICACHE_RAM_ATTR RXnb(uint32_t freq = 0);
    void ICACHE_RAM_ATTR StopContRX(void);

    int8_t ICACHE_RAM_ATTR GetLastPacketRSSI();

    // Internal ISR callbacks, don't use these!
    void ICACHE_RAM_ATTR TXnbISR(uint16_t irqs);
    void ICACHE_RAM_ATTR RXnbISR(uint32_t rx_us, uint16_t irqs);

    uint16_t ICACHE_RAM_ATTR GetIRQFlags();
    void ICACHE_RAM_ATTR ClearIrqStatus(uint16_t irqMask);

private:
    SX1280_RadioOperatingModes_t currOpmode;
    efe_scaler_t p_efe_scaler;

    void ConfigModParams(SX1280_RadioLoRaBandwidths_t bw,
                         SX1280_RadioLoRaSpreadingFactors_t sf,
                         SX1280_RadioLoRaCodingRates_t cr);
    void SetPacketParams(SX1280_RadioLoRaPacketLengthsModes_t HeaderType,
                         SX1280_RadioLoRaCrcModes_t crc,
                         SX1280_RadioLoRaIQModes_t InvertIQ,
                         uint8_t PreambleLength,
                         uint8_t PayloadLength);
    void SetPacketType(uint8_t type);
    void SetAutoFs(uint8_t enabled);
    void SetHighSensitivityMode(uint8_t enabled);
    void SetRegulatorMode(uint8_t mode);

    void ICACHE_RAM_ATTR SetFIFOaddr(uint8_t txBaseAddr, uint8_t rxBaseAddr);
    void SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask);
    int32_t ICACHE_RAM_ATTR GetRxBufferAddr(void);
    void ICACHE_RAM_ATTR GetLastRssiSnr(void);

    ///////// SPI Interface
    void ICACHE_RAM_ATTR WriteBuffer(uint8_t offset, uint8_t *buffer, uint8_t size) const;
    void ICACHE_RAM_ATTR ReadBuffer(uint8_t offset, uint8_t *buffer, uint8_t size) const;
    void TransferBuffer(uint8_t *buffer, uint8_t size, uint8_t read) const {
        WaitOnBusy();
        transfer(buffer, size, read);
    }
};
