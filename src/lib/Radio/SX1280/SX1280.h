#pragma once

#include "platform.h"
#include "SX1280_Regs.h"
#include "RadioInterface.h"


#define SX128X_SPI_SPEED (18000000) // speed up to 20 MHz

// SX127x compatible typedef
typedef SX1280_RadioLoRaBandwidths_t Bandwidth;
typedef SX1280_RadioLoRaSpreadingFactors_t SpreadingFactor;
typedef SX1280_RadioLoRaCodingRates_t CodingRate;

class SX1280Driver: public RadioInterface
{
public:
    ///////////Radio Variables////////
    SX1280_RadioLoRaBandwidths_t currBW = SX1280_LORA_BW_0800;
    SX1280_RadioLoRaSpreadingFactors_t currSF = SX1280_LORA_SF6;
    SX1280_RadioLoRaCodingRates_t currCR = SX1280_LORA_CR_4_7;
    uint32_t currFreq = 2400000000;

    /////////////Packet Stats//////////
    volatile uint32_t LastPacketIsrMicros = 0;
    volatile int16_t LastPacketRSSI;
    volatile uint8_t LastPacketRssiRaw;
    volatile int8_t LastPacketSNR;
    volatile uint8_t LastRadioStatus = 0;
    volatile uint8_t NonceTX = 0;
    volatile uint8_t NonceRX = 0;

    ////////////////Configuration Functions/////////////
    SX1280Driver(HwSpi &spi);
    void Begin();
    void End();
    void SetMode(SX1280_RadioOperatingModes_t OPmode);
    void Config(SX1280_RadioLoRaBandwidths_t bw, SX1280_RadioLoRaSpreadingFactors_t sf, SX1280_RadioLoRaCodingRates_t cr, uint32_t freq, uint8_t PreambleLength);
    void ConfigModParams(SX1280_RadioLoRaBandwidths_t bw, SX1280_RadioLoRaSpreadingFactors_t sf, SX1280_RadioLoRaCodingRates_t cr);
    void SetPacketParams(uint8_t PreambleLength, SX1280_RadioLoRaPacketLengthsModes_t HeaderType, uint8_t PayloadLength, SX1280_RadioLoRaCrcModes_t crc, SX1280_RadioLoRaIQModes_t InvertIQ);
    void ICACHE_RAM_ATTR SetFrequency(uint32_t freq);
    void SetOutputPower(int8_t power);

    void SetPreambleLength(uint16_t PreambleLen) {};
    void SetSyncWord(uint8_t syncWord) {};

    int32_t ICACHE_RAM_ATTR GetFrequencyError();
    void ICACHE_RAM_ATTR setPPMoffsetReg(int32_t error_hz, uint32_t frf = 0);

    void ICACHE_RAM_ATTR TXnb(const uint8_t *data, uint8_t length, uint32_t freq = 0);
    void ICACHE_RAM_ATTR RXnb(uint32_t freq = 0);
    void ICACHE_RAM_ATTR StopContRX() {}

    int8_t ICACHE_RAM_ATTR GetLastPacketRSSI();

    // Internal ISR callbacks, don't use these!
    void ICACHE_RAM_ATTR TXnbISR();
    void ICACHE_RAM_ATTR RXnbISR();

private:
    void ICACHE_RAM_ATTR SetFIFOaddr(uint8_t txBaseAddr, uint8_t rxBaseAddr);
    uint16_t ICACHE_RAM_ATTR GetIRQFlags();
    void ICACHE_RAM_ATTR ClearIrqStatus(uint16_t irqMask);
    void SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask);
    uint8_t ICACHE_RAM_ATTR GetRxBufferAddr();
    void ICACHE_RAM_ATTR GetLastRssiSnr(void);

    void ICACHE_RAM_ATTR WriteCommand(SX1280_RadioCommands_t command, uint8_t val) {
        WriteCommand(command, &val, 1);
    }
    void ICACHE_RAM_ATTR WriteCommand(SX1280_RadioCommands_t opcode, uint8_t *buffer, uint8_t size);
    void ICACHE_RAM_ATTR ReadCommand(SX1280_RadioCommands_t opcode, uint8_t *buffer, uint8_t size);

    void ICACHE_RAM_ATTR WriteRegister(uint16_t address, uint8_t value) {
        SX1280Driver::WriteRegister(address, &value, 1);
    }
    void ICACHE_RAM_ATTR WriteRegister(uint16_t address, uint8_t *buffer, uint8_t size);
    uint8_t ICACHE_RAM_ATTR ReadRegister(uint16_t address) {
        uint8_t data;
        SX1280Driver::ReadRegister(address, &data, 1);
        return data;
    }
    void ICACHE_RAM_ATTR ReadRegister(uint16_t address, uint8_t *buffer, uint8_t size);

    void ICACHE_RAM_ATTR WriteBuffer(uint8_t offset, uint8_t *buffer, uint8_t size); // Writes and Reads to FIFO
    void ICACHE_RAM_ATTR ReadBuffer(uint8_t offset, uint8_t *buffer, uint8_t size);
};

#if RADIO_SX128x
extern SX1280Driver Radio;
typedef SX1280Driver SXRadioDriver;
#endif
