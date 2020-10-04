#include "SX1280.h"
#include "debug_elrs.h"

/////////////////////////////////////////////////////////////////

static SX1280Driver * instance = NULL;

static void ICACHE_RAM_ATTR _rxtx_isr_handler(void)
{
    uint16_t irqs;
    instance->LastPacketIsrMicros = micros();
    irqs = instance->GetIRQFlags();
    instance->ClearIrqStatus(SX1280_IRQ_RADIO_ALL);

    switch (RadioInterface::p_state_isr) {
        case RX_DONE:
            instance->RXnbISR(irqs);
            break;
        case TX_DONE:
            instance->TXnbISR(irqs);
            break;
        default:
            break;
    };
}

/////////////////////////////////////////////////////////////////

SX1280Driver::SX1280Driver(HwSpi &spi, uint8_t payload_len):
    RadioInterface(spi)
{
    instance = this;
    current_freq = 0; //2400000000;
    current_power = -100;
    RX_buffer_size = payload_len;
}

void SX1280Driver::Begin(void)
{
    RadioInterface::InitPins();
    // initialize low-level drivers
    RadioHalSpi::Begin(SX128X_SPI_SPEED);

    Reset();
    delay(100);

    uint8_t buffer[2] = {0, 0};
    ReadRegister(REG_LR_FIRMWARE_VERSION_MSB, buffer, sizeof(buffer));
    uint16_t firmwareRev = buffer[0];
    firmwareRev <<= 8;
    firmwareRev += buffer[1];
    DEBUG_PRINTF("SX1280 fw rev %u\n", firmwareRev);

    //attachInterrupt(digitalPinToInterrupt(_BUSY), this->busyISR, CHANGE); //not used atm
    attachInterrupt(digitalPinToInterrupt(_DIO1), _rxtx_isr_handler, RISING);
}

void SX1280Driver::End(void)
{
    StopContRX();
    if (-1 < _DIO1)
        detachInterrupt(_DIO1);
}

int16_t SX1280Driver::MeasureNoiseFloor(uint32_t num_meas, uint32_t freq)
{
    return -999;
}

void SX1280Driver::Config(SX1280_RadioLoRaBandwidths_t bw,
                          SX1280_RadioLoRaSpreadingFactors_t sf,
                          SX1280_RadioLoRaCodingRates_t cr,
                          uint32_t freq, uint16_t PreambleLength,
                          uint8_t crc)
{
    uint16_t irqs = (SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE | SX1280_IRQ_RX_TX_TIMEOUT);
    //Reset(); // ????
    //SetMode(SX1280_MODE_STDBY_XOSC);
    SetMode(SX1280_MODE_STDBY_RC);
    WriteCommand(SX1280_RADIO_SET_PACKETTYPE, SX1280_PACKET_TYPE_LORA);
    SetFrequency(freq);
    //SetFIFOaddr(0x00, 0x00);
    ConfigModParams(bw, sf, cr);
    WriteCommand(SX1280_RADIO_SET_AUTOFS, 0x01); //enable auto FS
    SetPacketParams(SX1280_LORA_PACKET_IMPLICIT,
                    (crc) ? SX1280_LORA_CRC_ON : SX1280_LORA_CRC_OFF,
                    SX1280_LORA_IQ_NORMAL,
                    PreambleLength, RX_buffer_size);
    if (crc)
        irqs |= SX1280_IRQ_CRC_ERROR;
    // Config IRQs
    SetDioIrqParams(SX1280_IRQ_RADIO_ALL,
                    irqs,
                    SX1280_IRQ_RADIO_NONE,
                    SX1280_IRQ_RADIO_NONE);
    ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
}

void ICACHE_RAM_ATTR SX1280Driver::SetOutputPower(int8_t power)
{
    power += 18;
    if (power < 0) power = 0;           //  0 = -18dBm
    else if (power > 31) power = 31;    // 31 = +13dBm

    // Skip if already set
    if (power == current_power)
        return;

    uint8_t buf[] = {(uint8_t)(power), SX1280_RADIO_RAMP_04_US};
    WriteCommand(SX1280_RADIO_SET_TXPARAMS, buf, sizeof(buf));
    //DEBUG_PRINTF("SetOutputPower: %d", (power - 18));
    current_power = power;
}

void SX1280Driver::SetPacketParams(SX1280_RadioLoRaPacketLengthsModes_t HeaderType,
                                   SX1280_RadioLoRaCrcModes_t crc,
                                   SX1280_RadioLoRaIQModes_t InvertIQ,
                                   uint8_t PreambleLength,
                                   uint8_t PayloadLength)
{
    uint8_t buf[7];

    buf[0] = PreambleLength;
    buf[1] = HeaderType;
    buf[2] = PayloadLength;
    buf[3] = crc;
    buf[4] = InvertIQ;
    buf[5] = 0x00;
    buf[6] = 0x00;

    WriteCommand(SX1280_RADIO_SET_PACKETPARAMS, buf, sizeof(buf));
}

void SX1280Driver::SetMode(SX1280_RadioOperatingModes_t OPmode)
{
    WORD_ALIGNED_ATTR uint8_t buffer[3]; //TODO make word alignmed

    if (OPmode == currOpmode) {
       return;
    }

    switch (OPmode)
    {

    case SX1280_MODE_SLEEP:
        WriteCommand(SX1280_RADIO_SET_SLEEP, 0x01);
        break;

    case SX1280_MODE_CALIBRATION:
        break;

    case SX1280_MODE_STDBY_RC:
        WriteCommand(SX1280_RADIO_SET_STANDBY, SX1280_STDBY_RC);
        break;
    case SX1280_MODE_STDBY_XOSC:
        WriteCommand(SX1280_RADIO_SET_STANDBY, SX1280_STDBY_XOSC);
        break;

    case SX1280_MODE_FS:
        WriteCommand(SX1280_RADIO_SET_FS, 0x00);
        break;

    case SX1280_MODE_RX:
        //uses timeout Time-out duration = periodBase * periodBaseCount
        buffer[0] = 0x02; // periodBase: 0x2 = 1ms, page 71 datasheet
        buffer[1] = 0xFF; // periodBaseCount: 0xffff = Rx Continuous mode.
        buffer[2] = 0xFF;
        WriteCommand(SX1280_RADIO_SET_RX, buffer, sizeof(buffer));
        break;

    case SX1280_MODE_TX:
        //uses timeout Time-out duration = periodBase * periodBaseCount
        buffer[0] = 0x02; // periodBase: 0x2 = 1ms, page 71 datasheet
        buffer[1] = 0x00; // periodBaseCount: 0x0 = no timeout, returns when TX is ready
        buffer[2] = 0x00;
        WriteCommand(SX1280_RADIO_SET_TX, buffer, sizeof(buffer));
        break;

    case SX1280_MODE_CAD:
        break;

    default:
        break;
    }

    currOpmode = OPmode;
}

void SX1280Driver::ConfigModParams(SX1280_RadioLoRaBandwidths_t bw,
                                   SX1280_RadioLoRaSpreadingFactors_t sf,
                                   SX1280_RadioLoRaCodingRates_t cr)
{
    // Care must therefore be taken to ensure that modulation parameters are set using the command
    // SetModulationParam() only after defining the packet type SetPacketType() to be used

    uint8_t rfparams[3] = {(uint8_t)sf, (uint8_t)bw, (uint8_t)cr};
    WriteCommand(SX1280_RADIO_SET_MODULATIONPARAMS, rfparams, sizeof(rfparams));

    switch (sf)
    {
    case SX1280_LORA_SF5:
    case SX1280_LORA_SF6:
        WriteRegister(0x925, 0x1E);
        break;
    case SX1280_LORA_SF7:
    case SX1280_LORA_SF8:
        WriteRegister(0x925, 0x37);
        break;
    default:
        WriteRegister(0x925, 0x32);
        break;
    }

    currBW = bw;
    currSF = sf;
    currCR = cr;
}

void ICACHE_RAM_ATTR SX1280Driver::SetFrequency(uint32_t Reqfreq)
{
    // Skip if already set
    if (current_freq == Reqfreq) return;

#if 1
    uint32_t freq = (uint32_t)((double)Reqfreq / (double)SX1280_FREQ_STEP);
#else
    // 1024 * x / 203125
    uint64_t freq = Reqfreq;
    freq <<= 10; // * 1024
    freq /= 203125;
#endif
    uint8_t buf[3];
    buf[0] = (uint8_t)((freq >> 16) & 0xFF);
    buf[1] = (uint8_t)((freq >> 8) & 0xFF);
    buf[2] = (uint8_t)(freq & 0xFF);

    WriteCommand(SX1280_RADIO_SET_RFFREQUENCY, buf, sizeof(buf));
    current_freq = Reqfreq;
}

int32_t ICACHE_RAM_ATTR SX1280Driver::GetFrequencyError()
{
#define EFE_NO_DOUBLE 1

#if EFE_NO_DOUBLE
#define EFE_USE_32b 1
#if EFE_USE_32b
    int32_t efe;
#else
    int64_t efe;
#endif
#else
    double efeHz;
    int32_t efe = 0;
#endif
    uint8_t fei_reg[3] = {0x0, 0x0, 0x0};

    ReadRegister(SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB, fei_reg, sizeof(fei_reg));
    efe = fei_reg[0] & 0b0111;
    efe <<= 8;
    efe += fei_reg[1];
    efe <<= 8;
    efe += fei_reg[2];

     // Check the sign bit
    if (fei_reg[0] & 0b1000) {
        // convert to negative
        efe -= 524288;
    }

#if EFE_NO_DOUBLE
#if EFE_USE_32b
    /* This cause a bit of Hz error which does not effect to functionality
     *   SX1280 can handle 203kHz (BW 0.8MHz) or even 406kHz (BW 1.6MHz)
     *   frequency error.
     */
    switch (currBW) {
        case SX1280_LORA_BW_0200:
            efe *= 315;
            break;
        case SX1280_LORA_BW_0400:
            efe *= 630;
            break;
        case SX1280_LORA_BW_0800:
            efe *= 1259;
            break;
        case SX1280_LORA_BW_1600:
            efe *= 2519; // max value fits just to int32_t
            break;
    }
    return (efe / 1600);

#else // EFE_USE_32b
    switch (currBW) {
        case SX1280_LORA_BW_0200:
            efe *= 19677734375;
            return (int32_t)(efe / 100000000000);
            break;
        case SX1280_LORA_BW_0400:
            efe *= 3935546875;
            return (int32_t)(efe / 10000000000);
            break;
        case SX1280_LORA_BW_0800:
            efe *= 787109375;
            return (int32_t)(efe / 1000000000);
            break;
        case SX1280_LORA_BW_1600:
            efe *= 157421875;
            return (int32_t)(efe / 100000000);
            break;
    }
    return 0;
#endif // EFE_USE_32b

#else // EFE_NO_DOUBLE
    efeHz = efe;
    //efeHz *= 1.55;
    switch (currBW) {
        case SX1280_LORA_BW_0200:
            //efeHz *= 203.125;
            efeHz *= 314.84375; // 203.125 * 1.55
            break;
        case SX1280_LORA_BW_0400:
            //efeHz *= 406.25;
            efeHz *= 629.6875; // 406.25 * 1.55
            break;
        case SX1280_LORA_BW_0800:
            //efeHz *= 812.5;
            efeHz *= 1259.375; // 812.5 * 1.55
            break;
        case SX1280_LORA_BW_1600:
            //efeHz *= 1625.0;
            efeHz *= 2518.75; // 1625.0 * 1.55
            break;
    }
    efeHz /= 1600;
    return (int32_t)efeHz;
#endif // EFE_NO_DOUBLE
}

void ICACHE_RAM_ATTR SX1280Driver::setPPMoffsetReg(int32_t error_hz, uint32_t frf)
{
    // Apply freq error correction
}

void ICACHE_RAM_ATTR SX1280Driver::TXnbISR(uint16_t irqs)
{
    // Ignore if not a TX DONE ISR
    if (!(irqs & SX1280_IRQ_TX_DONE))
        return;

    currOpmode = SX1280_MODE_FS;
    TXdoneCallback1();
}

void ICACHE_RAM_ATTR SX1280Driver::TXnb(const uint8_t *data, uint8_t length, uint32_t freq)
{
    SetMode(SX1280_MODE_FS);
    TxEnable(); // do first to allow PA stablise
    //ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
    if (freq)
        SetFrequency(freq);
    SetFIFOaddr(0x00, 0x00);   // not 100% sure if needed again
    WriteBuffer(0x00, (uint8_t*)data, length); //todo fix offset to equal fifo addr
    SetMode(SX1280_MODE_TX);
}

static uint8_t DMA_ATTR RXdataBuffer[16];

void ICACHE_RAM_ATTR SX1280Driver::RXnbISR(uint16_t irqs)
{
    int32_t FIFOaddr;
    // Ignore if not a RX DONE ISR, CRC fail or timeout
    if (!(irqs & SX1280_IRQ_RX_DONE) ||
        (irqs & (SX1280_IRQ_CRC_ERROR | SX1280_IRQ_RX_TX_TIMEOUT)))
        return;
    currOpmode = SX1280_MODE_FS;
    GetLastRssiSnr();
    FIFOaddr = GetRxBufferAddr();
    if (FIFOaddr < 0) // RX len is not correct!
        return;
    ReadBuffer(FIFOaddr, RXdataBuffer, RX_buffer_size);
    RXdoneCallback1(RXdataBuffer);
}

void ICACHE_RAM_ATTR SX1280Driver::RXnb(uint32_t freq)
{
    SetMode(SX1280_MODE_FS);
    RxEnable();
    //ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
    if (freq)
        SetFrequency(freq);
    //SetFIFOaddr(0x00, 0x00);
    SetMode(SX1280_MODE_RX);
}

void ICACHE_RAM_ATTR SX1280Driver::StopContRX(void)
{
    SetMode(SX1280_MODE_STDBY_RC);
    TxRxDisable();
}

int8_t ICACHE_RAM_ATTR SX1280Driver::GetLastPacketRSSI()
{
    uint8_t buff[2] = {0};
    ReadCommand(SX1280_RADIO_GET_RSSIINST, buff, sizeof(buff));
    LastRadioStatus = buff[0];      // [0] = status
    return (-((int)buff[1])) / 2;   // [1] = rssiInst
};


/*************************************************************************************
 * PRIVATE METHODS
 *************************************************************************************/

void ICACHE_RAM_ATTR SX1280Driver::SetFIFOaddr(uint8_t txBaseAddr, uint8_t rxBaseAddr)
{
    uint8_t buf[] = {txBaseAddr, rxBaseAddr};
    WriteCommand(SX1280_RADIO_SET_BUFFERBASEADDRESS, buf, sizeof(buf));
}

uint16_t ICACHE_RAM_ATTR SX1280Driver::GetIRQFlags()
{
    uint16_t irqs;
    uint8_t buff[3] = {0};
    ReadCommand(SX1280_RADIO_GET_IRQSTATUS, buff, sizeof(buff));
    LastRadioStatus = buff[0]; // [0] = status
    irqs = buff[1];
    irqs <<= 8;
    irqs += buff[2];
    return irqs;
}

void ICACHE_RAM_ATTR SX1280Driver::ClearIrqStatus(uint16_t irqMask)
{
    uint8_t buf[] = {(uint8_t)(irqMask >> 8), (uint8_t)irqMask};
    WriteCommand(SX1280_RADIO_CLR_IRQSTATUS, buf, sizeof(buf));
}

void SX1280Driver::SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
{
    uint8_t buf[8];

    buf[0] = (uint8_t)((irqMask >> 8) & 0x00FF);
    buf[1] = (uint8_t)(irqMask & 0x00FF);
    buf[2] = (uint8_t)((dio1Mask >> 8) & 0x00FF);
    buf[3] = (uint8_t)(dio1Mask & 0x00FF);
    buf[4] = (uint8_t)((dio2Mask >> 8) & 0x00FF);
    buf[5] = (uint8_t)(dio2Mask & 0x00FF);
    buf[6] = (uint8_t)((dio3Mask >> 8) & 0x00FF);
    buf[7] = (uint8_t)(dio3Mask & 0x00FF);

    WriteCommand(SX1280_RADIO_SET_DIOIRQPARAMS, buf, sizeof(buf));
}

int32_t ICACHE_RAM_ATTR SX1280Driver::GetRxBufferAddr(void)
{
    uint8_t status[3];
    ReadCommand(SX1280_RADIO_GET_RXBUFFERSTATUS, status, sizeof(status));
    // [0] status, [1] rxPayloadLength, [2] rxStartBufferPointer
    LastRadioStatus = status[0];              // [0] = status
    //return (status[1] == sizeof(RXdataBuffer)) ? status[2] : -1;
    return status[2];
}

void ICACHE_RAM_ATTR SX1280Driver::GetLastRssiSnr(void)
{
    uint8_t buff[9] = {0};
    ReadCommand(SX1280_RADIO_GET_PACKETSTATUS, buff, sizeof(buff));
    LastRadioStatus = buff[0];              // [0] = status
    LastPacketRssiRaw = buff[1];            // [1] = rssiSync
    LastPacketRSSI = -((int)buff[1]) / 2;
    LastPacketSNR = buff[2] / 4;            // [2] = snr
}

///////// SPI Interface

void ICACHE_RAM_ATTR SX1280Driver::WriteCommand(SX1280_RadioCommands_t command, uint8_t *buffer, uint8_t size)
{
    WaitOnBusy();
    writeRegisterBurst(command, buffer, size);
}
void ICACHE_RAM_ATTR SX1280Driver::ReadCommand(SX1280_RadioCommands_t command, uint8_t *buffer, uint8_t size)
{
    WaitOnBusy();
    readRegisterBurst(command, size, buffer);
}

void ICACHE_RAM_ATTR SX1280Driver::WriteRegister(uint16_t address, uint8_t *buffer, uint8_t size)
{
    WaitOnBusy();
    writeRegisterAddr(SX1280_RADIO_WRITE_REGISTER, address, buffer, size);
}
void ICACHE_RAM_ATTR SX1280Driver::ReadRegister(uint16_t address, uint8_t *buffer, uint8_t size)
{
    readRegisterAddr(SX1280_RADIO_READ_REGISTER, address, buffer, size);
}

void ICACHE_RAM_ATTR SX1280Driver::WriteBuffer(uint8_t offset, uint8_t *buffer, uint8_t size)
{
    WaitOnBusy();
    writeRegisterOffset(SX1280_RADIO_WRITE_BUFFER, offset, buffer, size);
}
void ICACHE_RAM_ATTR SX1280Driver::ReadBuffer(uint8_t offset, uint8_t *buffer, uint8_t size)
{
    WaitOnBusy();
    readRegisterOffset(SX1280_RADIO_READ_BUFFER, offset, buffer, size);
}
