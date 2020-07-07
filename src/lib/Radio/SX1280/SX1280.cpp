#include "SX1280.h"
#include "debug_elrs.h"

/////////////////////////////////////////////////////////////////

static SX1280Driver * instance = NULL;

static void ICACHE_RAM_ATTR _rxtx_isr_handler(void)
{
    instance->LastPacketIsrMicros = micros();
    switch (RadioInterface::p_state_isr) {
        case RX_DONE:
            instance->RXnbISR();
            break;
        case TX_DONE:
            instance->TXnbISR();
            break;
        default:
            break;
    };
}

/////////////////////////////////////////////////////////////////

SX1280Driver::SX1280Driver(HwSpi &spi):
    RadioInterface(spi)
{
    instance = this;
}

void SX1280Driver::Begin()
{
    RadioInterface::InitPins();
    // initialize low-level drivers
    RadioHalSpi::Begin(SX128X_SPI_SPEED);

    Reset();
    delay(100);

    uint8_t buffer[2];
    ReadRegister(REG_LR_FIRMWARE_VERSION_MSB, buffer, sizeof(buffer));
    uint16_t firmwareRev = buffer[0];
    firmwareRev <<= 8;
    firmwareRev += buffer[1];
    DEBUG_PRINT("SX1280 Firmware Revision: ");
    DEBUG_PRINTLN(firmwareRev);

    // Step 1: put in STDBY_RC mode
    SetMode(SX1280_MODE_STDBY_RC);
    // Step 2: set packet type to LoRa
    WriteCommand(SX1280_RADIO_SET_PACKETTYPE, SX1280_PACKET_TYPE_LORA);
    // Step 5: Configure Modulation Params
    ConfigModParams(currBW, currSF, currCR);
    WriteCommand(SX1280_RADIO_SET_AUTOFS, 0x01); //enable auto FS
    SetPacketParams(12, SX1280_LORA_PACKET_IMPLICIT, RX_BUFFER_LEN, SX1280_LORA_CRC_OFF, SX1280_LORA_IQ_NORMAL);
    // Step 3: Set Freq
    SetFrequency(currFreq);
    // Step 4: Config FIFO addr
    SetFIFOaddr(0x00, 0x00);
    // Config IRQs
    SetDioIrqParams(SX1280_IRQ_RADIO_ALL,
                    (SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE),
                    SX1280_IRQ_RADIO_NONE, SX1280_IRQ_RADIO_NONE);

    //attachInterrupt(digitalPinToInterrupt(_BUSY), this->busyISR, CHANGE); //not used atm
    attachInterrupt(digitalPinToInterrupt(_DIO2), _rxtx_isr_handler, RISING);
}

void SX1280Driver::End()
{
    detachInterrupt(_DIO2);
}

void ICACHE_RAM_ATTR SX1280Driver::Config(SX1280_RadioLoRaBandwidths_t bw, SX1280_RadioLoRaSpreadingFactors_t sf, SX1280_RadioLoRaCodingRates_t cr, uint32_t freq, uint8_t PreambleLength)
{
    SetMode(SX1280_MODE_STDBY_XOSC);
    ConfigModParams(bw, sf, cr);
    SetPacketParams(PreambleLength, SX1280_LORA_PACKET_IMPLICIT, RX_BUFFER_LEN, SX1280_LORA_CRC_OFF, SX1280_LORA_IQ_NORMAL);
    SetFrequency(freq);
}

void ICACHE_RAM_ATTR SX1280Driver::SetOutputPower(int8_t power)
{
    if (power < 0) power = 0;
    else if (power > 13) power = 13;
    power += 18;
    uint8_t buf[] = {(uint8_t)(power), SX1280_RADIO_RAMP_04_US};
    WriteCommand(SX1280_RADIO_SET_TXPARAMS, buf, sizeof(buf));
    DEBUG_PRINT("SetPower: ");
    DEBUG_PRINTLN(power);
}

void SX1280Driver::SetPacketParams(uint8_t PreambleLength, SX1280_RadioLoRaPacketLengthsModes_t HeaderType,
                                   uint8_t PayloadLength, SX1280_RadioLoRaCrcModes_t crc,
                                   SX1280_RadioLoRaIQModes_t InvertIQ)
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
    WORD_ALIGNED_ATTR uint8_t buf3[3]; //TODO make word alignmed

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
        buf3[0] = 0x02; // periodBase: 0x2 = 1ms, page 71 datasheet
        buf3[1] = 0xFF; // periodBaseCount: 0xffff = Rx Continuous mode.
        buf3[2] = 0xFF;
        WriteCommand(SX1280_RADIO_SET_RX, buf3, sizeof(buf3));
        break;

    case SX1280_MODE_TX:
        //uses timeout Time-out duration = periodBase * periodBaseCount
        buf3[0] = 0x02; // periodBase: 0x2 = 1ms, page 71 datasheet
        buf3[1] = 0x00; // periodBaseCount: 0x0 = no timeout, returns when TX is ready
        buf3[2] = 0x00;
        WriteCommand(SX1280_RADIO_SET_TX, buf3, sizeof(buf3));
        break;

    case SX1280_MODE_CAD:
        break;

    default:
        break;
    }
}

void SX1280Driver::ConfigModParams(SX1280_RadioLoRaBandwidths_t bw, SX1280_RadioLoRaSpreadingFactors_t sf, SX1280_RadioLoRaCodingRates_t cr)
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
    //uint32_t freq = (uint32_t)((double)Reqfreq / (double)SX1280_FREQ_STEP);
    // 1024 * x / 203125
    uint64_t freq = Reqfreq;
    freq <<= 10; // * 1024
    freq /= 203125;
    uint8_t buf[3];
    buf[0] = (uint8_t)((freq >> 16) & 0xFF);
    buf[1] = (uint8_t)((freq >> 8) & 0xFF);
    buf[2] = (uint8_t)(freq & 0xFF);

    WriteCommand(SX1280_RADIO_SET_RFFREQUENCY, buf, sizeof(buf));
    currFreq = Reqfreq;
}

int32_t ICACHE_RAM_ATTR SX1280Driver::GetFrequencyError()
{
    int32_t efe = 0;
    double efeHz;
    uint8_t fei_reg[3] = {0x0, 0x0, 0x0};

    ReadRegister(SX1280_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB, fei_reg, sizeof(fei_reg));
    efe = fei_reg[0] & 0b0111;
    efe <<= 8;
    efe += fei_reg[1];
    efe <<= 8;
    efe += fei_reg[2];

    if (fei_reg[0] & 0b1000) // Sign bit is on
    {
        // convert to negative
        efe -= 524288;
    }

    efeHz = efe;
    efeHz *= 1.55;
    switch (currBW) {
        case SX1280_LORA_BW_0200:
            efeHz *= 203.125;
            break;
        case SX1280_LORA_BW_0400:
            efeHz *= 406.25;
            break;
        case SX1280_LORA_BW_0800:
            efeHz *= 812.5;
            break;
        case SX1280_LORA_BW_1600:
            efeHz *= 1625.0;
            break;
    }
    efeHz /= 1600;
    return (int32_t)efeHz;
}

void ICACHE_RAM_ATTR SX1280Driver::setPPMoffsetReg(int32_t error_hz, uint32_t frf)
{
    // Apply freq error correction
}

void ICACHE_RAM_ATTR SX1280Driver::TXnbISR()
{
    ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
    TXdoneCallback1();
}

void ICACHE_RAM_ATTR SX1280Driver::TXnb(const uint8_t *data, uint8_t length, uint32_t freq)
{
    TxEnable(); // do first to allow PA stablise
    ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
    if (freq)
        SetFrequency(freq);
    SetFIFOaddr(0x00, 0x00);   // not 100% sure if needed again
    WriteBuffer(0x00, (uint8_t*)data, length); //todo fix offset to equal fifo addr
    SetMode(SX1280_MODE_TX);
}

static uint8_t DMA_ATTR RXdataBuffer[RX_BUFFER_LEN];

void ICACHE_RAM_ATTR SX1280Driver::RXnbISR()
{
    GetLastRssiSnr();
    ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
    uint8_t FIFOaddr = GetRxBufferAddr();
    ReadBuffer(FIFOaddr, RXdataBuffer, sizeof(RXdataBuffer));
    RXdoneCallback1(RXdataBuffer);
}

void ICACHE_RAM_ATTR SX1280Driver::RXnb(uint32_t freq)
{
    RxEnable();
    ClearIrqStatus(SX1280_IRQ_RADIO_ALL);
    if (freq)
        SetFrequency(freq);
    //SetFIFOaddr(0x00, 0x00);
    SetMode(SX1280_MODE_RX);
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

uint8_t ICACHE_RAM_ATTR SX1280Driver::GetRxBufferAddr()
{
    uint8_t status[3];
    ReadCommand(SX1280_RADIO_GET_RXBUFFERSTATUS, status, sizeof(status));
    // [0] status, [1] rxPayloadLength, [2] rxStartBufferPointer
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
