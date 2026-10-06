/*
 * Electronic Cats Flipper LoRa (SX1262) Driver
 * Fixed and patched for Electronic Cats Flipper Sub-GHz Add-On
 */

#include "lora.h"

#define TAG "LORA"

#define REG_LR_SYNCWORD     0x0740
#define RADIO_READ_REGISTER 0x1D

#define REG_RFFrequency31_24 0x088B
#define REG_RFFrequency23_16 0x088C
#define REG_RFFrequency15_8  0x088D
#define REG_RFFrequency7_0   0x088E

#define FREQ_STEP 0.95367431640625

static uint32_t timeout = 1000;

FuriHalSpiBusHandle spi_handle;
const FuriHalSpiBusHandle* spi = &spi_handle;

/* Hardware pin mapping from Electronic Cats Sub-GHz Schematic & PCB:
 * Pin 10 (PA14 / SWCLK): LED beacon
 * Pin 4  (PA4): CC1101 CS (NSS0) - MUST BE KEPT HIGH TO DESELECT
 * Pin 16 (PC0): SX1262 CS (NSS1)
 * Pin 15 (PC1): SX1262 RESET
 * Pin 13 (PB6 / USART_TX): PE42421 RF Switch CTRL_B / V1 (ANT_SW)
 * Pin 12 (PA13 / SWDIO):   PE42421 RF Switch CTRL / V2 (DIO22)
 * Pin 14 (PB7 / USART_RX): SX1262 BUSY
 * Pin 7  (PC3): SX1262 DIO1 (IRQ)
 */
const GpioPin* const pin_beacon = &gpio_swclk;
const GpioPin* const pin_nss0 = &gpio_ext_pa4;
const GpioPin* const pin_nss1 = &gpio_ext_pc0;
const GpioPin* const pin_reset = &gpio_ext_pc1;
const GpioPin* const pin_ant_sw = &gpio_usart_tx;
const GpioPin* const pin_dio22 = &gpio_swdio;
const GpioPin* const pin_busy = &gpio_usart_rx;
const GpioPin* const pin_dio1 = &gpio_ext_pc3;

bool inReceiveMode = false;
uint8_t spiBuff[32]; // Buffer for sending SPI commands to radio

// Config variables
uint32_t pllFrequency = 959447040; // 915 MHz default
uint8_t bandwidth = 0x04; // 125 kHz
uint8_t codingRate = 0x01; // CR 4/5
uint8_t spreadingFactor = 0x07; // SF7
uint16_t syncWord = 0x3444;
uint8_t lowDataRateOptimize = 0;
uint32_t transmitTimeout = 1000;

uint16_t current_preamble = 8;
uint8_t current_header_type = 0; // 0 = Variable, 1 = Fixed
uint8_t current_crc = 1; // 0 = Off, 1 = On (Default ON for LoRa/Meshtastic)
uint8_t current_invert_iq = 0; // 0 = Standard, 1 = Inverted

int rssi = 0;
int snr = 0;
int signalRssi = 0;

void abandone() {
    FURI_LOG_E(TAG, "Electronic Cats SX1262 Driver initialized");
}

int16_t getRSSI() {
    return rssi;
}

int8_t getSNR() {
    return snr;
}

/* PE42421 RF Switch Control:
 * Pin 1: RF1 = Transmit path (SW_RFO)
 * Pin 3: RF2 = Receive path (SW_RFI)
 * Pin 5: RFC = Antenna
 * Pin 4: CTRL = DIO22 (gpio_swdio)
 * Pin 6: CTRL_B = ANT_SW (gpio_usart_tx)
 *
 * Truth Table (Complementary mode):
 * - RX (RFC to RF2): CTRL (DIO22) = HIGH, CTRL_B (ANT_SW) = LOW
 * - TX (RFC to RF1): CTRL (DIO22) = LOW,  CTRL_B (ANT_SW) = HIGH
 * - OFF / Standby:   CTRL = LOW, CTRL_B = LOW
 */
static void rf_switch_rx() {
    furi_hal_gpio_write(pin_dio22, true);
    furi_hal_gpio_write(pin_ant_sw, false);
}

static void rf_switch_tx() {
    furi_hal_gpio_write(pin_dio22, false);
    furi_hal_gpio_write(pin_ant_sw, true);
}

static void rf_switch_off() {
    furi_hal_gpio_write(pin_dio22, false);
    furi_hal_gpio_write(pin_ant_sw, false);
}

void checkBusy() {
    uint8_t busy_timeout_cnt = 0;
    furi_hal_gpio_init_simple(pin_busy, GpioModeInput);

    while(furi_hal_gpio_read(pin_busy)) {
        furi_delay_ms(1);
        busy_timeout_cnt++;
        if(busy_timeout_cnt > 20) {
            FURI_LOG_W(TAG, "Busy timeout");
            break;
        }
    }
}

void readRegisters(uint16_t address, uint8_t* buffer, uint16_t size) {
    uint8_t addr_h = (address >> 8) & 0xFF;
    uint8_t addr_l = address & 0xFF;

    checkBusy();
    furi_hal_spi_acquire(spi);

    spiBuff[0] = RADIO_READ_REGISTER;
    spiBuff[1] = addr_h;
    spiBuff[2] = addr_l;
    spiBuff[3] = 0x00;

    furi_hal_spi_bus_tx(spi, spiBuff, 4, timeout);

    for(uint16_t index = 0; index < size; index++) {
        furi_hal_spi_bus_rx(spi, buffer + index, 1, timeout);
    }

    furi_hal_spi_release(spi);
}

uint8_t readRegister(uint16_t address) {
    uint8_t data = 0;
    readRegisters(address, &data, 1);
    return data;
}

uint32_t getFreqInt() {
    uint8_t MsbH = readRegister(REG_RFFrequency31_24);
    uint8_t MsbL = readRegister(REG_RFFrequency23_16);
    uint8_t Mid = readRegister(REG_RFFrequency15_8);
    uint8_t Lsb = readRegister(REG_RFFrequency7_0);

    uint32_t currentFreq = ((uint32_t)MsbH << 24) | ((uint32_t)MsbL << 16) |
                           ((uint32_t)Mid << 8) | (uint32_t)Lsb;

    return (uint32_t)(((uint64_t)currentFreq * 32000000ULL) >> 25);
}

uint32_t frequencyToPLL(long rfFreq) {
    return (uint32_t)(((uint64_t)rfFreq << 25) / 32000000ULL);
}

void updateRadioFrequency() {
    checkBusy();
    furi_hal_spi_acquire(spi);

    spiBuff[0] = 0x86; // Opcode for SetRfFrequency
    spiBuff[1] = (pllFrequency >> 24) & 0xFF;
    spiBuff[2] = (pllFrequency >> 16) & 0xFF;
    spiBuff[3] = (pllFrequency >> 8) & 0xFF;
    spiBuff[4] = pllFrequency & 0xFF;

    furi_hal_spi_bus_tx(spi, spiBuff, 5, timeout);
    furi_hal_spi_release(spi);
    furi_delay_ms(5);
}

bool configSetFrequency(long frequencyInHz) {
    if(frequencyInHz < 150000000 || frequencyInHz > 960000000) {
        return false;
    }
    pllFrequency = frequencyToPLL(frequencyInHz);
    updateRadioFrequency();
    return true;
}

void updateModulationParameters() {
    checkBusy();
    furi_hal_spi_acquire(spi);

    spiBuff[0] = 0x8B; // Opcode for SetModulationParameters
    spiBuff[1] = spreadingFactor;
    spiBuff[2] = bandwidth;
    spiBuff[3] = codingRate;
    spiBuff[4] = lowDataRateOptimize;

    furi_hal_spi_bus_tx(spi, spiBuff, 5, timeout);
    furi_hal_spi_release(spi);
    furi_delay_ms(10);

    switch(spreadingFactor) {
    case 12:
        transmitTimeout = 25000;
        break;
    case 11:
        transmitTimeout = 15000;
        break;
    case 10:
        transmitTimeout = 8000;
        break;
    case 9:
        transmitTimeout = 4000;
        break;
    case 8:
        transmitTimeout = 2500;
        break;
    case 7:
        transmitTimeout = 1500;
        break;
    default:
        transmitTimeout = 1000;
        break;
    }
}

bool configSetPreset(int preset) {
    if(preset == PRESET_DEFAULT) {
        bandwidth = 0x04; // 125 kHz
        codingRate = 0x01; // CR 4/5
        spreadingFactor = 0x07; // SF7
        lowDataRateOptimize = 0;
        updateModulationParameters();
        return true;
    }

    if(preset == PRESET_LONGRANGE) {
        bandwidth = 0x04; // 125 kHz
        codingRate = 0x01; // CR 4/5
        spreadingFactor = 12; // SF12
        lowDataRateOptimize = 1;
        updateModulationParameters();
        return true;
    }

    if(preset == PRESET_FAST) {
        bandwidth = 0x06; // 500 kHz
        codingRate = 0x01; // CR 4/5
        spreadingFactor = 0x07; // SF7
        lowDataRateOptimize = 0;
        updateModulationParameters();
        return true;
    }

    return false;
}

bool waitForRadioCommandCompletion(uint32_t wait_timeout) {
    uint32_t startTime = furi_get_tick();
    uint8_t tx_buf[2] = {0xC0, 0x00};
    uint8_t rx_buf[2] = {0x00, 0x00};

    while((furi_get_tick() - startTime) < furi_ms_to_ticks(wait_timeout)) {
        furi_delay_ms(2);
        checkBusy();
        furi_hal_spi_acquire(spi);
        furi_hal_spi_bus_trx(spi, tx_buf, rx_buf, 2, timeout);
        furi_hal_spi_release(spi);

        uint8_t chipMode = (rx_buf[1] >> 4) & 0x07;
        uint8_t commandStatus = (rx_buf[1] >> 1) & 0x07;

        if(commandStatus >= 2 || chipMode == 0x02 || chipMode == 0x03) {
            return true;
        }
    }
    return false;
}

bool configSetBandwidth(int bw) {
    if(bw < 0 || bw > 0x0A || bw == 7) {
        return false;
    }
    bandwidth = bw;
    updateModulationParameters();
    return true;
}

bool configSetCodingRate(int cr) {
    if(cr < 1 || cr > 4) {
        return false;
    }
    codingRate = cr;
    updateModulationParameters();
    return true;
}

bool configSetSyncWord(uint8_t syncWordVal, uint8_t controlBits) {
    uint8_t msb = (syncWordVal & 0xF0) | ((controlBits & 0xF0) >> 4);
    uint8_t lsb = ((syncWordVal & 0x0F) << 4) | (controlBits & 0x0F);

    checkBusy();
    furi_hal_spi_acquire(spi);

    spiBuff[0] = 0x0D; // WriteRegister
    spiBuff[1] = 0x07;
    spiBuff[2] = 0x40;
    spiBuff[3] = msb;
    spiBuff[4] = lsb;

    furi_hal_spi_bus_tx(spi, spiBuff, 5, timeout);
    furi_hal_spi_release(spi);
    furi_delay_ms(2);

    return true;
}

bool configSetSpreadingFactor(int sf) {
    if(sf < 5 || sf > 12) {
        return false;
    }
    lowDataRateOptimize = (sf == 12 && bandwidth == 4) ? 1 : 0;
    spreadingFactor = sf;
    updateModulationParameters();
    return true;
}

void setPacketParams(
    uint16_t packetParam1,
    uint8_t packetParam2,
    uint8_t packetParam3,
    uint8_t packetParam4,
    uint8_t packetParam5) {
    current_preamble = packetParam1;
    current_header_type = packetParam2;
    current_crc = packetParam4;
    current_invert_iq = packetParam5;

    uint8_t preambleMSB = (packetParam1 >> 8) & 0xFF;
    uint8_t preambleLSB = packetParam1 & 0xFF;

    checkBusy();
    furi_hal_spi_acquire(spi);

    spiBuff[0] = 0x8C; // SetPacketParameters
    spiBuff[1] = preambleMSB;
    spiBuff[2] = preambleLSB;
    spiBuff[3] = packetParam2; // Header Type
    spiBuff[4] = packetParam3; // Payload Length
    spiBuff[5] = packetParam4; // CRC
    spiBuff[6] = packetParam5; // Invert IQ

    furi_hal_spi_bus_tx(spi, spiBuff, 7, timeout);
    furi_hal_spi_release(spi);
    waitForRadioCommandCompletion(100);
}

void configureRadioEssentials() {
    configSetFrequency(910300000);

    // SetPacketType = LoRa (0x01)
    checkBusy();
    furi_hal_spi_acquire(spi);
    spiBuff[0] = 0x8A;
    spiBuff[1] = 0x01;
    furi_hal_spi_bus_tx(spi, spiBuff, 2, timeout);
    furi_hal_spi_release(spi);
    furi_delay_ms(5);

    // StopTimerOnPreamble (0x9F, 0x00)
    checkBusy();
    furi_hal_spi_acquire(spi);
    spiBuff[0] = 0x9F;
    spiBuff[1] = 0x00;
    furi_hal_spi_bus_tx(spi, spiBuff, 2, timeout);
    furi_hal_spi_release(spi);
    furi_delay_ms(5);

    configSetPreset(PRESET_DEFAULT);
    configSetSyncWord(0x34, 0x44);

    // SetPaConfig (0x95) for SX1262 high power (+22dBm capable)
    checkBusy();
    furi_hal_spi_acquire(spi);
    spiBuff[0] = 0x95;
    spiBuff[1] = 0x04; // paDutyCycle
    spiBuff[2] = 0x07; // hpMax
    spiBuff[3] = 0x00; // device select SX1262
    spiBuff[4] = 0x01; // paLut
    furi_hal_spi_bus_tx(spi, spiBuff, 5, timeout);
    furi_hal_spi_release(spi);
    furi_delay_ms(5);

    // SetTxParams (0x8E): +22dBm power, 40us ramp
    checkBusy();
    furi_hal_spi_acquire(spi);
    spiBuff[0] = 0x8E;
    spiBuff[1] = 22; // 22 dBm
    spiBuff[2] = 0x02; // 40us ramp
    furi_hal_spi_bus_tx(spi, spiBuff, 3, timeout);
    furi_hal_spi_release(spi);
    furi_delay_ms(5);

    // SetLoRaSymbNumTimeout (0xA0, 0x00)
    checkBusy();
    furi_hal_spi_acquire(spi);
    spiBuff[0] = 0xA0;
    spiBuff[1] = 0x00;
    furi_hal_spi_bus_tx(spi, spiBuff, 2, timeout);
    furi_hal_spi_release(spi);
    furi_delay_ms(5);

    // SetDioIrqParams (0x08):
    // Enable all IRQs internally (0xFFFF), route ONLY RxDone (0x0002) to DIO1
    checkBusy();
    furi_hal_spi_acquire(spi);
    spiBuff[0] = 0x08;
    spiBuff[1] = 0xFF; // IrqMask MSB
    spiBuff[2] = 0xFF; // IrqMask LSB
    spiBuff[3] = 0x00; // DIO1Mask MSB
    spiBuff[4] = 0x02; // DIO1Mask LSB: RxDone only
    spiBuff[5] = 0x00;
    spiBuff[6] = 0x00;
    spiBuff[7] = 0x00;
    spiBuff[8] = 0x00;
    furi_hal_spi_bus_tx(spi, spiBuff, 9, timeout);
    furi_hal_spi_release(spi);
    furi_delay_ms(5);
}

void setModeReceive() {
    // 1. Configure RF switch for RX path (RFC -> RF2 / SW_RFI)
    rf_switch_rx();

    // 2. Set packet parameters
    checkBusy();
    furi_hal_spi_acquire(spi);
    spiBuff[0] = 0x8C;
    spiBuff[1] = (current_preamble >> 8) & 0xFF;
    spiBuff[2] = current_preamble & 0xFF;
    spiBuff[3] = current_header_type;
    spiBuff[4] = 0xFF; // Max payload length (255)
    spiBuff[5] = current_crc;
    spiBuff[6] = current_invert_iq;
    furi_hal_spi_bus_tx(spi, spiBuff, 7, timeout);
    furi_hal_spi_release(spi);

    // 3. Configure DIO1 IRQ: internal all enabled (0xFFFF), DIO1 triggers on RxDone (0x0002)
    checkBusy();
    furi_hal_spi_acquire(spi);
    spiBuff[0] = 0x08; // SetDioIrqParams
    spiBuff[1] = 0xFF;
    spiBuff[2] = 0xFF;
    spiBuff[3] = 0x00;
    spiBuff[4] = 0x06; // DIO1Mask = RxDone (0x02) | PreambleDetected (0x04)
    spiBuff[5] = 0x00;
    spiBuff[6] = 0x00;
    spiBuff[7] = 0x00;
    spiBuff[8] = 0x00;
    furi_hal_spi_bus_tx(spi, spiBuff, 9, timeout);
    furi_hal_spi_release(spi);

    // 4. Clear existing interrupts
    checkBusy();
    furi_hal_spi_acquire(spi);
    spiBuff[0] = 0x02; // ClearIRQStatus
    spiBuff[1] = 0xFF;
    spiBuff[2] = 0xFF;
    furi_hal_spi_bus_tx(spi, spiBuff, 3, timeout);
    furi_hal_spi_release(spi);

    // 5. SetRx continuous mode (0xFFFFFF)
    checkBusy();
    furi_hal_spi_acquire(spi);
    spiBuff[0] = 0x82; // SetRx
    spiBuff[1] = 0xFF;
    spiBuff[2] = 0xFF;
    spiBuff[3] = 0xFF;
    furi_hal_spi_bus_tx(spi, spiBuff, 4, timeout);
    furi_hal_spi_release(spi);

    inReceiveMode = true;
    FURI_LOG_I(TAG, "RX armed: Freq=%lu Hz, Preamble=%u, CRC=%u", (unsigned long)getFreqInt(), current_preamble, current_crc);
}

void setModeStandby() {
    rf_switch_off();
    checkBusy();
    furi_hal_spi_acquire(spi);
    spiBuff[0] = 0x80; // SetStandby
    spiBuff[1] = 0x00; // STDBY_RC
    furi_hal_spi_bus_tx(spi, spiBuff, 2, timeout);
    furi_hal_spi_release(spi);
    waitForRadioCommandCompletion(100);
    inReceiveMode = false;
}

void transmit(uint8_t* data, int dataLen) {
    if(dataLen > 255) {
        dataLen = 255;
    }
    if(inReceiveMode) {
        setModeStandby();
    }

    // 1. Switch RF switch to TX path (RFC -> RF1 / SW_RFO)
    rf_switch_tx();

    // 2. Set packet parameters
    checkBusy();
    furi_hal_spi_acquire(spi);
    spiBuff[0] = 0x8C;
    spiBuff[1] = (current_preamble >> 8) & 0xFF;
    spiBuff[2] = current_preamble & 0xFF;
    spiBuff[3] = current_header_type;
    spiBuff[4] = (uint8_t)dataLen;
    spiBuff[5] = current_crc;
    spiBuff[6] = current_invert_iq;
    furi_hal_spi_bus_tx(spi, spiBuff, 7, timeout);
    furi_hal_spi_release(spi);
    waitForRadioCommandCompletion(100);

    // 3. Write payload into buffer
    checkBusy();
    furi_hal_spi_acquire(spi);
    spiBuff[0] = 0x0E; // WriteBuffer
    spiBuff[1] = 0x00; // Offset
    furi_hal_spi_bus_tx(spi, spiBuff, 2, timeout);
    furi_hal_spi_bus_tx(spi, data, dataLen, timeout);
    furi_hal_spi_release(spi);
    waitForRadioCommandCompletion(100);

    // 4. SetTx
    checkBusy();
    furi_hal_spi_acquire(spi);
    spiBuff[0] = 0x83; // SetTx
    spiBuff[1] = 0x00;
    spiBuff[2] = 0x00;
    spiBuff[3] = 0x00;
    furi_hal_spi_bus_tx(spi, spiBuff, 4, timeout);
    furi_hal_spi_release(spi);

    waitForRadioCommandCompletion(transmitTimeout);

    // Return to safe standby
    rf_switch_off();
    inReceiveMode = false;
}

int lora_receive_async(uint8_t* buff, int buffMaxLen) {
    if(!inReceiveMode) {
        setModeReceive();
    }

    // Check DIO1 interrupt line (high = interrupt pending)
    if(!furi_hal_gpio_read(pin_dio1)) {
        return -1;
    }

    // Blink beacon LED
    furi_hal_gpio_write(pin_beacon, true);

    // 1. Get IRQ status using opcode 0x12
    uint8_t tx_irq[4] = {0x12, 0x00, 0x00, 0x00};
    uint8_t rx_irq[4] = {0};
    checkBusy();
    furi_hal_spi_acquire(spi);
    furi_hal_spi_bus_trx(spi, tx_irq, rx_irq, 4, timeout);
    furi_hal_spi_release(spi);

    uint16_t irqStatus = ((uint16_t)rx_irq[2] << 8) | rx_irq[3];

    // 2. Clear interrupts
    checkBusy();
    furi_hal_spi_acquire(spi);
    spiBuff[0] = 0x02; // ClearIRQStatus
    spiBuff[1] = 0xFF;
    spiBuff[2] = 0xFF;
    furi_hal_spi_bus_tx(spi, spiBuff, 3, timeout);
    furi_hal_spi_release(spi);

    // If RxDone bit (0x0002) is NOT set, this is not a finished packet
    if(!(irqStatus & 0x0002)) {
        if(irqStatus & 0x0004) {
            FURI_LOG_I(TAG, "Preamble detected on channel! (irqStatus=0x%04X)", irqStatus);
        }
        furi_hal_gpio_write(pin_beacon, false);
        return -1;
    }

    // If CRC error occurred, discard packet and re-arm
    if(irqStatus & 0x0040) {
        FURI_LOG_W(TAG, "CRC error on received packet (irq=0x%04X)", irqStatus);
        furi_hal_gpio_write(pin_beacon, false);
        inReceiveMode = false;
        setModeReceive();
        return -1;
    }

    // 3. GetPacketStatus (Opcode 0x14)
    uint8_t tx_stat[5] = {0x14, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx_stat[5] = {0};
    checkBusy();
    furi_hal_spi_acquire(spi);
    furi_hal_spi_bus_trx(spi, tx_stat, rx_stat, 5, timeout);
    furi_hal_spi_release(spi);

    rssi = -((int)rx_stat[2]) / 2;
    snr = ((int8_t)rx_stat[3]) / 4;
    signalRssi = -((int)rx_stat[4]) / 2;

    // 4. GetRxBufferStatus (Opcode 0x13)
    uint8_t tx_bufstat[4] = {0x13, 0x00, 0x00, 0x00};
    uint8_t rx_bufstat[4] = {0};
    checkBusy();
    furi_hal_spi_acquire(spi);
    furi_hal_spi_bus_trx(spi, tx_bufstat, rx_bufstat, 4, timeout);
    furi_hal_spi_release(spi);

    uint8_t payloadLen = rx_bufstat[2];
    uint8_t startAddress = rx_bufstat[3];
    if(payloadLen == 0) {
        furi_hal_gpio_write(pin_beacon, false);
        inReceiveMode = false;
        setModeReceive();
        return -1;
    }

    if(payloadLen > buffMaxLen) {
        payloadLen = buffMaxLen;
    }

    // 5. Read payload via ReadBuffer (Opcode 0x1E)
    checkBusy();
    furi_hal_spi_acquire(spi);
    spiBuff[0] = 0x1E; // ReadBuffer
    spiBuff[1] = startAddress;
    spiBuff[2] = 0x00; // Dummy
    furi_hal_spi_bus_tx(spi, spiBuff, 3, timeout);
    furi_hal_spi_bus_rx(spi, buff, payloadLen, timeout);
    furi_hal_spi_release(spi);

    furi_hal_gpio_write(pin_beacon, false);

    // Re-arm continuous RX for subsequent packets
    inReceiveMode = false;
    setModeReceive();

    return payloadLen;
}

bool sanityCheck() {
    uint8_t tx_cmd[4] = {0x1D, 0x07, 0x40, 0x00};
    uint8_t regValue = 0;

    checkBusy();
    furi_hal_spi_acquire(spi);

    if(furi_hal_spi_bus_tx(spi, tx_cmd, 4, timeout) &&
       furi_hal_spi_bus_rx(spi, &regValue, 1, timeout)) {
        furi_hal_spi_release(spi);
        FURI_LOG_I(TAG, "SyncWord reg 0x0740 = 0x%02X", regValue);

        if(regValue == 0x14) {
            // Flash LED twice on successful connection
            furi_hal_gpio_write(pin_beacon, true);
            furi_delay_ms(60);
            furi_hal_gpio_write(pin_beacon, false);
            furi_delay_ms(60);
            furi_hal_gpio_write(pin_beacon, true);
            furi_delay_ms(60);
            furi_hal_gpio_write(pin_beacon, false);
            return true;
        }
    } else {
        furi_hal_spi_release(spi);
    }

    return false;
}

static void init_spi() {
    memcpy(&spi_handle, &furi_hal_spi_bus_handle_external, sizeof(FuriHalSpiBusHandle));
    spi_handle.cs = pin_nss1;
    furi_hal_spi_bus_handle_init(&spi_handle);
}

static void deinit_spi() {
    furi_hal_spi_bus_handle_deinit(&spi_handle);
}

bool begin() {
    init_spi();

    furi_hal_gpio_init_simple(pin_reset, GpioModeOutputPushPull);
    furi_hal_gpio_init_simple(pin_nss0, GpioModeOutputPushPull);
    furi_hal_gpio_init_simple(pin_beacon, GpioModeOutputPushPull);
    furi_hal_gpio_init_simple(pin_ant_sw, GpioModeOutputPushPull);
    furi_hal_gpio_init_simple(pin_dio22, GpioModeOutputPushPull);
    furi_hal_gpio_init_simple(pin_dio1, GpioModeInput);
    furi_hal_gpio_init_simple(pin_busy, GpioModeInput);

    // CRITICAL FIX: Keep CC1101 CS (NSS0) HIGH so it does NOT assert on MISO!
    furi_hal_gpio_write(pin_nss0, true);

    // Set RF switch off initially
    rf_switch_off();
    furi_hal_gpio_write(pin_beacon, false);

    // Hardware reset SX1262 (Active Low)
    furi_hal_gpio_write(pin_reset, false);
    furi_delay_ms(10);
    furi_hal_gpio_write(pin_reset, true);
    furi_delay_ms(30);

    checkBusy();

    // Verify SPI communication with SX1262
    bool success = sanityCheck();
    if(!success) {
        FURI_LOG_E(TAG, "SX1262 Sanity check failed!");
        return false;
    }

    configureRadioEssentials();
    return true;
}

void lora_deinit() {
    setModeStandby();
    deinit_spi();

    // Safely return all pins to Analog mode
    furi_hal_gpio_init_simple(pin_reset, GpioModeAnalog);
    furi_hal_gpio_init_simple(pin_nss0, GpioModeAnalog);
    furi_hal_gpio_init_simple(pin_beacon, GpioModeAnalog);
    furi_hal_gpio_init_simple(pin_ant_sw, GpioModeAnalog);
    furi_hal_gpio_init_simple(pin_dio22, GpioModeAnalog);
    furi_hal_gpio_init_simple(pin_dio1, GpioModeAnalog);
    furi_hal_gpio_init_simple(pin_busy, GpioModeAnalog);
}
