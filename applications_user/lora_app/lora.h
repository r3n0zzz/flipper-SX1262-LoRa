#pragma once

#include <furi.h>
#include <furi_hal.h>

#define PRESET_DEFAULT   0
#define PRESET_LONGRANGE 1
#define PRESET_FAST      2

extern const GpioPin* const pin_nss1;
extern const GpioPin* const pin_reset;
extern const GpioPin* const pin_beacon;
extern const GpioPin* const pin_ant_sw;
extern const GpioPin* const pin_dio22;
extern const GpioPin* const pin_busy;
extern const GpioPin* const pin_dio1;

bool begin(void);
void lora_deinit(void);
bool sanityCheck(void);
void checkBusy(void);
void configureRadioEssentials(void);

void setModeReceive(void);
void setModeStandby(void);
int lora_receive_async(uint8_t* buff, int buffMaxLen);
void transmit(uint8_t* data, int dataLen);

bool configSetFrequency(long frequencyInHz);
bool configSetBandwidth(int bw);
bool configSetSpreadingFactor(int sf);
bool configSetCodingRate(int cr);
bool configSetSyncWord(uint8_t syncWord, uint8_t controlBits);
bool configSetPreset(int preset);
void configSetPresetMeshtastic(int preset);
void setPacketParams(
    uint16_t packetParam1,
    uint8_t packetParam2,
    uint8_t packetParam3,
    uint8_t packetParam4,
    uint8_t packetParam5);

int16_t getRSSI(void);
int8_t getSNR(void);
uint32_t getFreqInt(void);
void abandone(void);
