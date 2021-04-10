#ifndef __CRC_H_
#define __CRC_H_

#include "platform.h"
#include <stdint.h>

uint8_t CalcCRCxor(uint8_t const *data, uint16_t length, uint8_t crc = 0);
uint8_t CalcCRCxor(uint8_t data, uint8_t crc = 0);

uint8_t CalcCRC8(uint8_t data, uint8_t crc, uint8_t poly=0xD5);
uint8_t CalcCRC8len(uint8_t const *data, uint16_t length, uint8_t crc = 0, uint8_t poly=0xD5);

uint16_t CalcCRC16(uint8_t const *data, uint16_t length, uint16_t crc = 0);

uint32_t CalcCRC32(uint8_t const *data, uint16_t len);

#if CRC16_POLY_TESTING
extern uint8_t CRC16_POLY_PKT[5];
#endif

#endif /* __CRC_H_ */
