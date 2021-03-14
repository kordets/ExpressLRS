#ifndef __CRC_H_
#define __CRC_H_

#include "platform.h"
#include <stdint.h>

uint8_t CalcCRC(uint8_t data, uint8_t crc);
uint8_t CalcCRC(volatile uint8_t const *data, uint16_t length, uint8_t crc = 0);
uint8_t CalcCRC(uint8_t const *data, uint16_t length, uint8_t crc = 0);
uint8_t CalcCRCxor(uint8_t *data, uint16_t length, uint8_t crc = 0);
uint8_t CalcCRCxor(uint8_t data, uint8_t crc = 0);
uint16_t CalcCRC16_XMODEM(uint8_t const *data, uint16_t length);
uint16_t CalcCRC16_CCITT(uint8_t const *data, uint16_t length, uint16_t crc = 0);

#define CalcCRC16(D, L, C) (CalcCRC16_CCITT((D), (L), (C)))
//#define CalcCRC16(D, L, C) (CalcCRC16_XMODEM((D), (L)) ^ (C))

uint8_t CalcParity(uint8_t const *data, uint16_t length);

uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a);
uint8_t crc8_dvb_s2(uint8_t const* data, uint16_t length);

uint32_t CalcCRC32(uint8_t const *data, uint16_t len);

#if (CRSF_CMD_CRC)
uint8_t CalcCRCcmd(uint8_t const *data, uint16_t length, uint8_t crc = 0);
#endif

#if CRC16_POLY_TESTING
extern uint8_t CRC16_POLY_PKT[5];
#endif

#endif /* CRC_H_ */
