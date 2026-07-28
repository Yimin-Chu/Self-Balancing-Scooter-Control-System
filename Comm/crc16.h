/**
 * @file    crc16.h
 * @brief   CRC16-MODBUS 校验，接口与追觅 Package/CRC_16.h 保持一致
 */

#ifndef __CRC16_H__
#define __CRC16_H__

#include "stm32f1xx_hal.h"

uint16_t CRC16(const uint8_t *pData, uint16_t len);
void     crc16_dynaCal(uint8_t data, uint8_t *const pCrc);
uint8_t  crc16_check(const uint8_t *const pByte, uint16_t len);

#endif
