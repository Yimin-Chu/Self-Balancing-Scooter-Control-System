/**
 * @file    cali_store.h
 * @brief   IMU 标定数据的 Flash 掉电保存
 *
 * 存放位置：片内 Flash 最后一页 0x0800FC00~0x0800FFFF(1K)。
 * 该页已在 STM32F103C8Tx_FLASH.ld 里从代码区划走(FLASH 只给 63K)，
 * 两边地址必须保持一致。
 */

#ifndef __CALI_STORE_H__
#define __CALI_STORE_H__

#include "stm32f1xx_hal.h"

/* 中容量 STM32F103 一页 1K，这里用最后一页 */
#define CALI_STORE_ADDR    (0x0800FC00UL)
#define CALI_STORE_MAGIC   (0x494C4143UL) // "CALI" 小端
#define CALI_STORE_VERSION (1U)

typedef struct
{
    uint32_t magic;        // CALI_STORE_MAGIC
    uint16_t version;      // CALI_STORE_VERSION
    uint16_t length;       // 本结构体长度，方便以后加字段还能识别老数据
    float    med_angle;    // 平衡中值角(度)
    int32_t  gyrox_offset; // 陀螺 X 轴零偏(LSB)
    uint16_t crc;          // CRC16-MODBUS，覆盖本字段之前的所有字节
    uint16_t reserved;     // 补齐到偶数长度，Flash 按半字编程
} CaliData_tTypeDef;

uint8_t CaliStore_Load(void);
uint8_t CaliStore_Save(float med_angle, int32_t gyrox_offset);
uint8_t CaliStore_Erase(void);
uint8_t CaliStore_IsValid(void);
float   CaliStore_GetMedAngle(void);
int32_t CaliStore_GetGyroxOffset(void);

#endif
