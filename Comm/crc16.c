/**
 * @file    crc16.c
 * @brief   CRC16-MODBUS 校验
 *
 * 多项式 X16+X15+X2+1，反转形式 0XA001，初值 0XFFFF。
 *
 * 追觅原版用两张 256 字节查表(auchCRCHi/auchCRCLo)，这里改为逐位移位实现：
 * 两者对同一数据的返回值完全相同(已逐组数据比对验证)，只是省下 512 字节 Flash——
 * STM32F103C8T6 只有 64K Flash，本工程已用掉 49K 左右，所以选逐位版本。
 * 若日后换大容量芯片且在意 CPU 占用，可直接换回查表版，协议不受影响。
 */

#include "crc16.h"

/**
 * @brief  计算一段数据的 CRC16
 * @param  pData 数据首地址
 * @param  len   数据长度
 * @retval CRC 值，高字节为先发送的那一字节
 */
uint16_t CRC16(const uint8_t *pData, uint16_t len)
{
    uint16_t crc = 0XFFFFU;
    uint8_t  i;

    while (len--)
    {
        crc ^= (uint16_t)(*pData++);

        for (i = 0; i < 8U; i++)
        {
            crc = (crc & 0X0001U) ? ((crc >> 1) ^ 0XA001U) : (crc >> 1);
        }
    }

    return crc;
}

/**
 * @brief  逐字节动态累加 CRC，用于边组帧边算校验，避免额外的中间缓冲
 * @param  data 本次参与计算的字节
 * @param  pCrc 2 字节校验值，调用前必须初始化为 {0XFF, 0XFF}
 *              pCrc[0] 为低字节，pCrc[1] 为高字节(即先发送的字节)
 */
void crc16_dynaCal(uint8_t data, uint8_t *const pCrc)
{
    uint16_t crc = ((uint16_t)pCrc[1] << 8) | pCrc[0];
    uint8_t  i;

    crc ^= (uint16_t)data;

    for (i = 0; i < 8U; i++)
    {
        crc = (crc & 0X0001U) ? ((crc >> 1) ^ 0XA001U) : (crc >> 1);
    }

    pCrc[0] = (uint8_t)(crc & 0XFFU);
    pCrc[1] = (uint8_t)(crc >> 8);
}

/**
 * @brief  校验一段"数据 + 2 字节CRC"
 * @param  pByte 数据首地址
 * @param  len   含 CRC 的总长度
 * @retval 0:校验通过  1:校验失败
 */
uint8_t crc16_check(const uint8_t *const pByte, uint16_t len)
{
    uint16_t crcVal, crcCal;

    if (len > 2U)
    {
        crcVal = ((uint16_t)pByte[len - 2] << 8) | pByte[len - 1];
        crcCal = CRC16(pByte, len - 2);

        return (crcVal == crcCal) ? 0U : 1U;
    }

    return 1U;
}
