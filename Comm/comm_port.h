/**
 * @file    comm_port.h
 * @brief   传输层：USART3(JDY-31 蓝牙) 收发环形缓冲
 */

#ifndef __COMM_PORT_H__
#define __COMM_PORT_H__

#include "comm_protocol.h"

typedef struct
{
    uint16_t rxDrop;      // 接收缓冲满而丢弃的字节数
    uint16_t txDropFrame; // 发送缓冲装不下整帧而丢弃的帧数
    uint16_t uartError;   // UART 溢出/噪声/帧错误次数
} CommPortStat_tTypeDef;

extern CommPortStat_tTypeDef commPortStat;

void    CommPort_Init(void);
uint8_t CommPort_RxPop(uint8_t *const pByte);
uint8_t CommPort_TxPushFrame(const uint8_t *pData, uint16_t len);
void    CommPort_TxKick(void);

#endif
