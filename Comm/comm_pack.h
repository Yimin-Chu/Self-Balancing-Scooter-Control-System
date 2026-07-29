/**
 * @file    comm_pack.h
 * @brief   帧层：0x3C/0x3E 定界 + 0x3F 转义 + CRC16 的组帧与解帧
 */

#ifndef __COMM_PACK_H__
#define __COMM_PACK_H__

#include "comm_protocol.h"

typedef struct
{
    uint16_t rxFrame;    // 校验通过并成功分发的帧数
    uint16_t crcError;   // CRC 校验失败帧数
    uint16_t frameError; // 转义非法 / 帧长超限次数
    uint16_t cmdError;   // 命令ID 越界次数
    uint8_t  lastCmd;    // 最近一次成功分发的命令ID
    uint8_t  lastError;  // 最近一次 revError_eTypeDef
} CommPackStat_tTypeDef;

extern CommPackStat_tTypeDef commPackStat;

/* 帧内静默多久就放弃这一帧(ms)。串口再慢，一帧的字节间隔也不会有这么久 */
#define COMM_FRAME_TIMEOUT_MS (100U)

uint8_t       CommPack_RxByte(uint8_t byte);
void          CommPack_Poll(void);
packErrorType CommPack_Send(uint8_t cmdId, const void *pData, uint8_t length);

/* 追觅工程里发送用的是 send2board 宏，本工程对端是手机 App，改名 send2app */
#define send2app(id, add, len) CommPack_Send((uint8_t)(id), (add), (uint8_t)(len))

#endif
