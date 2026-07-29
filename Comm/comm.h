/**
 * @file    comm.h
 * @brief   通信层门面：main.c 只需要 Comm_Init() 和 Comm_Poll()
 *
 * 分层结构(自下而上，对照追觅工程)：
 *
 *   comm_port.c     传输层    USART3 收发环形缓冲 + HAL 回调   <- task_com.c
 *   crc16.c         校验层    CRC16-MODBUS                     <- Package/CRC_16.c
 *   comm_pack.c     帧层      0x3C/0x3E 定界 + 0x3F 转义       <- Package/pack.c
 *   comm_receive.c  应用层    下行命令分发表                    <- ComProtocol/receive.c
 *   comm_send.c     应用层    上行状态上报                      <- ComProtocol/send.c
 *   comm.c          门面      初始化 + 主循环轮询
 */

#ifndef __COMM_H__
#define __COMM_H__

#include "comm_protocol.h"

/* 1:保留旧的单字节遥控码(0x01/0x03/0x05/0x07/0x09)  0:只认新协议帧 */
#define COMM_LEGACY_BYTE_CMD_ENABLE (1)

/* 1:帧外的可见字符转给串口命令行(Cli/cli.c)  0:关闭 CLI */
#define COMM_CLI_ENABLE (1)

/* 单次 Comm_Poll() 最多解析的字节数。115200bps 一个 10ms 周期最多约 115 字节，
 * 与 RX 环形缓冲(64)对齐，避免积压溢出；设上限也防止一次解析拖太久 */
#define COMM_RX_POLL_BUDGET (64U)

void Comm_Init(void);
void Comm_Poll(void);

#endif
