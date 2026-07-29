/**
 * @file    comm_send.h
 * @brief   应用层上行：状态上报与周期上报调度
 */

#ifndef __COMM_SEND_H__
#define __COMM_SEND_H__

#include "comm_protocol.h"

/* 开机默认不自动上报(period=0)。
 * 二进制帧和 CLI 的文本输出走的是同一个串口，上报一开，终端上全是乱码，
 * 开机横幅和命令回执根本看不清。手机 App 连上后自己下发 REV_CMD_REPORT_CFG(0x06)
 * 打开即可；想恢复"一上电就上报"的老行为，把 PERIOD 改回 10U。 */
#define COMM_REPORT_PERIOD_DEFAULT (0U)
#define COMM_REPORT_MASK_DEFAULT   (COMM_REPORT_ATTITUDE | COMM_REPORT_MOTION)

/* 上报周期下限(单位 10ms)。115200bps 带宽充裕，仍保留下限避免无意义的过密上报 */
#define COMM_REPORT_PERIOD_MIN (5U)

void    CommSend_Init(void);
void    CommSend_Poll(void);
uint8_t CommSend_SetReportCfg(uint8_t period_10ms, uint16_t mask);
uint8_t CommSend_IsReporting(void);

void CommSend_Ack(uint8_t cmd, uint8_t error);
void CommSend_Attitude(void);
void CommSend_Motion(void);
void CommSend_Status(void);
void CommSend_Pid(void);
void CommSend_Version(void);

#endif
