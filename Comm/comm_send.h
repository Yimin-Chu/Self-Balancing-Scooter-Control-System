/**
 * @file    comm_send.h
 * @brief   应用层上行：状态上报与周期上报调度
 */

#ifndef __COMM_SEND_H__
#define __COMM_SEND_H__

#include "comm_protocol.h"

/* 默认自动上报：100ms 一次姿态 + 运动 */
#define COMM_REPORT_PERIOD_DEFAULT (10U)
#define COMM_REPORT_MASK_DEFAULT   (COMM_REPORT_ATTITUDE | COMM_REPORT_MOTION)

/* 上报周期下限(单位 10ms)。9600bps 实际吞吐只有 960 字节/秒，姿态+运动+状态
 * 三帧加起来约 50 字节，50ms 一轮就已经吃掉一半带宽，再快就会挤爆发送缓冲 */
#define COMM_REPORT_PERIOD_MIN (5U)

void    CommSend_Init(void);
void    CommSend_Poll(void);
uint8_t CommSend_SetReportCfg(uint8_t period_10ms, uint16_t mask);

void CommSend_Ack(uint8_t cmd, uint8_t error);
void CommSend_Attitude(void);
void CommSend_Motion(void);
void CommSend_Status(void);
void CommSend_Pid(void);
void CommSend_Version(void);

#endif
