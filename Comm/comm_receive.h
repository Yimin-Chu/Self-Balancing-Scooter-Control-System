/**
 * @file    comm_receive.h
 * @brief   应用层下行：命令分发表与命令处理
 */

#ifndef __COMM_RECEIVE_H__
#define __COMM_RECEIVE_H__

#include "comm_protocol.h"

/* 运动指令死区：|speed| / |turn| 小于此值视为松开摇杆 */
#define COMM_MOVE_DEADZONE (10)
/* 中值角合法范围(度)，超出必然是上位机传错了单位或字节序 */
#define COMM_MED_ANGLE_ABS_MAX (20.0f)
/* PID 参数上限，挡住手滑输入的离谱值 */
#define COMM_PID_GAIN_ABS_MAX (2000.0f)

typedef revError_eTypeDef (*FuncCmdReceive)(uint8_t *pByte, uint8_t len);

extern const FuncCmdReceive cmdReceiveTab[REV_CMD_TOTAL];

/* 运动方向标志，由 pid.c 的 Control() 消费 */
extern uint8_t Fore, Back, Left, Right;
/* 最近一次收到的单字节遥控码，仅用于调试观察 */
extern uint8_t Bluetooth_data;

void CommReceive_LegacyByte(uint8_t byte);

#endif
