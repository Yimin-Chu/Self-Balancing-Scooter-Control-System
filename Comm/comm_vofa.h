/**
 * @file    comm_vofa.h
 * @brief   VOFA+ 波形输出：把控制环内部量按 VOFA+ 能直接画的格式吐到 USART3
 *
 * 用途是调 PID：把"目标值/实测值/环路输出"同屏画出来，看超调、振荡和稳态误差，
 * 比在终端里刷数字直观得多。配合 CLI 的 `pid` 命令可以边看波形边改增益。
 *
 * 支持 VOFA+ 的两种内置协议：
 *
 *   JustFloat  N 个小端 float + 4 字节帧尾 00 00 80 7F。二进制，带宽省，推荐。
 *   FireWater  文本 "1.23,4.56,...\n"。带宽是 JustFloat 的两倍，但用任何串口
 *              助手都能肉眼核对数值，怀疑通道错位时拿它对一下。
 *
 * 和 comm_send.c 的二进制上报共用一个串口，两者同时开会互相插队谁都解不出来，
 * 所以 Vofa_SetMode() 打开波形时会顺手关掉周期上报。
 */

#ifndef __COMM_VOFA_H__
#define __COMM_VOFA_H__

#include "stm32f1xx_hal.h"

/* 通道数。改这个值必须同步改 comm_vofa.c 里的 vofa_sample() 与通道名表 */
#define VOFA_CH_NUM (8U)

/* 控制环频率(Hz)，仅用于把分频值换算成人看得懂的输出频率。
 * 与 MyCode/inv_mpu.h 的 DEFAULT_MPU_HZ 一致——控制周期由 DMP 数据就绪中断驱动 */
#define VOFA_CTRL_HZ (100U)

/* 分频范围：每 n 个控制周期发一帧。
 * 下限 1 = 100Hz 满速；JustFloat 一帧 36 字节，此时占 3.6KB/s，约为 115200bps
 * 可用带宽(11.5KB/s)的三成，还留得出 CLI 和协议帧的余量。
 * 上限 50 = 2Hz，再慢波形就没有观察价值了 */
#define VOFA_DIV_MIN (1U)
#define VOFA_DIV_MAX (50U)

/* 开机默认分频。2 -> 50Hz，对 10ms 控制周期的阶跃响应足够，带宽还很宽裕 */
#define VOFA_DIV_DEFAULT (2U)

typedef enum
{
    VOFA_MODE_OFF = 0,   // 关闭
    VOFA_MODE_JUSTFLOAT, // 二进制 float + 帧尾 00 00 80 7F
    VOFA_MODE_FIREWATER, // 文本 "1.23,4.56,...\n"
    VOFA_MODE_TOTAL,
} VofaMode_eTypeDef;

void Vofa_Init(void);
void Vofa_Poll(void);

/**
 * @brief  切换输出模式
 * @retval 0:成功  1:模式非法
 */
uint8_t Vofa_SetMode(VofaMode_eTypeDef mode);

/**
 * @brief  设置分频：每 div 个控制周期发一帧
 * @retval 0:成功  1:超出 VOFA_DIV_MIN~VOFA_DIV_MAX
 */
uint8_t Vofa_SetDiv(uint8_t div);

VofaMode_eTypeDef Vofa_GetMode(void);
uint8_t           Vofa_GetDiv(void);
uint32_t          Vofa_GetDropCount(void); // 发送缓冲满而丢掉的帧数
const char       *Vofa_GetModeName(void);
const char       *Vofa_GetChannelName(uint8_t index);

#endif
