#ifndef __PID_H__
#define __PID_H__

#include "stm32f1xx_hal.h"

/* ??????(ms)???????????? */
#define CMD_TIMEOUT_MS  350

extern volatile uint32_t last_bt_cmd_tick;

void Control(void);

/* 开机静置标定平衡中值角(在 main 初始化阶段调用，实现见 pid.c)。阻塞约 20s */
void Calibrate_Med_Angle(void);

/* 直接套用一组已有的标定值(例如从 Flash 读回来的)，免去 20s 静置等待 */
void Calibrate_Apply(float med_angle, int gyrox_off);

/* 标定结果，CLI 与 Flash 存储都要读写 */
extern float Med_Angle;
extern int   gyrox_offset;

/* 电机输出总闸。0 时 PWM 恒为 0，只做计算不驱动 */
extern uint8_t motor_enable;

/* ---------------------------------------------------------------------------
 * 手动速度目标：给 VOFA+ 滑块 / CLI 设速度环阶跃用
 *
 * 注意不要绕过这组接口去直接写 Target_Speed —— Control() 每一拍都会按遥控输入
 * 重算它，直接写进去的值活不过一个控制周期(10ms)就被覆盖了。这里存的是一份独立
 * 的目标值，Control() 会在遥控分支之前优先取用。
 *
 * 开机默认就在 manual（目标 0）；start/stop/看门狗只清目标，不退出 manual。
 * 只有 Manual_Speed_Off() / spd off 才交还给蓝牙遥控。
 * 生效期间转向目标被强制为 0，免得调速度环时转向环掺进来。
 * ------------------------------------------------------------------------- */
#define SPEED_Y                  30       /* 速度目标限幅，正负对称 */
#define MANUAL_SPEED_TIMEOUT_MS  10000U   /* 多久没有新设定就把目标清 0（仍留在 manual） */

void    Manual_Speed_Set(int speed);  /* 设定并进入/保持手动模式，超出 ±SPEED_Y 会被夹住 */
void    Manual_Speed_Off(void);       /* 退出手动模式，目标归零并交还给遥控 */
uint8_t Manual_Speed_IsOn(void);
int     Manual_Speed_Get(void);       /* 当前手动目标(已夹幅) */

/* ---------------------------------------------------------------------------
 * IMU 数据就绪信号 (P0: 把 Control() 移出中断上下文)
 *
 * 现在(裸机): 用 volatile 标志 + 时间戳，在 main 主循环里轮询消费。
 * 以后(RTOS): 只需把这两个函数体换成二值信号量，调用点(sr04.c / main.c)不变：
 *   - Imu_DataReady_FromISR() -> xSemaphoreGiveFromISR(imuSem, &woken)
 *   - Imu_ControlPending()    -> xSemaphoreTake(imuSem, portMAX_DELAY)
 * ------------------------------------------------------------------------- */
void     Imu_DataReady_FromISR(void);   /* EXTI 回调里调用：仅置位，立即返回     */
uint8_t  Imu_ControlPending(void);      /* 主循环里调用：有新帧返回1并清标志       */

extern volatile uint32_t imu_last_ready_tick;  /* 最近一次数据就绪的 tick(用于超时) */

/* 已完成的控制周期数，每跑完一次 Control() 自增。
 * 供通信层给波形输出分频用，比按 HAL_GetTick() 分频得到的采样间隔更均匀 */
extern volatile uint32_t control_cycle;

#endif
