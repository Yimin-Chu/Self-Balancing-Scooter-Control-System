#ifndef __PID_H__
#define __PID_H__

#include "stm32f1xx_hal.h"

/* ??????(ms)???????????? */
#define CMD_TIMEOUT_MS  350

extern volatile uint32_t last_bt_cmd_tick;

void Control(void);

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

#endif
