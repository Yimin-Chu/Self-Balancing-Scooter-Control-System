#ifndef __PID_H__
#define __PID_H__

#include "stm32f1xx_hal.h"

/* ??????(ms)???????????? */
#define CMD_TIMEOUT_MS  350

extern volatile uint32_t last_bt_cmd_tick;

void Control(void);

#endif
