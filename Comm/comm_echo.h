/**
 * @file    comm_echo.h
 * @brief   命令回显缓存：给 OLED 提供"最近收到了什么"的快照
 *
 * 解决的问题：串口那头看得见回显，但小车这边看不见。
 * 通信层(CLI / 协议帧 / 单字节遥控码)只管往这里写字符串，
 * 显示层(main.c)只管读，两边互不依赖，OLED 换成别的屏也不用改通信代码。
 *
 * 两条独立的行：
 *   input  正在输入的那行命令，逐字符更新，用来确认"每个字符都收到了"
 *   event  最近一次事件的结果，用来确认"命令被执行了"
 *
 * dirty 标志避免无变化时空刷屏 —— OLED 刷一行要十几毫秒，很贵。
 */

#ifndef __COMM_ECHO_H__
#define __COMM_ECHO_H__

#include <stdint.h>

/* OLED 一行 16 个字符槽，input 行要留一个给提示符 '>' */
#define COMM_ECHO_LEN_MAX (16U)

/* 1:每条事件同时往串口打一行  0:只上 OLED
 * 手机 App 高频下发命令时可以关掉，省带宽 */
#define COMM_ECHO_UART_ENABLE (1)
#define COMM_ECHO_UART_PREFIX "[ev] "

void CommEcho_Init(void);

/* 写入正在输入的命令行(超长自动截取尾部，跟着光标走) */
void CommEcho_SetInput(const char *str);

/* 写入最近一次事件，printf 风格，超长截断。
 * 内容相同时不重画 OLED，但串口那一行每次都打——连按两次 start 要看得见两行回执 */
void CommEcho_SetEvent(const char *fmt, ...);

/* 读取 + 清脏位：返回 1 表示内容有更新、需要重画 */
uint8_t CommEcho_TakeInputDirty(void);
uint8_t CommEcho_TakeEventDirty(void);

const char *CommEcho_GetInput(void);
const char *CommEcho_GetEvent(void);

#endif
