/**
 * @file    comm_echo.c
 * @brief   命令回显缓存实现
 *
 * 所有写入口都在主循环里(Comm_Poll -> 帧解析 / CLI)，中断只负责往环形缓冲塞字节，
 * 不会碰这里的缓存，所以不需要临界区。
 */

#include "comm_echo.h"
#include "comm_port.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* Datas ---------------------------------------------------------------------*/
static char    echoInput[COMM_ECHO_LEN_MAX + 1U];
static char    echoEvent[COMM_ECHO_LEN_MAX + 1U];
static uint8_t echoInputDirty;
static uint8_t echoEventDirty;

/* Function ------------------------------------------------------------------*/
#if COMM_ECHO_UART_ENABLE
/**
 * @brief  把事件整行拼好一次性塞进发送缓冲
 * @note   必须拼成一整块再推：CommPort_TxPush 是"要么整块进要么整块丢"，
 *         分成前缀/正文/换行三次推的话，缓冲快满时会只进前缀、丢掉正文
 */
static void commEcho_toUart(const char *str)
{
    char line[COMM_ECHO_LEN_MAX + 12U];
    int  len = snprintf(line, sizeof(line), COMM_ECHO_UART_PREFIX "%s\r\n", str);

    if (len <= 0)
    {
        return;
    }

    if ((size_t)len >= sizeof(line))
    {
        len = (int)sizeof(line) - 1;
    }

    (void)CommPort_TxPush((const uint8_t *)line, (uint16_t)len);
}
#endif

void CommEcho_Init(void)
{
    echoInput[0]   = '\0';
    echoEvent[0]   = '\0';
    echoInputDirty = 1U;
    echoEventDirty = 1U;
}

void CommEcho_SetInput(const char *str)
{
    /* 让出一格给提示符 '>' */
    const size_t max = (size_t)COMM_ECHO_LEN_MAX - 1U;
    size_t       len;

    if (NULL == str)
    {
        return;
    }

    /* 输入超过一行时只显示尾部，视觉上跟着光标走 */
    len = strlen(str);
    if (len > max)
    {
        str += (len - max);
    }

    /* 内容没变就别置脏位，省掉一次十几毫秒的刷屏 */
    if (0 == strcmp(echoInput, str))
    {
        return;
    }

    strncpy(echoInput, str, max);
    echoInput[max] = '\0';
    echoInputDirty = 1U;
}

void CommEcho_SetEvent(const char *fmt, ...)
{
    char    tmp[COMM_ECHO_LEN_MAX + 1U];
    va_list ap;

    va_start(ap, fmt);
    (void)vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);

#if COMM_ECHO_UART_ENABLE
    commEcho_toUart(tmp);
#endif

    /* 串口那一行在上面已经打过了，这里的提前返回只跳过 OLED 重画 */
    if (0 == strcmp(echoEvent, tmp))
    {
        return;
    }

    (void)strcpy(echoEvent, tmp);
    echoEventDirty = 1U;
}

uint8_t CommEcho_TakeInputDirty(void)
{
    uint8_t dirty  = echoInputDirty;
    echoInputDirty = 0U;
    return dirty;
}

uint8_t CommEcho_TakeEventDirty(void)
{
    uint8_t dirty  = echoEventDirty;
    echoEventDirty = 0U;
    return dirty;
}

const char *CommEcho_GetInput(void)
{
    return echoInput;
}

const char *CommEcho_GetEvent(void)
{
    return echoEvent;
}
