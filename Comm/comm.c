/**
 * @file    comm.c
 * @brief   通信层门面：初始化与主循环轮询
 */

#include "comm.h"
#include "comm_echo.h"
#include "comm_pack.h"
#include "comm_port.h"
#include "comm_receive.h"
#include "comm_send.h"

#if COMM_CLI_ENABLE
#include "cli.h"
#endif

/**
 * @brief  初始化通信层
 * @note   必须在 MX_USART3_UART_Init() 之后调用；调用后 USART3 接收中断即开始工作
 */
void Comm_Init(void)
{
    CommPort_Init();
    CommSend_Init();
    CommEcho_Init();

#if COMM_CLI_ENABLE
    Cli_Init();
#endif

    /* 先事件后横幅：事件那行会被打成 "[ev] boot ok"，排在横幅上面像一条日志；
     * 顺带把 OLED 的事件行也刷成 boot ok */
    CommEcho_SetEvent("boot ok");

#if COMM_CLI_ENABLE
    Cli_Banner();
#endif
}

/**
 * @brief  通信层轮询，放在主循环里每圈调用
 * @note   协议解析全部在这里(线程态)完成，中断里只搬字节。这样命令处理函数
 *         可以放心读写 PID 参数、Target_Speed 等控制变量，不会和 Control() 打架。
 */
void Comm_Poll(void)
{
    uint8_t byte;
    uint8_t budget = COMM_RX_POLL_BUDGET;

    while ((budget > 0U) && (0U != CommPort_RxPop(&byte)))
    {
        budget--;

        /* 帧外字节的去向：可见字符是人在敲命令，其余当旧的单字节遥控码。
         * 两者取值范围不重叠(遥控码 0x00~0x09，可见字符 0x20~0x7E)，所以
         * 手机 App 和串口终端可以同时连着而互不干扰。 */
        if (0U == CommPack_RxByte(byte))
        {
#if COMM_CLI_ENABLE
            if (0U != Cli_IsTextByte(byte))
            {
                Cli_RxByte(byte);
                continue;
            }
#endif

#if COMM_LEGACY_BYTE_CMD_ENABLE
            CommReceive_LegacyByte(byte);
#endif
        }
    }

    CommPack_Poll(); // 半截帧超时丢弃，否则杂散的 0x3C 会让 CLI 永久失声
    CommSend_Poll();
    CommPort_TxKick();
}
