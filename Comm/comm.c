/**
 * @file    comm.c
 * @brief   通信层门面：初始化与主循环轮询
 */

#include "comm.h"
#include "comm_pack.h"
#include "comm_port.h"
#include "comm_receive.h"
#include "comm_send.h"

/**
 * @brief  初始化通信层
 * @note   必须在 MX_USART3_UART_Init() 之后调用；调用后 USART3 接收中断即开始工作
 */
void Comm_Init(void)
{
    CommPort_Init();
    CommSend_Init();
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

        if (0U == CommPack_RxByte(byte))
        {
#if COMM_LEGACY_BYTE_CMD_ENABLE
            CommReceive_LegacyByte(byte);
#endif
        }
    }

    CommSend_Poll();
    CommPort_TxKick();
}
