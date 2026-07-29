/**
 * @file    comm_port.c
 * @brief   传输层：USART3(蓝牙透传) 收发环形缓冲
 *
 * 对应追觅工程里的 task_com.c，只是那边靠 FreeRTOS 队列 + DMA，
 * 本工程是裸机超循环，所以换成"中断填环形缓冲 + 主循环消费"：
 *
 *   接收: USART3 中断 -> HAL_UART_RxCpltCallback -> 压入 rxRing -> 主循环 CommPort_RxPop
 *   发送: CommPack_Send -> CommPort_TxPushFrame -> txRing -> 中断逐段 HAL_UART_Transmit_IT
 *
 * 为什么发送不能直接用阻塞的 HAL_UART_Transmit：
 * 即使 115200bps，阻塞发一帧仍可能挤占平衡控制周期，所以发送一律走中断，
 * 主循环只负责往缓冲里塞。
 */

#include "comm_port.h"
#include "usart.h"

/* Define --------------------------------------------------------------------*/
/* 两个缓冲大小都必须是 2 的幂，下面用位与代替取模。
 * 接收缓冲 256 字节按"主循环最长一次卡多久"算：115200bps 每毫秒进 11.5 字节，
 * 而 OLED 刷一行要十几毫秒、Flash 擦写(cali 命令)要几十毫秒，这期间没人来取字节。
 * 256 字节约等于 22ms 的余量；原来的 64 字节只够 5.5ms，粘一段长命令就溢出。
 * 发送缓冲 512 字节是为了装得下 CLI 的多行输出(help/status 一次约 200 字节)，
 * 115200bps 下约 45ms 发完；缓冲小了后面几行会被整块丢弃 */
#define COMM_RX_RING_SIZE (256U)
#define COMM_TX_RING_SIZE (512U)
#define COMM_RX_RING_MASK (COMM_RX_RING_SIZE - 1U)
#define COMM_TX_RING_MASK (COMM_TX_RING_SIZE - 1U)

/* Datas ---------------------------------------------------------------------*/
CommPortStat_tTypeDef commPortStat;

/* rxHead 只由中断写，rxTail 只由主循环写；txHead 反之。单生产者单消费者，
 * 索引加 volatile 即可，无需临界区。 */
static uint8_t           rxRing[COMM_RX_RING_SIZE];
static volatile uint16_t rxHead, rxTail;

static uint8_t           txRing[COMM_TX_RING_SIZE];
static volatile uint16_t txHead, txTail;
static volatile uint16_t txBusyLen; // 本次提交给 HAL 的字节数，0 表示发送空闲

static uint8_t rxByte; // HAL 单字节接收落点

/* Function ------------------------------------------------------------------*/
static uint16_t CommPort_TxFree(void);
static void     CommPort_TxStart(void);

/**
 * @brief  初始化收发缓冲并武装接收中断
 * @note   必须在 MX_USART3_UART_Init() 之后调用
 */
void CommPort_Init(void)
{
    rxHead = rxTail = 0U;
    txHead = txTail = 0U;
    txBusyLen       = 0U;

    commPortStat.rxDrop      = 0U;
    commPortStat.txDropFrame = 0U;
    commPortStat.uartError   = 0U;

    HAL_UART_Receive_IT(&huart3, &rxByte, 1U);
}

/**
 * @brief  从接收缓冲取一个字节
 * @param  pByte 取出的字节
 * @retval 1:取到  0:缓冲为空
 */
uint8_t CommPort_RxPop(uint8_t *const pByte)
{
    if (rxHead == rxTail)
    {
        return 0U;
    }

    *pByte = rxRing[rxTail];
    rxTail = (uint16_t)((rxTail + 1U) & COMM_RX_RING_MASK);

    return 1U;
}

/**
 * @brief  整块写入发送缓冲(协议帧或 CLI 文本都走这里)
 * @param  pData 数据首地址
 * @param  len   数据长度
 * @retval 1:已全部写入  0:剩余空间不足，整块丢弃
 * @note   要么整块写入要么整块丢弃：半帧上线只会让对端收到坏帧，还得等超时重同步；
 *         CLI 输出同理，宁可整行不打印也不要打印半行
 */
uint8_t CommPort_TxPush(const uint8_t *pData, uint16_t len)
{
    uint16_t i;

    if ((NULL == pData) || (0U == len))
    {
        return 0U;
    }

    if (CommPort_TxFree() < len)
    {
        commPortStat.txDropFrame++;
        return 0U;
    }

    for (i = 0; i < len; i++)
    {
        txRing[txHead] = pData[i];
        txHead         = (uint16_t)((txHead + 1U) & COMM_TX_RING_MASK);
    }

    CommPort_TxStart();

    return 1U;
}

/**
 * @brief  兜底启动发送，供主循环轮询调用
 * @note   正常情况下 CommPort_TxPushFrame 里已经启动过，这里只是防止某次
 *         HAL_UART_Transmit_IT 因 UART 忙而没启动成功导致缓冲一直卡住
 */
void CommPort_TxKick(void)
{
    CommPort_TxStart();
}

/**
 * @brief  发送缓冲剩余可写字节数
 * @note   环形缓冲留 1 字节空位区分"满"和"空"，所以是 SIZE-1
 */
static uint16_t CommPort_TxFree(void)
{
    uint16_t head = txHead;
    uint16_t tail = txTail;

    return (uint16_t)((COMM_TX_RING_SIZE - 1U) - ((head - tail) & COMM_TX_RING_MASK));
}

/**
 * @brief  若发送空闲且缓冲有数据，提交一段连续数据给 UART 中断发送
 * @note   txBusyLen 与 HAL 状态的检查+提交必须原子完成：否则 TxCplt 中断可能在
 *         检查和提交之间插进来同时启动一次发送，同一段数据会被发两遍
 */
 //If the UART truck is free, load the next continuous group of bytes onto it and start transmission.
static void CommPort_TxStart(void)
{
    uint32_t primask = __get_PRIMASK();
    uint16_t head, len;

    __disable_irq();

    if ((0U == txBusyLen) && (txHead != txTail) && (HAL_UART_STATE_READY == huart3.gState))
    {
        head = txHead;
        /* 只发到缓冲末尾为止，绕回的部分留给下一次中断 */
        len  = (head > txTail) ? (uint16_t)(head - txTail) : (uint16_t)(COMM_TX_RING_SIZE - txTail);

        txBusyLen = len;

        if (HAL_OK != HAL_UART_Transmit_IT(&huart3, &txRing[txTail], len))
        {
            txBusyLen = 0U;
        }
    }

    __set_PRIMASK(primask);
}

/**
 * @brief  UART 发送完成回调：推进尾指针并接着发下一段
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (USART3 != huart->Instance)
    {
        return;
    }

    txTail    = (uint16_t)((txTail + txBusyLen) & COMM_TX_RING_MASK);
    txBusyLen = 0U;

    CommPort_TxStart();
}

/**
 * @brief  UART 接收完成回调：压入环形缓冲并重新武装接收
 * @note   这里只搬字节，协议解析放到主循环(Comm_Poll)做，中断里不碰控制变量
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t next;

    if (USART3 != huart->Instance)
    {
        return;
    }

    next = (uint16_t)((rxHead + 1U) & COMM_RX_RING_MASK);

    if (next != rxTail)
    {
        rxRing[rxHead] = rxByte;
        rxHead         = next;
    }
    else
    {
        commPortStat.rxDrop++;
    }

    HAL_UART_Receive_IT(&huart3, &rxByte, 1U);
}

/**
 * @brief  UART 错误回调：清标志并重新武装接收
 * @note   必须实现。一旦发生 ORE(接收溢出)，HAL 会退出接收态，
 *         不在这里重新 Receive_IT 的话蓝牙就此彻底失联，只能靠复位恢复
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (USART3 != huart->Instance)
    {
        return;
    }

    commPortStat.uartError++;

    __HAL_UART_CLEAR_OREFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    if (HAL_UART_STATE_BUSY_RX != huart->RxState)
    {
        HAL_UART_Receive_IT(huart, &rxByte, 1U);
    }
}
