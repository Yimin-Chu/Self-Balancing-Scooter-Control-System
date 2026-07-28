/**
 * @file    comm_pack.c
 * @brief   帧层：0x3C/0x3E 定界 + 0x3F 转义 + CRC16 的组帧与解帧
 *
 * 对应追觅工程的 Package/pack.c + Package/package.c，帧格式完全一致：
 *
 *   0x3C | 转义( len | cmd | data[len] | crcH | crcL ) | 0x3E
 *
 * 转义规则保证了 0x3C/0x3E 在帧内绝不出现，因此接收端任何时候看到 0x3C
 * 都能立刻重新对齐帧头，不用等超时——这是这套格式最实用的地方。
 */

#include "comm_pack.h"
#include "comm_port.h"
#include "comm_receive.h"
#include "comm_send.h"
#include "crc16.h"

/* Define --------------------------------------------------------------------*/
/* 解帧缓冲存的是"去掉转义之后"的内容：len + cmd + data + crc16 */
#define COMM_RX_FRAME_LEN_MAX (COMM_PAYLOAD_LEN_MAX + 4U)
/* 组帧缓冲要按最坏情况(每个字节都需要转义)算，再加帧头帧尾 */
#define COMM_TX_FRAME_LEN_MAX (COMM_RX_FRAME_LEN_MAX * 2U + 2U)

typedef enum
{
    PACK_STA_START = 0, // 帧外，等待帧头 0x3C
    PACK_STA_ING,       // 帧内收数据
    PACK_STA_ESCAPE,    // 收到 0x3F，下一字节按原值存
} CommPackSta_eTypeDef;

/* Datas ---------------------------------------------------------------------*/
CommPackStat_tTypeDef commPackStat;

static uint8_t rxData[COMM_RX_FRAME_LEN_MAX];
static uint8_t txBuf[COMM_TX_FRAME_LEN_MAX];

/* Function ------------------------------------------------------------------*/
static void     commPack_reportError(uint8_t cmdId, revError_eTypeDef revError);
static void     commPack_dispatch(uint8_t *pFrame, uint8_t len);
static uint16_t commPack_fillEscape(uint16_t point, uint8_t byte);
static uint16_t commPack_fillEscapeCrc(uint16_t point, uint8_t byte, uint8_t *const pCrc);

/**
 * @brief  按字节喂给解帧状态机
 * @param  byte 收到的一个字节
 * @retval 1:该字节被帧解析器消费  0:帧外的无关字节
 * @note   返回 0 是留给旧的单字节遥控码用的：手机 App 还没升级到新协议时，
 *         0x01/0x03/0x05/0x07/0x09 这类裸字节会落到这里，由 Comm_Poll 转交
 *         CommReceive_LegacyByte() 处理，新旧协议可以并存。
 */
uint8_t CommPack_RxByte(uint8_t byte)
{
    static CommPackSta_eTypeDef state    = PACK_STA_START;
    static uint8_t              bufPoint = 0;

    switch (state)
    {
    case PACK_STA_START:
        if (FRAME_HEAD_CHAR != byte)
        {
            return 0U;
        }
        state    = PACK_STA_ING;
        bufPoint = 0U;
        break;

    case PACK_STA_ING:
        if (FRAME_END_CHAR == byte)
        {
            state = PACK_STA_START;
            commPack_dispatch(rxData, bufPoint);
        }
        else if (FRAME_ESCAPE_CHAR == byte)
        {
            state = PACK_STA_ESCAPE;
        }
        else if (FRAME_HEAD_CHAR == byte)
        {
            bufPoint = 0U; // 上一帧没收完就来了新帧头，直接重新对齐
        }
        else if (bufPoint >= COMM_RX_FRAME_LEN_MAX)
        {
            commPackStat.frameError++;
            commPack_reportError(0XFFU, REV_ERROR_OVER_NUM);
            state = PACK_STA_START;
        }
        else
        {
            rxData[bufPoint++] = byte;
        }
        break;

    case PACK_STA_ESCAPE:
        /* 0x3F 后面只允许跟这三个值，否则说明数据流已经错位 */
        if ((FRAME_HEAD_CHAR != byte) && (FRAME_END_CHAR != byte) && (FRAME_ESCAPE_CHAR != byte))
        {
            commPackStat.frameError++;
            commPack_reportError(0XFFU, REV_ERROR_FRAME);
            state = PACK_STA_START;
        }
        else if (bufPoint >= COMM_RX_FRAME_LEN_MAX)
        {
            commPackStat.frameError++;
            commPack_reportError(0XFFU, REV_ERROR_OVER_NUM);
            state = PACK_STA_START;
        }
        else
        {
            rxData[bufPoint++] = byte;
            state              = PACK_STA_ING;
        }
        break;

    default:
        state = PACK_STA_START;
        break;
    }

    return 1U;
}

/**
 * @brief  组帧并写入发送缓冲
 * @param  cmdId  上行命令ID，见 SendId_eTypeDef
 * @param  pData  载荷首地址，无载荷时可传 NULL
 * @param  length 载荷长度，不超过 COMM_PAYLOAD_LEN_MAX
 * @retval packErrorType
 * @note   txBuf 是共享静态缓冲，本函数只允许在主循环(线程态)调用，不可在中断里调用
 */
packErrorType CommPack_Send(uint8_t cmdId, const void *pData, uint8_t length)
{
    const uint8_t *pByte  = (const uint8_t *)pData;
    uint8_t        crc[2] = {0XFFU, 0XFFU};
    uint16_t       point  = 0U;
    uint8_t        i;

    if (length > COMM_PAYLOAD_LEN_MAX)
    {
        return PACK_ERROR_LEN;
    }

    if ((length > 0U) && (NULL == pByte))
    {
        return PACK_ERROR_PARA;
    }

    txBuf[point++] = FRAME_HEAD_CHAR;

    point = commPack_fillEscapeCrc(point, length, crc);
    point = commPack_fillEscapeCrc(point, cmdId, crc);

    for (i = 0; i < length; i++)
    {
        point = commPack_fillEscapeCrc(point, pByte[i], crc);
    }

    /* CRC 高字节先发，与追觅 readme 里的 crcH8-crcL8 一致 */
    point = commPack_fillEscape(point, crc[1]);
    point = commPack_fillEscape(point, crc[0]);

    txBuf[point++] = FRAME_END_CHAR;

    return CommPort_TxPushFrame(txBuf, point) ? PACK_ERROR_NONE : PACK_ERROR_FULL;
}

/**
 * @brief  记录错误并回一帧 ACK
 * @param  cmdId 出错的命令ID，0XFF 表示帧层错误(命令ID 不可信)
 */
static void commPack_reportError(uint8_t cmdId, revError_eTypeDef revError)
{
    commPackStat.lastError = (uint8_t)revError;

    CommSend_Ack(cmdId, (uint8_t)revError);
}

/**
 * @brief  校验并分发一帧
 * @param  pFrame 去转义后的帧内容：len + cmd + data + crc16
 * @param  len    pFrame 的总长度
 */
static void commPack_dispatch(uint8_t *pFrame, uint8_t len)
{
    revError_eTypeDef revError;
    uint8_t           cmdId;

    if (crc16_check(pFrame, len))
    {
        commPackStat.crcError++;
        commPack_reportError(0XFFU, REV_ERROR_CRC);
        return;
    }

    /* 总长 = 1(len) + 1(cmd) + data + 2(crc)，对不上说明发送端算错了长度 */
    if (((uint16_t)pFrame[0] + 4U) != (uint16_t)len)
    {
        commPackStat.frameError++;
        commPack_reportError(pFrame[1], REV_ERROR_LEN);
        return;
    }

    cmdId = pFrame[1];

    if (cmdId >= REV_CMD_TOTAL)
    {
        commPackStat.cmdError++;
        commPack_reportError(cmdId, REV_ERROR_INVALID);
        return;
    }

    commPackStat.rxFrame++;
    commPackStat.lastCmd = cmdId;

    revError = cmdReceiveTab[cmdId](&pFrame[2], pFrame[0]);

    if (REV_ERROR_NONE != revError)
    {
        commPack_reportError(cmdId, revError);
    }
}

/**
 * @brief  往 txBuf 填一个字节，必要时前插转义符
 * @retval 填完之后的写指针
 */
static uint16_t commPack_fillEscape(uint16_t point, uint8_t byte)
{
    if ((FRAME_HEAD_CHAR == byte) || (FRAME_END_CHAR == byte) || (FRAME_ESCAPE_CHAR == byte))
    {
        txBuf[point++] = FRAME_ESCAPE_CHAR;
    }

    txBuf[point++] = byte;

    return point;
}

/**
 * @brief  填一个字节，同时把它累加进 CRC
 * @note   CRC 算的是转义前的原始字节，转义符本身不参与校验
 */
static uint16_t commPack_fillEscapeCrc(uint16_t point, uint8_t byte, uint8_t *const pCrc)
{
    crc16_dynaCal(byte, pCrc);

    return commPack_fillEscape(point, byte);
}
