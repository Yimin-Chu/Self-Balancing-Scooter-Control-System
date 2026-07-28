/**
 * @file    comm_protocol.h
 * @brief   蓝牙串口通信协议定义：帧格式 / 命令ID / 载荷结构
 *
 * 帧格式参考追觅 P1917 协议(详见 comm_protocol_readme.md)：
 *
 *   0x3C | 转义( len | cmd | data[len] | crcH | crcL ) | 0x3E
 *
 *   len : data 段字节数，不含 len/cmd/crc 自身
 *   cmd : 命令ID，下行见 ReceiveCmd_eTypeDef，上行见 SendId_eTypeDef
 *   crc : CRC16-MODBUS，覆盖 [len cmd data]，高字节先发
 *   转义: 数据区(不含帧头帧尾)出现 0x3C/0x3E/0x3F 时，在其前插入 0x3F
 *
 * 载荷内多字节字段一律小端，与 STM32 本机字节序一致。
 */

#ifndef __COMM_PROTOCOL_H__
#define __COMM_PROTOCOL_H__

#include "stm32f1xx_hal.h"

/* Define --------------------------------------------------------------------*/
#define COMM_PROTOCOL_VERSION_MAJOR (1)
#define COMM_PROTOCOL_VERSION_MINOR (0)
#define COMM_PROTOCOL_VERSION_PATCH (0)

#define FRAME_HEAD_CHAR   (0X3C)
#define FRAME_END_CHAR    (0X3E)
#define FRAME_ESCAPE_CHAR (0X3F)

/* 单帧 data 段上限。留够 SEND_ID_PID(28 字节)并有余量，同时限制 RAM 占用 */
#define COMM_PAYLOAD_LEN_MAX (48U)

/* 自动上报内容位图，配合 REV_CMD_REPORT_CFG 使用 */
#define COMM_REPORT_ATTITUDE (1U << 0)
#define COMM_REPORT_MOTION   (1U << 1)
#define COMM_REPORT_STATUS   (1U << 2)

/* 下行命令ID：手机/上位机 -> 小车 */
typedef enum
{
    REV_CMD_MOVE = 0,   // 0X00 运动控制
    REV_CMD_STOP,       // 0X01 紧急停止
    REV_CMD_MOTOR_EN,   // 0X02 电机使能
    REV_CMD_PID_SET,    // 0X03 PID 参数设置
    REV_CMD_PID_GET,    // 0X04 PID 参数查询
    REV_CMD_MED_ANGLE,  // 0X05 平衡中值角设置
    REV_CMD_REPORT_CFG, // 0X06 自动上报配置
    REV_CMD_HEARTBEAT,  // 0X07 心跳
    REV_CMD_VERSION,    // 0X08 版本查询
    REV_CMD_TOTAL,
} ReceiveCmd_eTypeDef;

/* 上行命令ID：小车 -> 手机/上位机 */
typedef enum
{
    SEND_ID_ACK = 0,   // 0X00 命令应答
    SEND_ID_ATTITUDE,  // 0X01 姿态上报
    SEND_ID_MOTION,    // 0X02 运动上报
    SEND_ID_STATUS,    // 0X03 状态上报
    SEND_ID_PID,       // 0X04 PID 参数上报
    SEND_ID_VERSION,   // 0X05 版本上报
    SEND_ID_TOTAL,
} SendId_eTypeDef;

/* 接收处理结果，随 SEND_ID_ACK 回给上位机 */
typedef enum
{
    REV_ERROR_NONE = 0, // 0:正常
    REV_ERROR_CRC,      // 1:校验失败
    REV_ERROR_FRAME,    // 2:帧结构错误(转义非法)
    REV_ERROR_OVER_NUM, // 3:帧长超限
    REV_ERROR_INVALID,  // 4:无效命令
    REV_ERROR_LEN,      // 5:载荷长度不对
    REV_ERROR_PARA,     // 6:参数越界
    REV_ERROR_TOTAL,
} revError_eTypeDef;

/* 发送结果 */
typedef enum
{
    PACK_ERROR_NONE = 0, // 0:已写入发送缓冲
    PACK_ERROR_LEN,      // 1:载荷超过 COMM_PAYLOAD_LEN_MAX
    PACK_ERROR_PARA,     // 2:入参非法
    PACK_ERROR_FULL,     // 3:发送缓冲已满，整帧丢弃
} packErrorType;

/* REV_CMD_PID_SET 的环路选择 */
typedef enum
{
    COMM_PID_VERTICAL = 0, // 0:直立环(PD, ki 忽略)
    COMM_PID_VELOCITY,     // 1:速度环(PI, kd 忽略)
    COMM_PID_TURN,         // 2:转向环(PD, ki 忽略)
    COMM_PID_TOTAL,
} CommPidLoop_eTypeDef;

/* Payload -------------------------------------------------------------------*/
#pragma pack(1)

/* 0X00 REV_CMD_MOVE：3 字节 */
typedef struct
{
    int8_t  speed;  // -100~100，前进为正。当前只取符号(见 comm_receive.c 说明)
    int8_t  turn;   // -100~100，右转为正
    uint8_t enable; // 0:松开(目标归零) 1:使能
} CommMove_tTypeDef;

/* 0X02 REV_CMD_MOTOR_EN：1 字节 */
typedef struct
{
    uint8_t enable; // 0:PWM 输出恒为 0 1:输出 Control() 计算值
} CommMotorEn_tTypeDef;

/* 0X03 REV_CMD_PID_SET：13 字节 */
typedef struct
{
    uint8_t loop; // CommPidLoop_eTypeDef
    float   kp;
    float   ki;
    float   kd;
} CommPidSet_tTypeDef;

/* 0X05 REV_CMD_MED_ANGLE：4 字节 */
typedef struct
{
    float med_angle; // 平衡中值角(度)，允许范围 ±COMM_MED_ANGLE_ABS_MAX
} CommMedAngle_tTypeDef;

/* 0X06 REV_CMD_REPORT_CFG：3 字节 */
typedef struct
{
    uint8_t  period_10ms; // 上报周期，单位 10ms；0 表示关闭自动上报
    uint16_t mask;        // COMM_REPORT_xxx 位图
} CommReportCfg_tTypeDef;

/* 0X00 SEND_ID_ACK：2 字节 */
typedef struct
{
    uint8_t cmd;   // 被应答的下行命令ID，0XFF 表示帧层错误(命令ID不可信)
    uint8_t error; // revError_eTypeDef
} CommAck_tTypeDef;

/* 0X01 SEND_ID_ATTITUDE：12 字节 */
typedef struct
{
    int16_t roll_x100;  // 横滚角 × 100(平衡小车的控制角)
    int16_t pitch_x100; // 俯仰角 × 100
    int16_t yaw_x100;   // 偏航角 × 100
    int16_t gyrox;      // 已扣除零偏的角速度原始值
    int16_t gyroy;
    int16_t gyroz;
} CommAttitude_tTypeDef;

/* 0X02 SEND_ID_MOTION：12 字节 */
typedef struct
{
    int16_t encoder_l;
    int16_t encoder_r;
    int16_t target_speed;
    int16_t target_turn;
    int16_t moto1; // 限幅后的 PWM 值，与实际输出可能不同(见 motor_enable)
    int16_t moto2;
} CommMotion_tTypeDef;

/* 0X03 SEND_ID_STATUS：10 字节 */
typedef struct
{
    uint8_t  motor_enable;
    uint8_t  cmd_timeout;  // 1:超过 CMD_TIMEOUT_MS 没收到运动指令
    int16_t  gyrox_offset;
    uint16_t distance_mm;  // 超声波测距
    uint32_t tick_ms;      // HAL_GetTick()
} CommStatus_tTypeDef;

/* 0X04 SEND_ID_PID：28 字节 */
typedef struct
{
    float vertical_kp;
    float vertical_kd;
    float velocity_kp;
    float velocity_ki;
    float turn_kp;
    float turn_kd;
    float med_angle;
} CommPid_tTypeDef;

/* 0X05 SEND_ID_VERSION：3 字节 */
typedef struct
{
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
} CommVersion_tTypeDef;

#pragma pack()

/* 编译期校验载荷长度与文档一致。结构体一旦被意外对齐填充，协议就会静默错位，
 * 这种问题在串口上极难查，所以在这里直接编译报错拦住 */
_Static_assert(sizeof(CommMove_tTypeDef) == 3U, "CommMove_tTypeDef must be 3 bytes");
_Static_assert(sizeof(CommPidSet_tTypeDef) == 13U, "CommPidSet_tTypeDef must be 13 bytes");
_Static_assert(sizeof(CommReportCfg_tTypeDef) == 3U, "CommReportCfg_tTypeDef must be 3 bytes");
_Static_assert(sizeof(CommAttitude_tTypeDef) == 12U, "CommAttitude_tTypeDef must be 12 bytes");
_Static_assert(sizeof(CommMotion_tTypeDef) == 12U, "CommMotion_tTypeDef must be 12 bytes");
_Static_assert(sizeof(CommStatus_tTypeDef) == 10U, "CommStatus_tTypeDef must be 10 bytes");
_Static_assert(sizeof(CommPid_tTypeDef) == 28U, "CommPid_tTypeDef must be 28 bytes");

#endif
