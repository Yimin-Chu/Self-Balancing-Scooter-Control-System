/**
 * @file    comm_send.c
 * @brief   应用层上行：状态上报与周期上报调度
 *
 * 对应追觅工程的 ComProtocol/send.c：每种上报内容一个打包函数，统一走
 * send2app(id, 载荷地址, 载荷长度) 交给帧层。
 *
 * 所有上报函数都只能在主循环调用(帧层的 txBuf 是共享静态缓冲，见 comm_pack.c)。
 */

#include "comm_send.h"
#include "comm_pack.h"
#include "pid.h"

/* Datas ---------------------------------------------------------------------*/
static CommReportCfg_tTypeDef commReportCfg;
static uint32_t               reportTick;

/* pid.c 的传感器与控制量 */
extern float pitch, roll, yaw;
extern short gyrox, gyroy, gyroz;
extern int   Encoder_Left, Encoder_Right;
extern int   Target_Speed, Target_turn, MOTO1, MOTO2;
extern int   gyrox_offset;
extern float Med_Angle;
extern float Vertical_Kp, Vertical_Kd;
extern float Velocity_Kp, Velocity_Ki;
extern float Turn_Kp, Turn_Kd;
extern uint8_t motor_enable;

/* sr04.c 的超声波测距，单位 cm */
extern float distance;

/* Function ------------------------------------------------------------------*/
static int16_t commSend_toInt16(float value);

/**
 * @brief  初始化上报配置
 */
void CommSend_Init(void)
{
    commReportCfg.period_10ms = COMM_REPORT_PERIOD_DEFAULT;
    commReportCfg.mask        = COMM_REPORT_MASK_DEFAULT;

    reportTick = HAL_GetTick();
}

/**
 * @brief  当前是否正在周期上报
 * @retval 1:上报开着(终端上会是乱码)  0:已关闭
 */
uint8_t CommSend_IsReporting(void)
{
    return ((0U != commReportCfg.period_10ms) && (0U != commReportCfg.mask)) ? 1U : 0U;
}

/**
 * @brief  周期上报调度，由 Comm_Poll() 每圈主循环调用
 */
void CommSend_Poll(void)
{
    uint32_t now = HAL_GetTick();

    if (0U == commReportCfg.period_10ms)
    {
        return;
    }

    if ((now - reportTick) < ((uint32_t)commReportCfg.period_10ms * 10U))
    {
        return;
    }

    reportTick = now;

    if (0U != (commReportCfg.mask & COMM_REPORT_ATTITUDE))
    {
        CommSend_Attitude();
    }

    if (0U != (commReportCfg.mask & COMM_REPORT_MOTION))
    {
        CommSend_Motion();
    }

    if (0U != (commReportCfg.mask & COMM_REPORT_STATUS))
    {
        CommSend_Status();
    }
}

/**
 * @brief  设置自动上报周期与内容
 * @param  period_10ms 上报周期，单位 10ms；0 表示关闭自动上报
 * @param  mask        COMM_REPORT_xxx 位图
 * @retval 0:设置成功  1:参数非法
 */
uint8_t CommSend_SetReportCfg(uint8_t period_10ms, uint16_t mask)
{
    const uint16_t maskValid = COMM_REPORT_ATTITUDE | COMM_REPORT_MOTION | COMM_REPORT_STATUS;

    if ((0U != period_10ms) && (period_10ms < COMM_REPORT_PERIOD_MIN))
    {
        return 1U;
    }

    if (0U != (mask & (uint16_t)(~maskValid)))
    {
        return 1U;
    }

    commReportCfg.period_10ms = period_10ms;
    commReportCfg.mask        = mask;
    reportTick                = HAL_GetTick();

    return 0U;
}

/**
 * @brief  0X00 命令应答
 * @param  cmd   被应答的下行命令ID，0XFF 表示帧层错误
 * @param  error revError_eTypeDef
 */
void CommSend_Ack(uint8_t cmd, uint8_t error)
{
    CommAck_tTypeDef ack;

    ack.cmd   = cmd;
    ack.error = error;

    send2app(SEND_ID_ACK, &ack, sizeof(ack));
}

/**
 * @brief  0X01 姿态上报
 */
void CommSend_Attitude(void)
{
    CommAttitude_tTypeDef att;

    att.roll_x100  = commSend_toInt16(roll * 100.0f);
    att.pitch_x100 = commSend_toInt16(pitch * 100.0f);
    att.yaw_x100   = commSend_toInt16(yaw * 100.0f);
    att.gyrox      = (int16_t)gyrox;
    att.gyroy      = (int16_t)gyroy;
    att.gyroz      = (int16_t)gyroz;

    send2app(SEND_ID_ATTITUDE, &att, sizeof(att));
}

/**
 * @brief  0X02 运动上报
 */
void CommSend_Motion(void)
{
    CommMotion_tTypeDef motion;

    motion.encoder_l    = (int16_t)Encoder_Left;
    motion.encoder_r    = (int16_t)Encoder_Right;
    motion.target_speed = (int16_t)Target_Speed;
    motion.target_turn  = (int16_t)Target_turn;
    motion.moto1        = (int16_t)MOTO1;
    motion.moto2        = (int16_t)MOTO2;

    send2app(SEND_ID_MOTION, &motion, sizeof(motion));
}

/**
 * @brief  0X03 状态上报
 */
void CommSend_Status(void)
{
    CommStatus_tTypeDef status;
    float               distance_mm = distance * 10.0f;

    status.motor_enable = motor_enable;
    status.cmd_timeout  = ((HAL_GetTick() - last_bt_cmd_tick) > CMD_TIMEOUT_MS) ? 1U : 0U;
    status.gyrox_offset = (int16_t)gyrox_offset;
    status.distance_mm  = (distance_mm > 65535.0f) ? 65535U : (uint16_t)distance_mm;
    status.tick_ms      = HAL_GetTick();

    send2app(SEND_ID_STATUS, &status, sizeof(status));
}

/**
 * @brief  0X04 PID 参数上报
 */
void CommSend_Pid(void)
{
    CommPid_tTypeDef pid;

    pid.vertical_kp = Vertical_Kp;
    pid.vertical_kd = Vertical_Kd;
    pid.velocity_kp = Velocity_Kp;
    pid.velocity_ki = Velocity_Ki;
    pid.turn_kp     = Turn_Kp;
    pid.turn_kd     = Turn_Kd;
    pid.med_angle   = Med_Angle;

    send2app(SEND_ID_PID, &pid, sizeof(pid));
}

/**
 * @brief  0X05 版本上报
 */
void CommSend_Version(void)
{
    CommVersion_tTypeDef version;

    version.major = COMM_PROTOCOL_VERSION_MAJOR;
    version.minor = COMM_PROTOCOL_VERSION_MINOR;
    version.patch = COMM_PROTOCOL_VERSION_PATCH;

    send2app(SEND_ID_VERSION, &version, sizeof(version));
}

/**
 * @brief  浮点转 int16 并饱和限幅
 * @note   不限幅的话超范围转换是未定义行为，上位机会看到莫名其妙的跳变
 */
static int16_t commSend_toInt16(float value)
{
    if (value > 32767.0f)
    {
        return 32767;
    }

    if (value < -32768.0f)
    {
        return -32768;
    }

    return (int16_t)value;
}
