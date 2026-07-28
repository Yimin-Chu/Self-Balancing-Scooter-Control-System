/**
 * @file    comm_receive.c
 * @brief   应用层下行：命令分发表与命令处理
 *
 * 对应追觅工程的 ComProtocol/receive.c：命令ID 当下标查函数指针表，
 * 每个处理函数只拿到"载荷首地址 + 载荷长度"，先校验长度再解析结构体。
 *
 * 所有处理函数都运行在主循环(线程态)，和 Control() 是同一个上下文，
 * 因此直接读写 Target_Speed / PID 参数这些控制变量没有竞态问题。
 */

#include "comm_receive.h"
#include "comm_pack.h"
#include "comm_send.h"
#include "math.h"

/* Datas ---------------------------------------------------------------------*/
/* 这几个变量原先定义在 stm32f1xx_it.c，现在归通信层管：
 * 它们本质是"遥控指令的解析结果"，放中断文件里既不好找也不好扩展。
 * pid.c 用 extern 引用，定义搬家不影响它。 */
uint8_t Fore, Back, Left, Right;
uint8_t Bluetooth_data;

/* pid.h 里声明的指令看门狗时间戳：Control() 靠它判断 CMD_TIMEOUT_MS 失联 */
volatile uint32_t last_bt_cmd_tick = 0;

/* pid.c 的控制变量 */
extern uint8_t stop;
extern uint8_t motor_enable;
extern float   Med_Angle;
extern float   Vertical_Kp, Vertical_Kd;
extern float   Velocity_Kp, Velocity_Ki;
extern float   Turn_Kp, Turn_Kd;

/* Function ------------------------------------------------------------------*/
static void              commReceive_release(void);
static uint8_t           commReceive_gainValid(float gain);
static revError_eTypeDef cmdReceive_move(uint8_t *pByte, uint8_t len);
static revError_eTypeDef cmdReceive_stop(uint8_t *pByte, uint8_t len);
static revError_eTypeDef cmdReceive_motorEn(uint8_t *pByte, uint8_t len);
static revError_eTypeDef cmdReceive_pidSet(uint8_t *pByte, uint8_t len);
static revError_eTypeDef cmdReceive_pidGet(uint8_t *pByte, uint8_t len);
static revError_eTypeDef cmdReceive_medAngle(uint8_t *pByte, uint8_t len);
static revError_eTypeDef cmdReceive_reportCfg(uint8_t *pByte, uint8_t len);
static revError_eTypeDef cmdReceive_heartbeat(uint8_t *pByte, uint8_t len);
static revError_eTypeDef cmdReceive_version(uint8_t *pByte, uint8_t len);

const FuncCmdReceive cmdReceiveTab[REV_CMD_TOTAL] = {
    cmdReceive_move,      // 0X00 运动控制
    cmdReceive_stop,      // 0X01 紧急停止
    cmdReceive_motorEn,   // 0X02 电机使能
    cmdReceive_pidSet,    // 0X03 PID 参数设置
    cmdReceive_pidGet,    // 0X04 PID 参数查询
    cmdReceive_medAngle,  // 0X05 平衡中值角设置
    cmdReceive_reportCfg, // 0X06 自动上报配置
    cmdReceive_heartbeat, // 0X07 心跳
    cmdReceive_version,   // 0X08 版本查询
};

/**
 * @brief  兼容旧的单字节遥控码
 * @param  byte 落在帧外的字节
 * @note   手机 App 升级到新协议之前，老的 0x01/0x03/0x05/0x07/0x09 还能用。
 *         等 App 切换完成，把 comm.h 里的 COMM_LEGACY_BYTE_CMD_ENABLE 改成 0
 *         就能彻底关掉这条路径——留着它的风险是任何噪声字节都可能被当成指令。
 */
void CommReceive_LegacyByte(uint8_t byte)
{
    Bluetooth_data   = byte;
    last_bt_cmd_tick = HAL_GetTick();

    switch (byte)
    {
    case 0X01: // 前进
        Fore = 1U, Back = 0U, Left = 0U, Right = 0U;
        break;

    case 0X05: // 后退
        Fore = 0U, Back = 1U, Left = 0U, Right = 0U;
        break;

    case 0X03: // 右转
        Fore = 0U, Back = 0U, Left = 0U, Right = 1U;
        break;

    case 0X07: // 左转
        Fore = 0U, Back = 0U, Left = 1U, Right = 0U;
        break;

    case 0X09: // 停止
        commReceive_release();
        stop = 1U;
        break;

    default: // 含 0x00 松开，以及任何未定义码
        commReceive_release();
        break;
    }
}

/**
 * @brief  松开所有方向：目标速度/转向由 Control() 归零
 */
static void commReceive_release(void)
{
    Fore  = 0U;
    Back  = 0U;
    Left  = 0U;
    Right = 0U;
}

/**
 * @brief  PID 参数合法性检查
 * @retval 1:合法  0:越界或 NaN
 * @note   写成 (fabsf(x) <= MAX) 而不是 !(fabsf(x) > MAX)，是为了让 NaN 也判为非法：
 *         和 NaN 的任何比较都返回假，所以 <= 形式会自然把 NaN 挡在外面
 */
static uint8_t commReceive_gainValid(float gain)
{
    return (uint8_t)((fabsf(gain) <= COMM_PID_GAIN_ABS_MAX) ? 1U : 0U);
}

/**
 * @brief  0X00 运动控制
 * @note   speed/turn 目前只用到符号与死区，幅值暂未使用：
 *         Control() 里的目标速度是"按住方向键每 10ms 累加 1、上限 ±SPEED_Y"的斜坡
 *         模型，直接写 Target_Speed 会被下一个控制周期覆盖掉。等以后把 pid.c 改成
 *         接收比例给定，这两个字段就能直接用上，协议不用动。
 */
static revError_eTypeDef cmdReceive_move(uint8_t *pByte, uint8_t len)
{
    const CommMove_tTypeDef *pMove = (const CommMove_tTypeDef *)pByte;

    if (sizeof(CommMove_tTypeDef) != len)
    {
        return REV_ERROR_LEN;
    }

    last_bt_cmd_tick = HAL_GetTick();

    if (0U == pMove->enable)
    {
        commReceive_release();
        return REV_ERROR_NONE;
    }

    Fore  = (pMove->speed > COMM_MOVE_DEADZONE) ? 1U : 0U;
    Back  = (pMove->speed < -COMM_MOVE_DEADZONE) ? 1U : 0U;
    Right = (pMove->turn > COMM_MOVE_DEADZONE) ? 1U : 0U;
    Left  = (pMove->turn < -COMM_MOVE_DEADZONE) ? 1U : 0U;

    return REV_ERROR_NONE;
}

/**
 * @brief  0X01 紧急停止：切断电机输出并清速度环积分
 * @note   无载荷。这里连 motor_enable 一起清掉，恢复行走必须重新下发 0X02，
 *         避免"停了之后手一抖又冲出去"
 */
static revError_eTypeDef cmdReceive_stop(uint8_t *pByte, uint8_t len)
{
    (void)pByte;
    (void)len;

    commReceive_release();
    stop             = 1U;
    motor_enable     = 0U;
    last_bt_cmd_tick = HAL_GetTick();

    CommSend_Ack(REV_CMD_STOP, REV_ERROR_NONE);

    return REV_ERROR_NONE;
}

/**
 * @brief  0X02 电机使能
 * @note   使能瞬间顺手置 stop，清掉停车期间累积的速度环积分，否则一上电就窜车
 */
static revError_eTypeDef cmdReceive_motorEn(uint8_t *pByte, uint8_t len)
{
    const CommMotorEn_tTypeDef *pEn = (const CommMotorEn_tTypeDef *)pByte;

    if (sizeof(CommMotorEn_tTypeDef) != len)
    {
        return REV_ERROR_LEN;
    }

    motor_enable = (0U != pEn->enable) ? 1U : 0U;
    stop         = 1U;

    CommSend_Ack(REV_CMD_MOTOR_EN, REV_ERROR_NONE);

    return REV_ERROR_NONE;
}

/**
 * @brief  0X03 PID 参数设置
 */
static revError_eTypeDef cmdReceive_pidSet(uint8_t *pByte, uint8_t len)
{
    const CommPidSet_tTypeDef *pPid = (const CommPidSet_tTypeDef *)pByte;

    if (sizeof(CommPidSet_tTypeDef) != len)
    {
        return REV_ERROR_LEN;
    }

    if (!commReceive_gainValid(pPid->kp) || !commReceive_gainValid(pPid->ki) ||
        !commReceive_gainValid(pPid->kd))
    {
        return REV_ERROR_PARA;
    }

    switch (pPid->loop)
    {
    case COMM_PID_VERTICAL:
        Vertical_Kp = pPid->kp;
        Vertical_Kd = pPid->kd;
        break;

    case COMM_PID_VELOCITY:
        Velocity_Kp = pPid->kp;
        Velocity_Ki = pPid->ki;
        break;

    case COMM_PID_TURN:
        Turn_Kp = pPid->kp;
        /* Turn_Kd 每个控制周期都会被 Control() 按"是否正在转向"重算，这里写了
         * 也留不住，所以转向环只接受 Kp */
        break;

    default:
        return REV_ERROR_PARA;
    }

    CommSend_Ack(REV_CMD_PID_SET, REV_ERROR_NONE);

    return REV_ERROR_NONE;
}

/**
 * @brief  0X04 PID 参数查询：回一帧 SEND_ID_PID
 */
static revError_eTypeDef cmdReceive_pidGet(uint8_t *pByte, uint8_t len)
{
    (void)pByte;
    (void)len;

    CommSend_Pid();

    return REV_ERROR_NONE;
}

/**
 * @brief  0X05 平衡中值角设置
 * @note   开机 Calibrate_Med_Angle() 会自动标定，这条命令用于现场微调
 */
static revError_eTypeDef cmdReceive_medAngle(uint8_t *pByte, uint8_t len)
{
    const CommMedAngle_tTypeDef *pMed = (const CommMedAngle_tTypeDef *)pByte;

    if (sizeof(CommMedAngle_tTypeDef) != len)
    {
        return REV_ERROR_LEN;
    }

    if (fabsf(pMed->med_angle) > COMM_MED_ANGLE_ABS_MAX)
    {
        return REV_ERROR_PARA;
    }

    Med_Angle = pMed->med_angle;

    CommSend_Ack(REV_CMD_MED_ANGLE, REV_ERROR_NONE);

    return REV_ERROR_NONE;
}

/**
 * @brief  0X06 自动上报配置
 */
static revError_eTypeDef cmdReceive_reportCfg(uint8_t *pByte, uint8_t len)
{
    const CommReportCfg_tTypeDef *pCfg = (const CommReportCfg_tTypeDef *)pByte;

    if (sizeof(CommReportCfg_tTypeDef) != len)
    {
        return REV_ERROR_LEN;
    }

    if (0U != CommSend_SetReportCfg(pCfg->period_10ms, pCfg->mask))
    {
        return REV_ERROR_PARA;
    }

    CommSend_Ack(REV_CMD_REPORT_CFG, REV_ERROR_NONE);

    return REV_ERROR_NONE;
}

/**
 * @brief  0X07 心跳：只喂指令看门狗
 * @note   手机端如果只是长时间不动摇杆，也要按 <CMD_TIMEOUT_MS 的间隔发心跳，
 *         否则 Control() 会认为失联并强制归零
 */
static revError_eTypeDef cmdReceive_heartbeat(uint8_t *pByte, uint8_t len)
{
    (void)pByte;
    (void)len;

    last_bt_cmd_tick = HAL_GetTick();

    return REV_ERROR_NONE;
}

/**
 * @brief  0X08 版本查询：回一帧 SEND_ID_VERSION
 */
static revError_eTypeDef cmdReceive_version(uint8_t *pByte, uint8_t len)
{
    (void)pByte;
    (void)len;

    CommSend_Version();

    return REV_ERROR_NONE;
}
