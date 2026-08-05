/**
 * @file    comm_vofa.c
 * @brief   VOFA+ 波形输出：采样控制环内部量并按 JustFloat / FireWater 发出
 *
 * 和 comm_send.c 一样，本文件只用 extern 取 pid.c 的全局量，不反过来让 pid.c
 * 依赖通信层，保持"通信层向下看"的单向依赖。
 *
 * 调用时机：Comm_Poll() 里，也就是主循环、Control() 刚跑完之后。八个通道全部
 * 来自同一次 Control() 的计算结果，波形上不会出现"角度是这一拍、输出是上一拍"
 * 的错位。
 */

#include "comm_vofa.h"
#include "comm_port.h"
#include "comm_send.h"
#include "pid.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Datas ---------------------------------------------------------------------*/
/* pid.c 的传感器与控制量 */
extern float  roll;
extern short  gyrox;
extern int    Encoder_Left, Encoder_Right;
extern int    Vertical_out, Velocity_out;
extern int    Target_Speed;
extern int    MOTO1, MOTO2;
extern float  Med_Angle;
extern uint8_t motor_enable;

static VofaMode_eTypeDef vofaMode;
static uint8_t           vofaDiv;
static uint32_t          vofaLastCycle;
static uint32_t          vofaDropCount;

/* 通道顺序就是 VOFA+ 里曲线的顺序：
 * 直立环 ch0/ch1 比目标与实测、ch2 是 D 项输入、ch3 是直立输出；
 * 速度环 ch4/ch5 比实测与目标(速度环输出叠在 ch1 上，不单列)；
 * ch6/ch7 是限幅后的左右轮 PWM(真正 Load 用的量；motor_enable=0 时采 0) */
static const char *const VOFA_CH_NAMES[VOFA_CH_NUM] = {
    "roll",     // 0 实测倾角(度)
    "roll_ref", // 1 直立环目标角 = Med_Angle + Velocity_out
    "gyrox",    // 2 角速度(已扣零偏)，直立环 D 项输入
    "vert_out", // 3 直立环输出，也就是基础 PWM
    "enc_sum",  // 4 实测速度 = 左右编码器之和
    "spd_ref",  // 5 目标速度
    "pwm_l",    // 6 左轮 PWM = MOTO1(Limit 后)
    "pwm_r",    // 7 右轮 PWM = MOTO2(Limit 后)
};

static const char *const VOFA_MODE_NAMES[VOFA_MODE_TOTAL] = {
    "off",
    "justfloat",
    "firewater",
};

/* Function ------------------------------------------------------------------*/
static float vofa_finite(float value);
static void  vofa_sample(float *pCh);
static void  vofa_sendJustFloat(const float *pCh);
static void  vofa_sendFireWater(const float *pCh);

/**
 * @brief  初始化波形输出
 * @note   默认关闭。二进制流一开，终端上全是乱码，开机横幅和命令回执都看不清，
 *         与 comm_send.h 里 COMM_REPORT_PERIOD_DEFAULT 默认为 0 是一个道理
 */
void Vofa_Init(void)
{
    vofaMode      = VOFA_MODE_OFF;
    vofaDiv       = VOFA_DIV_DEFAULT;
    vofaLastCycle = control_cycle;
    vofaDropCount = 0U;
}

/**
 * @brief  波形输出轮询，由 Comm_Poll() 每圈主循环调用
 */
void Vofa_Poll(void)
{
    float    ch[VOFA_CH_NUM];
    uint32_t cycle;

    if (VOFA_MODE_OFF == vofaMode)
    {
        return;
    }

    /* 按控制周期分频，而不是按 HAL_GetTick() 计时。
     * 主循环一圈的耗时随 OLED 刷新、Flash 读写抖得厉害，用时间分频会出现
     * 同一个控制周期发两帧、或者连着跳过两拍的情况，画出来的波形横轴是歪的。
     * 用控制周期计数，每 div 拍恰好一帧，横轴才是均匀的 */
    cycle = control_cycle;

    if ((cycle - vofaLastCycle) < (uint32_t)vofaDiv)
    {
        return;
    }

    vofaLastCycle = cycle;

    vofa_sample(ch);

    if (VOFA_MODE_JUSTFLOAT == vofaMode)
    {
        vofa_sendJustFloat(ch);
    }
    else
    {
        vofa_sendFireWater(ch);
    }
}

/**
 * @brief  切换输出模式
 */
uint8_t Vofa_SetMode(VofaMode_eTypeDef mode)
{
    if (mode >= VOFA_MODE_TOTAL)
    {
        return 1U;
    }

    /* 周期上报的二进制帧会插进波形数据流中间，两边都解不出来。开波形就把它关掉，
     * 和 Cli_OnFirstActivity() 里的兜底是同一个处理 */
    if ((VOFA_MODE_OFF != mode) && (0U != CommSend_IsReporting()))
    {
        CommSend_SetReportCfg(0U, 0U);
    }

    vofaMode      = mode;
    vofaLastCycle = control_cycle;
    vofaDropCount = 0U;

    return 0U;
}

/**
 * @brief  设置分频
 */
uint8_t Vofa_SetDiv(uint8_t div)
{
    if ((div < VOFA_DIV_MIN) || (div > VOFA_DIV_MAX))
    {
        return 1U;
    }

    vofaDiv = div;

    return 0U;
}

VofaMode_eTypeDef Vofa_GetMode(void)
{
    return vofaMode;
}

uint8_t Vofa_GetDiv(void)
{
    return vofaDiv;
}

uint32_t Vofa_GetDropCount(void)
{
    return vofaDropCount;
}

const char *Vofa_GetModeName(void)
{
    return VOFA_MODE_NAMES[vofaMode];
}

const char *Vofa_GetChannelName(uint8_t index)
{
    return (index < VOFA_CH_NUM) ? VOFA_CH_NAMES[index] : "?";
}

/**
 * @brief  挡掉 inf/NaN
 * @note   +inf 的字节形态(00 00 80 7F)恰好就是 JustFloat 的帧尾，某个通道一旦
 *         取到它，上位机会在数据中间切帧，之后所有曲线永久错位。NaN 则会被画成
 *         断点。两者都换成 0，宁可丢一个点也不能让整屏波形失去意义
 */
static float vofa_finite(float value)
{
    return (0 != isfinite(value)) ? value : 0.0f;
}

/**
 * @brief  采样一帧通道值
 */
static void vofa_sample(float *pCh)
{
    /* 直立环的给定是"中值角 + 速度环输出"，见 pid.c 的 Control()。
     * 把它和 roll 画在一起，直立环的跟随误差一眼可见 */
    pCh[0] = vofa_finite(roll);
    pCh[1] = vofa_finite(Med_Angle + (float)Velocity_out);
    pCh[2] = (float)gyrox;
    pCh[3] = (float)Vertical_out;
    pCh[4] = (float)(Encoder_Left + Encoder_Right);
    pCh[5] = (float)Target_Speed;
    /* 与 Load() 一致：未使能时实际输出为 0，波形也按 0 采，避免误判 */
    pCh[6] = (0U != motor_enable) ? (float)MOTO1 : 0.0f;
    pCh[7] = (0U != motor_enable) ? (float)MOTO2 : 0.0f;
}

/**
 * @brief  JustFloat：N 个小端 float + 帧尾 00 00 80 7F
 * @note   STM32 本机就是小端 IEEE754，直接把数组按字节拷出去即可，不用逐个转换
 */
static void vofa_sendJustFloat(const float *pCh)
{
    uint8_t frame[(VOFA_CH_NUM * 4U) + 4U];

    memcpy(frame, pCh, VOFA_CH_NUM * 4U);

    frame[(VOFA_CH_NUM * 4U) + 0U] = 0X00U;
    frame[(VOFA_CH_NUM * 4U) + 1U] = 0X00U;
    frame[(VOFA_CH_NUM * 4U) + 2U] = 0X80U;
    frame[(VOFA_CH_NUM * 4U) + 3U] = 0X7FU;

    if (0U == CommPort_TxPush(frame, (uint16_t)sizeof(frame)))
    {
        vofaDropCount++;
    }
}

/**
 * @brief  FireWater：文本 "1.23,4.56,...\n"
 * @note   每通道最长 "-32768.00" + 分隔符 = 10 字节，按 12 留余量
 */
static void vofa_sendFireWater(const float *pCh)
{
    char   line[(VOFA_CH_NUM * 12U) + 4U];
    size_t len = 0U;
    uint8_t i;
    int     n;

    for (i = 0U; i < VOFA_CH_NUM; i++)
    {
        n = snprintf(&line[len], sizeof(line) - len, "%.2f%c", pCh[i],
                     (i == (VOFA_CH_NUM - 1U)) ? '\n' : ',');

        /* 被截断就整帧放弃：半行文本会让 VOFA+ 把残缺的数字当成一个真实采样点画出来 */
        if ((n <= 0) || ((size_t)n >= (sizeof(line) - len)))
        {
            vofaDropCount++;
            return;
        }

        len += (size_t)n;
    }

    if (0U == CommPort_TxPush((const uint8_t *)line, (uint16_t)len))
    {
        vofaDropCount++;
    }
}
