/**
 * @file    cli_cmd.c
 * @brief   CLI 具体命令：cali / start / stop / status / pid / vofa
 *
 * 每条命令都是"一个 static 函数 + 一行 CLI_CMD_EXPORT"，注册表由链接器自动收集，
 * 本文件里没有任何命令列表需要维护。
 *
 * 所有处理函数都跑在主循环(线程态)，和 Control() 是同一个上下文，
 * 因此直接读写 PID 增益这些控制变量没有竞态问题。
 */

#include "cli.h"
#include "cali_store.h"
#include "comm_pack.h"
#include "comm_port.h"
#include "comm_receive.h"
#include "comm_vofa.h"
#include "motor.h"
#include "pid.h"
#include <math.h>
#include <string.h>

/* pid.c 的控制量 */
extern uint8_t stop;
extern float   roll;
extern short   gyrox;
extern int     Encoder_Left, Encoder_Right;
extern float   Vertical_Kp, Vertical_Kd;
extern float   Velocity_Kp, Velocity_Ki;
extern float   Turn_Kp, Turn_Kd;

/* Function ------------------------------------------------------------------*/
static uint8_t cliStr2Float(const char *str, float *pValue);
static int cliCmd_cali(int argc, char *argv[]);
static int cliCmd_start(int argc, char *argv[]);
static int cliCmd_stop(int argc, char *argv[]);
static int cliCmd_status(int argc, char *argv[]);
static int cliCmd_pid(int argc, char *argv[]);
static int cliCmd_spd(int argc, char *argv[]);
static int cliCmd_vofa(int argc, char *argv[]);

CLI_CMD_EXPORT(cali, "calibrate imu (~20s) and save to flash", cliCmd_cali);
CLI_CMD_EXPORT(start, "enable motors and run pid", cliCmd_start);
CLI_CMD_EXPORT(stop, "disable motors", cliCmd_stop);
CLI_CMD_EXPORT(status, "show control and comm status", cliCmd_status);
CLI_CMD_EXPORT(pid, "show/set gains: pid [vkp vkd skp ski tkp med] <val>", cliCmd_pid);
CLI_CMD_EXPORT(spd, "set speed target for step test: spd [<-30..30>|off]", cliCmd_spd);
CLI_CMD_EXPORT(vofa, "vofa+ wave out: vofa [off jf fw rate <n>]", cliCmd_vofa);

/**
 * @brief  解析十进制小数，形如 "-12.34"
 * @param  pValue 解析结果，失败时不写
 * @retval 0:成功  1:格式非法
 * @note   自己写而不是调 strtof：newlib 的 strtod 一旦被引用就会带进约 8KB 的
 *         通用浮点解析代码(指数形式、十六进制浮点、舍入模式)，而本工程 FLASH
 *         只有 63K(最后一页留给标定数据)，链接期直接就 overflow 了。
 *         命令行输入的增益用不到那些形式，这二十行足够。
 */
static uint8_t cliStr2Float(const char *str, float *pValue)
{
    float   result = 0.0f;
    float   scale  = 0.1f;
    uint8_t sign   = 0U;
    uint8_t digits = 0U;
    uint8_t dot    = 0U;

    if ('-' == *str)
    {
        sign = 1U;
        str++;
    }
    else if ('+' == *str)
    {
        str++;
    }
    else
    {
    }

    for (; '\0' != *str; str++)
    {
        if ('.' == *str)
        {
            if (0U != dot)
            {
                return 1U; // 第二个小数点
            }

            dot = 1U;
            continue;
        }

        if ((*str < '0') || (*str > '9'))
        {
            return 1U;
        }

        digits++;

        if (0U == dot)
        {
            result = (result * 10.0f) + (float)(*str - '0');
        }
        else
        {
            result += (float)(*str - '0') * scale;
            scale *= 0.1f;
        }
    }

    if (0U == digits)
    {
        return 1U; // 只给了符号或只有一个小数点
    }

    *pValue = (0U != sign) ? -result : result;

    return 0U;
}

/**
 * @brief  cali：静置标定 IMU 并把结果写进 Flash
 * @note   会阻塞约 20s，期间 Control() 完全不执行，所以第一件事就是切电机。
 *         之后的 Flash 擦写又要占用 20~40ms，同样必须在电机停转时做。
 */
static int cliCmd_cali(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    motor_enable = 0U;
    Load(0, 0);

    Cli_Printf("calibrating, keep the car upright and still (~20s)...\r\n");

    Calibrate_Med_Angle();

    Cli_Printf("done: med_angle=%.2f gyrox_offset=%d\r\n", Med_Angle, gyrox_offset);

    if (0U != CaliStore_Save(Med_Angle, (int32_t)gyrox_offset))
    {
        Cli_Printf("flash save FAILED\r\n");
        return -1;
    }

    Cli_Printf("saved to flash @0x%08lX, motors still off, type 'start' to run\r\n",
               (unsigned long)CALI_STORE_ADDR);

    return 0;
}

/**
 * @brief  start：使能电机，开始跑 PID
 * @note   顺带置 stop 清掉速度环积分，否则停车期间攒下的积分会让车一上电就窜出去；
 *         再喂一次指令看门狗，避免刚使能就被 CMD_TIMEOUT_MS 判定失联
 */
static int cliCmd_start(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    if (0U == CaliStore_IsValid())
    {
        Cli_Printf("no valid calibration, run 'cali' first\r\n");
        return -1;
    }

    Manual_Speed_Set(0);  // 留在 manual，只清目标，避免上次 spd 残留一使能就窜出去
    stop             = 1U;
    last_bt_cmd_tick = HAL_GetTick();
    motor_enable     = 1U;

    Cli_Printf("motors ON, pid running (med_angle=%.2f)\r\n", Med_Angle);

    return 0;
}

/**
 * @brief  stop：切电机
 */
static int cliCmd_stop(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    motor_enable = 0U;
    Load(0, 0);
    stop = 1U;
    Manual_Speed_Set(0);  // 留在 manual，只清目标；要交还遥控用 spd off

    Cli_Printf("motors OFF\r\n");

    return 0;
}

/**
 * @brief  status：一屏看完控制与通信状态
 */
static int cliCmd_status(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    Cli_Printf("motor : %s\r\n", motor_enable ? "on" : "off");
    Cli_Printf("cali  : %s med=%.2f gyroff=%d\r\n", CaliStore_IsValid() ? "valid" : "none",
               Med_Angle, gyrox_offset);
    Cli_Printf("imu   : roll=%.2f gyrox=%d\r\n", roll, gyrox);
    Cli_Printf("wheel : encL=%d encR=%d\r\n", Encoder_Left, Encoder_Right);
    Cli_Printf("spdref: %s\r\n", Manual_Speed_IsOn() ? "manual" : "remote");
    if (Manual_Speed_IsOn()) { Cli_Printf("        target=%d\r\n", Manual_Speed_Get()); }
    Cli_Printf("comm  : rx=%u crcErr=%u frmErr=%u txDrop=%u\r\n", commPackStat.rxFrame,
               commPackStat.crcError, commPackStat.frameError, commPortStat.txDropFrame);
    Cli_Printf("vofa  : %s div=%u drop=%lu\r\n", Vofa_GetModeName(), Vofa_GetDiv(),
               (unsigned long)Vofa_GetDropCount());
    Cli_Printf("quiet : ok=%u err=%u\r\n", Cli_GetQuietOk(), Cli_GetQuietErr());

    return 0;
}

/**
 * @brief  pid：查看/修改各环增益，配合 vofa 波形边看边调
 * @note   不带参数打印全部；带参数改一项。改完只在 RAM 里生效，掉电恢复默认值，
 *         调出满意的一组后请写回 Core/Src/pid.c 的初值。
 */
static int cliCmd_pid(int argc, char *argv[])
{
    float value;

    if (argc < 2)
    {
        Cli_Printf("vert: kp=%.2f kd=%.2f\r\n", Vertical_Kp, Vertical_Kd);
        Cli_Printf("velo: kp=%.3f ki=%.3f\r\n", Velocity_Kp, Velocity_Ki);
        Cli_Printf("turn: kp=%.2f kd=%.2f(auto)\r\n", Turn_Kp, Turn_Kd);
        Cli_Printf("med : %.2f\r\n", Med_Angle);
        return 0;
    }

    if (argc < 3)
    {
        Cli_Printf("usage: pid [vkp|vkd|skp|ski|tkp|med] <value>\r\n");
        return -1;
    }

    if (0U != cliStr2Float(argv[2], &value))
    {
        Cli_Printf("bad number: %s\r\n", argv[2]);
        return -1;
    }

    if (0 == strcmp(argv[1], "med"))
    {
        if (fabsf(value) > COMM_MED_ANGLE_ABS_MAX)
        {
            Cli_Printf("out of range (+-%.1f)\r\n", COMM_MED_ANGLE_ABS_MAX);
            return -1;
        }

        Med_Angle = value;
    }
    else if (0 == strcmp(argv[1], "tkd"))
    {
        /* Turn_Kd 每个控制周期都会被 Control() 按"是否正在转向"重算，写了留不住 */
        Cli_Printf("turn kd is recomputed every cycle by Control()\r\n");
        return -1;
    }
    else
    {
        if (fabsf(value) > COMM_PID_GAIN_ABS_MAX)
        {
            Cli_Printf("out of range (+-%.0f)\r\n", COMM_PID_GAIN_ABS_MAX);
            return -1;
        }

        if (0 == strcmp(argv[1], "vkp"))
        {
            Vertical_Kp = value;
        }
        else if (0 == strcmp(argv[1], "vkd"))
        {
            Vertical_Kd = value;
        }
        else if (0 == strcmp(argv[1], "skp"))
        {
            Velocity_Kp = value;
            stop        = 1U; // 清速度环积分：换了增益，旧积分对应的是旧比例，留着会突跳
        }
        else if (0 == strcmp(argv[1], "ski"))
        {
            Velocity_Ki = value;
            stop        = 1U;
        }
        else if (0 == strcmp(argv[1], "tkp"))
        {
            Turn_Kp = value;
        }
        else
        {
            Cli_Printf("unknown gain: %s\r\n", argv[1]);
            return -1;
        }
    }

    Cli_Printf("%s = %.3f\r\n", argv[1], value);

    return 0;
}

/**
 * @brief  spd：设定速度环目标，用来做阶跃响应
 * @note   进的是 pid.c 的手动目标通道而不是 Target_Speed 本身 —— 后者每个控制周期
 *         都被 Control() 按遥控输入重算，直接写留不住。开机默认就在 manual。
 *         生效期间转向被强制为 0。VOFA+ 滑块绑 "@spd %.0f\n" 即可拖着设值。
 *         超过 MANUAL_SPEED_TIMEOUT_MS 没有新设定会把目标清 0，但仍留在 manual；
 *         只有 spd off 才交还给蓝牙遥控。
 */
static int cliCmd_spd(int argc, char *argv[])
{
    float value;

    if (argc < 2)
    {
        Cli_Printf("spd : %s target=%d\r\n", Manual_Speed_IsOn() ? "manual" : "remote",
                   Manual_Speed_Get());
        return 0;
    }

    if (0 == strcmp(argv[1], "off"))
    {
        Manual_Speed_Off();
        Cli_Printf("spd = remote\r\n");
        return 0;
    }

    if (0U != cliStr2Float(argv[1], &value))
    {
        Cli_Printf("bad number: %s\r\n", argv[1]);
        return -1;
    }

    Manual_Speed_Set((int)value);
    /* 回显读回来的值而不是入参：超出 ±SPEED_Y 会被夹住，让你看见夹了 */
    Cli_Printf("spd = %d\r\n", Manual_Speed_Get());

    return 0;
}

/**
 * @brief  vofa：控制 VOFA+ 波形输出
 * @note   JustFloat 是二进制流，开着的时候终端里敲命令会插进数据中间。
 *         VOFA+ 靠帧尾重新对齐，最多糊掉一两个采样点，不影响继续调参
 */
static int cliCmd_vofa(int argc, char *argv[])
{
    float   div;
    uint8_t i;

    if (argc >= 2)
    {
        if (0 == strcmp(argv[1], "off"))
        {
            Vofa_SetMode(VOFA_MODE_OFF);
        }
        else if ((0 == strcmp(argv[1], "jf")) || (0 == strcmp(argv[1], "on")))
        {
            Vofa_SetMode(VOFA_MODE_JUSTFLOAT);
        }
        else if (0 == strcmp(argv[1], "fw"))
        {
            Vofa_SetMode(VOFA_MODE_FIREWATER);
        }
        else if ((0 == strcmp(argv[1], "rate")) && (argc >= 3))
        {
            /* 先在 float 域判范围再转 uint8_t：直接转的话 "300" 会被截成 44，
             * 落进合法区间，用户以为设成功了其实是另一个值 */
            if ((0U != cliStr2Float(argv[2], &div)) || (div < (float)VOFA_DIV_MIN) ||
                (div > (float)VOFA_DIV_MAX))
            {
                Cli_Printf("rate must be %u..%u\r\n", VOFA_DIV_MIN, VOFA_DIV_MAX);
                return -1;
            }

            Vofa_SetDiv((uint8_t)div);
        }
        else
        {
            Cli_Printf("usage: vofa [off|jf|fw|rate <%u-%u>]\r\n", VOFA_DIV_MIN, VOFA_DIV_MAX);
            return -1;
        }
    }

    Cli_Printf("vofa: %s div=%u (%uHz) drop=%lu\r\n", Vofa_GetModeName(), Vofa_GetDiv(),
               (unsigned)(VOFA_CTRL_HZ / Vofa_GetDiv()), (unsigned long)Vofa_GetDropCount());

    /* 只在不带参数时列通道：改完模式还刷八行，波形数据里会插进一大段文本 */
    if (argc < 2)
    {
        for (i = 0U; i < VOFA_CH_NUM; i++)
        {
            Cli_Printf("  ch%u %s\r\n", i, Vofa_GetChannelName(i));
        }
    }

    return 0;
}
