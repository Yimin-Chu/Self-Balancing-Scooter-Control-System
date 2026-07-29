/**
 * @file    cli_cmd.c
 * @brief   CLI 具体命令：cali / start / stop / status
 *
 * 每条命令都是"一个 static 函数 + 一行 CLI_CMD_EXPORT"，注册表由链接器自动收集，
 * 本文件里没有任何命令列表需要维护。
 */

#include "cli.h"
#include "cali_store.h"
#include "comm_pack.h"
#include "comm_port.h"
#include "motor.h"
#include "pid.h"

/* pid.c 的控制量 */
extern uint8_t stop;
extern float   roll;
extern short   gyrox;
extern int     Encoder_Left, Encoder_Right;

/* Function ------------------------------------------------------------------*/
static int cliCmd_cali(int argc, char *argv[]);
static int cliCmd_start(int argc, char *argv[]);
static int cliCmd_stop(int argc, char *argv[]);
static int cliCmd_status(int argc, char *argv[]);

CLI_CMD_EXPORT(cali, "calibrate imu (~20s) and save to flash", cliCmd_cali);
CLI_CMD_EXPORT(start, "enable motors and run pid", cliCmd_start);
CLI_CMD_EXPORT(stop, "disable motors", cliCmd_stop);
CLI_CMD_EXPORT(status, "show control and comm status", cliCmd_status);

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
    Cli_Printf("comm  : rx=%u crcErr=%u frmErr=%u txDrop=%u\r\n", commPackStat.rxFrame,
               commPackStat.crcError, commPackStat.frameError, commPortStat.txDropFrame);

    return 0;
}
