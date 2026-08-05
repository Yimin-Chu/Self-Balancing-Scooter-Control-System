#include "pid.h"
#include "encoder.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "motor.h"
#include "math.h"
#include <stdlib.h>   // for abs()

// Sensor readings
int Encoder_Left, Encoder_Right;
float pitch, roll, yaw;
short gyrox, gyroy, gyroz;
short aacx, aacy, aacz;

// Control outputs and targets
int Vertical_out, Velocity_out, Turn_out, Target_Speed, Target_turn, MOTO1, MOTO2;
float Med_Angle = 3.1;      // Auto-calibrated at startup via Calibrate_Med_Angle()
int   gyrox_offset = 0;     // Auto-calibrated gyrox zero-bias, subtracted every loop

// PID gains
float Vertical_Kp = 480, Vertical_Kd = 1.2;    // Vertical (balance) PD
float Velocity_Kp = 0.0,  Velocity_Ki = 0.0;   // Velocity PI — set to 0 until vertical is stable
float Turn_Kp = 10, Turn_Kd = 0.6;             // Turn PD

// stop=1 clears velocity integral.
// MUST only be set once per stop event (edge-triggered), NOT every loop.
uint8_t stop;
static uint8_t prev_bt_timeout = 0;
static uint8_t prev_neutral    = 0;

// 电机输出总闸。0 时 MOTO1/MOTO2 照常计算但 PWM 恒为 0，方便只观察传感器调试。
// 默认 0，与之前硬写 Load(0,0) 的行为一致；由蓝牙命令 REV_CMD_MOTOR_EN 打开。
uint8_t motor_enable = 0;

extern TIM_HandleTypeDef htim2, htim4;
extern float distance;
extern uint8_t Fore, Back, Left, Right;

#define SPEED_Z  150

// ===========================================================================
// 手动速度目标 (调参用)
// ---------------------------------------------------------------------------
// 为什么要单开一条通道：Control() 每一拍都会按遥控输入重算 Target_Speed(见下面
// 第 2 步)，外部直接写它的话，写进去的值活不过 10ms 就被覆盖了。所以这里存一份
// 独立的目标值，由 Control() 在遥控分支之前优先取用。
//
// 用途是给速度环做阶跃：VOFA+ 上绑个滑块发 "@spd %.0f\n"，ch5(spd_ref) 立刻跟着
// 动，然后看 ch4(enc_sum) 用多久追上去、超调多少。原先没有这条通道时 spd_ref
// 恒为 0，速度环只能靠"推一把看它怎么回来"的抗扰测试来调。
//
// 开机默认就在 manual（目标 0），只有显式 spd off 才交还给蓝牙遥控。这样 VOFA
// 滑块不会被 start/看门狗踢回 remote。
//
// 自带看门狗：超过 MANUAL_SPEED_TIMEOUT_MS 没有新的设定就把目标清 0（并清速度环
// 积分），但不退出 manual。拖滑块期间每次改动都会刷新它，所以只有撒手不管才触发。
// ===========================================================================
static int      manualSpeed;       // 已夹到 ±SPEED_Y，开机为 0
static uint8_t  manualEn = 1;      // 开机默认 manual；spd off 才清零
static uint32_t manualTick;        // 最后一次设定的时刻

void Manual_Speed_Set(int speed)
{
    manualSpeed = (speed >  SPEED_Y) ?  SPEED_Y :
                 ((speed < -SPEED_Y) ? -SPEED_Y : speed);
    manualEn    = 1;
    manualTick  = HAL_GetTick();
}

void Manual_Speed_Off(void)
{
    manualSpeed = 0;
    manualEn    = 0;
}

uint8_t Manual_Speed_IsOn(void)
{
    return manualEn;
}

int Manual_Speed_Get(void)
{
    return manualSpeed;
}

// ===========================================================================
// Runtime gyrox zero-bias tracking (slow IIR)
// ---------------------------------------------------------------------------
// Absorbs slow temperature-induced bias drift during long sessions.
// Silicon die temperature rises 3-5°C over 5-10 min of operation, which
// shifts the gyro zero-bias by ~5-25 LSB. The startup calibration captures
// the cold-state bias accurately (see Calibrate_Med_Angle below), but it
// cannot predict warm-state drift. This IIR tracker handles that.
//
// Operating principle: when the vehicle is detected static (wheels still,
// no remote command), run a very slow IIR on raw gyrox to follow the
// drifting bias. Anomalous samples (shake/bump) are discarded.
// ===========================================================================
#define STATIC_ENC_THRESH      2       // |encoder count| under which wheel is "still"
#define STATIC_GYRO_REJECT     400     // |raw - offset| LSB beyond this => discard
#define STATIC_DEBOUNCE_CNT    50      // 50 × 10ms = 500ms of stillness before updating
#define BIAS_TRACK_ALPHA       0.003f  // IIR coefficient; smaller = slower tracking

static int   static_count   = 0;
static float gyrox_offset_f = 0.0f;    // float-precision offset for slow IIR
static int   fail_continus  = 0;       // consecutive anomalous-sample counter

static void Update_Gyrox_Bias(short raw_gyrox)
{
    int     dev          = abs((int)raw_gyrox - (int)gyrox_offset_f);
    uint8_t wheels_still = (abs(Encoder_Left)  <= STATIC_ENC_THRESH) &&
                           (abs(Encoder_Right) <= STATIC_ENC_THRESH);
    uint8_t no_command   = (Target_Speed == 0) && (Target_turn == 0);
    uint8_t gyro_calm    = (dev < STATIC_GYRO_REJECT);

    // Anomalous sample while otherwise static (shake/bump) — discard.
    // 5 consecutive anomalies reset debounce so we don't pollute the bias.
    if (wheels_still && no_command && !gyro_calm)
    {
        if (++fail_continus >= 5) { static_count = 0; fail_continus = 0; }
        return;
    }
    fail_continus = 0;

    if (wheels_still && no_command && gyro_calm)
    {
        if (static_count < STATIC_DEBOUNCE_CNT)
        {
            static_count++;
        }
        else
        {
            gyrox_offset_f = (1.0f - BIAS_TRACK_ALPHA) * gyrox_offset_f
                           +        BIAS_TRACK_ALPHA  * (float)raw_gyrox;
            gyrox_offset   = (int)gyrox_offset_f;
        }
    }
    else
    {
        static_count = 0;
    }
}

// ---------------------------------------------------------------------------
// Calibrate Med_Angle and gyrox zero-bias at startup — root-cure version.
//
// Background: the MPU6050 raw gyro register takes ~15-20 s to settle after
// power-on (PLL lock, MEMS resonator startup, DLPF transient flushing,
// analog reference settling, self-heating). The accelerometer settles
// almost instantly because gravity gives it a permanent DC reference;
// the gyro has no such anchor and must reach its true bias on its own.
//
// Previous version of this routine waited for DMP roll stability, which
// converges in < 1 s (acc bails it out via the complementary filter).
// That left the actual quantity we wanted to calibrate (raw gyrox) still
// drifting, and the captured offset was ~70 LSB off.
//
// New approach: monitor raw gyrox directly. Compute 1-second window
// averages and wait for THREE consecutive windows to agree within 2 LSB.
// This guarantees the bias has actually stabilized before we sample it.
//
// Expected total time: ~18-22 s after power-on. Place the car upright
// and still before calling. After this returns, gyrox - gyrox_offset ≈ 0
// immediately, no further convergence wait needed before enabling motors.
// ---------------------------------------------------------------------------
void Calibrate_Med_Angle(void)
{
    const int   WINDOW_SAMPLES  = 100;   // 100 × 10ms = 1 s per window
    const float CONVERGE_THRESH = 2.0f;  // adjacent windows must agree within 2 LSB
    const int   REQUIRED_STABLE = 3;     // need 3 consecutive agreeing windows
    const int   MAX_WINDOWS     = 60;    // safety cap: 60 s max

    float window_avg_prev = 0.0f, window_avg_curr = 0.0f;
    int   stable_windows  = 0;
    int   total_windows   = 0;

    // P1: 关闭 DMP 数据通路，避免这 ~20s 原始陀螺settling期间 DMP FIFO 持续填充并溢出。
    //     Step 1 只读原始寄存器(MPU_Get_Gyroscope)，不需要 DMP；FIFO 从源头不产生溢出。
    mpu_set_dmp_state(0);

    // Step 1: wait for raw gyrox to settle.
    while (stable_windows < REQUIRED_STABLE && total_windows < MAX_WINDOWS)
    {
        long sum = 0;
        for (int i = 0; i < WINDOW_SAMPLES; i++)
        {
            MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
            sum += gyrox;
            HAL_Delay(10);
        }
        window_avg_curr = sum / (float)WINDOW_SAMPLES;

        // Skip the very first window (nothing to compare against).
        if (total_windows > 0 &&
            fabsf(window_avg_curr - window_avg_prev) < CONVERGE_THRESH)
        {
            stable_windows++;
        }
        else
        {
            stable_windows = 0;
        }

        window_avg_prev = window_avg_curr;
        total_windows++;
    }

    // P1: Step 2 需要 DMP 输出的 roll 来求 Med_Angle，重新使能 DMP
    //     (mpu_set_dmp_state(1) 内部会复位 FIFO)，并给几帧时间产出有效数据。
    mpu_set_dmp_state(1);
    HAL_Delay(50);

    // Step 2: raw gyrox is now stable. DMP roll has been stable for many
    // seconds at this point, so we can sample both directly without an
    // additional roll-stability check. Average 200 samples (~1 s).
    float sum_roll  = 0.0f;
    long  sum_gyrox = 0;
    for (int i = 0; i < 200; i++)
    {
        mpu_dmp_get_data(&pitch, &roll, &yaw);
        MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
        sum_roll  += roll;
        sum_gyrox += gyrox;
        HAL_Delay(5);
    }

    Med_Angle      = sum_roll  / 200.0f;
    gyrox_offset   = (int)(sum_gyrox / 200);
    gyrox_offset_f = (float)gyrox_offset;   // seed runtime IIR tracker
}

// ---------------------------------------------------------------------------
// 套用一组现成的标定值(从 Flash 读回来的)，跳过 20s 静置。
//
// gyrox_offset_f 必须一起写：它是 Update_Gyrox_Bias 的 IIR 状态量，也是判断
// "本次采样是否异常" 的基准(dev = |raw - gyrox_offset_f|)。只写 gyrox_offset
// 而漏掉它的话，gyrox_offset_f 保持 0，dev 会一直大于 STATIC_GYRO_REJECT，
// 运行时零偏跟踪就彻底失效了——而且现象很隐蔽，只表现为长时间运行后慢慢跑偏。
// ---------------------------------------------------------------------------
void Calibrate_Apply(float med_angle, int gyrox_off)
{
    Med_Angle      = med_angle;
    gyrox_offset   = gyrox_off;
    gyrox_offset_f = (float)gyrox_off;
    static_count   = 0;
    fail_continus  = 0;
}

// ---------------------------------------------------------------------------
// Vertical (balance) PD controller
// ---------------------------------------------------------------------------
int Vertical(float Med, float Angle, float gyro_Y)
{
    return (int)(Vertical_Kp * (Angle - Med) + Vertical_Kd * gyro_Y);
}

// ---------------------------------------------------------------------------
// Velocity PI controller (outer loop, drives target angle offset)
// ---------------------------------------------------------------------------
int Velocity(int Target, int encoder_L, int encoder_R)
{
    static int Err_LowOut_last, Encoder_S;
    static float a = 0.7f;
    int Err, Err_LowOut, temp;

    Err = (encoder_L + encoder_R) - Target;

    Err_LowOut = (int)((1.0f - a) * Err + a * Err_LowOut_last);
    Err_LowOut_last = Err_LowOut;

    Encoder_S += Err_LowOut;

    if (Encoder_S >  3000) Encoder_S =  3000;
    if (Encoder_S < -3000) Encoder_S = -3000;

    if (stop == 1) { Encoder_S = 0; stop = 0; }

    temp = (int)(Velocity_Kp * Err_LowOut + Velocity_Ki * Encoder_S);
    return temp;
}

// ---------------------------------------------------------------------------
// Turn PD controller
// ---------------------------------------------------------------------------
int Turn(float gyro_Z, int Target_turn)
{
    return (int)(Turn_Kp * Target_turn + Turn_Kd * gyro_Z);
}

// ===========================================================================
// IMU data-ready signaling (P0) — 见 pid.h 的 RTOS 迁移说明。
// ---------------------------------------------------------------------------
// EXTI9_5(MPU INT) 回调不再直接跑 Control()，而是调用下面的 FromISR 置标志。
// 主循环轮询 Imu_ControlPending() 消费，从而把 Control()(含 I2C 阻塞读、可能的
// mpu_reset_fifo->HAL_Delay) 全部搬到线程态执行，彻底消除“中断里等 SysTick”死锁。
// ===========================================================================
volatile uint8_t  imu_data_ready      = 0;
volatile uint32_t imu_last_ready_tick = 0;

/* 已完成的控制周期数。通信层(comm_vofa.c)拿它给波形输出分频：主循环一圈的耗时
 * 抖得厉害(OLED 刷新、Flash 读写)，按时间分频会出现同一拍发两帧或连跳两拍，
 * 波形横轴就歪了；按控制周期数分频，每 N 拍恰好一帧 */
volatile uint32_t control_cycle = 0;

// 由 EXTI9_5 回调调用：只记录“有新 DMP 帧 + 何时到”。禁止在这里做 I2C / delay。
void Imu_DataReady_FromISR(void)
{
    imu_data_ready      = 1;
    imu_last_ready_tick = HAL_GetTick();
    // RTOS: xSemaphoreGiveFromISR(imuDataSem, &xHigherPriorityTaskWoken);
}

// 由主循环轮询：有新帧则返回1并清标志。RTOS 下改为阻塞式信号量 take。
//
// 前提：主循环单次阻塞必须 < 10ms(一个 MPU 中断间隔)。
// imu_data_ready 是标志位不是计数器，而 mpu_dmp_get_data() 每次只取一包，所以一次
// 阻塞若跨过两次中断，两次置位只被消费一次，DMP FIFO 就永久多压一包 —— 之后每拍读到
// 的都是 10ms 前的旧姿态，且不会自愈，积压满 1024 字节还会触发 mpu_reset_fifo() 里的
// HAL_Delay(50)。OLED 刷新(12.3ms)正是踩这条线的，已由 OLED_ENABLE 默认关掉。
// 以后再往主循环加阻塞操作(存参数到 Flash、恢复屏显等)，要么控制在 10ms 内，要么把
// 这里改成计数器 + 在 Control() 里循环排空 FIFO。
uint8_t Imu_ControlPending(void)
{
    if (imu_data_ready)
    {
        imu_data_ready = 0;
        return 1;
    }
    return 0;
    // RTOS: return (xSemaphoreTake(imuDataSem, portMAX_DELAY) == pdTRUE);
}

// ---------------------------------------------------------------------------
// Main control loop — 由主循环在收到 MPU 数据就绪信号后调用(约每 10 ms 一次)
// ---------------------------------------------------------------------------
void Control(void)
{
    int   PWM_out;
    uint8_t bt_timeout, neutral;
    short raw_gyrox_saved;          // save raw gyrox BEFORE offset correction

    // 1. Read sensors
    Encoder_Left  =  Read_Speed(&htim2);
    Encoder_Right = -Read_Speed(&htim4);
    mpu_dmp_get_data(&pitch, &roll, &yaw);
    MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
    MPU_Get_Accelerometer(&aacx, &aacy, &aacz);

    // 1a. Update runtime gyrox bias tracker (absorbs slow temperature drift).
    raw_gyrox_saved = gyrox;
    Update_Gyrox_Bias(raw_gyrox_saved);

    // Apply gyrox zero-bias correction.
    gyrox -= gyrox_offset;

    // 2. 目标值的来源：调参用的手动目标优先，其次才是蓝牙遥控
    //
    // 手动目标必须在这里覆盖而不是让外部直接写 Target_Speed —— 下面的遥控分支
    // 每拍都会重算它，外部写进去的值撑不过一个控制周期。
    /* 看门狗：只清非零目标，不退出 manual。目标已是 0 时不再反复置 stop */
    if (manualEn && (0 != manualSpeed) &&
        ((HAL_GetTick() - manualTick) > MANUAL_SPEED_TIMEOUT_MS))
    {
        manualSpeed = 0;
        stop        = 1; // 清速度环积分，避免目标归零后积分还顶着输出
    }

    if (manualEn)
    {
        Target_Speed = manualSpeed;
        Target_turn  = 0;     // 调速度环时不让转向掺进来

        // 退出手动模式后的第一拍，下面的遥控分支会看到 prev_bt_timeout==0 而置
        // stop，把手动阶跃期间攒下的速度环积分清掉，避免切回遥控时输出突跳
        prev_bt_timeout = 0;
        prev_neutral    = 0;
    }
    else
    {
        bt_timeout = ((HAL_GetTick() - last_bt_cmd_tick) > CMD_TIMEOUT_MS) ? 1 : 0;

        if (bt_timeout)
        {
            Target_Speed = 0;
            Target_turn  = 0;
            if (!prev_bt_timeout) { stop = 1; }
        }
        else
        {
            neutral = ((Fore == 0) && (Back == 0)) ? 1 : 0;

            if (neutral)
            {
                Target_Speed = 0;
                if (!prev_neutral) { stop = 1; }
            }
            else
            {
                if (Fore == 1)
                {
                    if (distance < 50) Target_Speed--;
                    else               Target_Speed++;
                }
                if (Back == 1) { Target_Speed--; }
            }

            Target_Speed = Target_Speed >  SPEED_Y ?  SPEED_Y :
                          (Target_Speed < -SPEED_Y ? -SPEED_Y : Target_Speed);

            if ((Left == 0) && (Right == 0)) Target_turn = 0;
            if (Left  == 1) Target_turn -= 30;
            if (Right == 1) Target_turn += 30;

            Target_turn = Target_turn >  SPEED_Z ?  SPEED_Z :
                         (Target_turn < -SPEED_Z ? -SPEED_Z : Target_turn);

            prev_neutral = neutral;
        }

        prev_bt_timeout = bt_timeout;
    }

    // 3. Adjust turn Kd: disable gyro damping while actively steering
    Turn_Kd = ((Left == 0) && (Right == 0)) ? 0.6f : 0.0f;

    // 4. Cascade PID
    Velocity_out = Velocity(Target_Speed, Encoder_Left, Encoder_Right);
    Vertical_out = Vertical(Velocity_out + Med_Angle, roll, gyrox);
    Turn_out     = Turn(gyroz, Target_turn);

    PWM_out = Vertical_out;
    MOTO1 = PWM_out - Turn_out;
    MOTO2 = PWM_out + Turn_out;

    Limit(&MOTO1, &MOTO2);

    // 电机输出受 motor_enable 控制：开机默认为 0(只观察 gyrox / roll)，
    // 蓝牙下发 REV_CMD_MOTOR_EN(0x02) 才真正驱动电机。
    if (motor_enable)
    {
        Load(MOTO1, MOTO2);
    }
    else
    {
        Load(0, 0);
    }

    /* 放在最后自增：通信层看到它变化时，上面那些控制量已经全部是本拍的新值 */
    control_cycle++;
}