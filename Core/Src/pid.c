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

extern TIM_HandleTypeDef htim2, htim4;
extern float distance;
extern uint8_t Fore, Back, Left, Right;

#define SPEED_Y  30
#define SPEED_Z  150

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

// ---------------------------------------------------------------------------
// Main control loop — called every 10 ms from MPU6050 INT interrupt
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

    // 2. Remote command handling
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
    Load(MOTO1, MOTO2);
}