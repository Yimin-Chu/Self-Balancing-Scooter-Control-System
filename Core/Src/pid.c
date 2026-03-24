#include "pid.h"
#include "encoder.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "motor.h"
#include "math.h"

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

// ---------------------------------------------------------------------------
// Auto-calibrate Med_Angle and gyrox zero-bias on startup.
//
// Step 1 — Wait for DMP to converge:
//   Reads roll every 10 ms. Requires 20 consecutive readings with change
//   < 0.1 deg before proceeding. Prevents reading a drifting value.
//
// Step 2 — Average 200 samples (~1 s):
//   Computes Med_Angle (roll mean) and gyrox_offset (gyro zero-bias).
//   gyrox_offset is subtracted in Control() so static gyrox ≈ 0.
//
// Place the car upright and still before calling. Takes up to ~5 s total.
// ---------------------------------------------------------------------------
void Calibrate_Med_Angle(void)
{
    float sum_roll  = 0.0f;
    long  sum_gyrox = 0;
    float last_roll = 0.0f, cur_roll = 0.0f;
    int   stable_count = 0;

    // Step 1: wait until roll is stable (DMP converged)
    while (stable_count < 20)
    {
        mpu_dmp_get_data(&pitch, &roll, &yaw);
        cur_roll = roll;

        if (fabsf(cur_roll - last_roll) < 0.1f)
            stable_count++;
        else
            stable_count = 0;   // still drifting, reset counter

        last_roll = cur_roll;
        HAL_Delay(10);
    }

    // Step 2: stable — average 200 samples for Med_Angle and gyrox offset
    for (int i = 0; i < 200; i++)
    {
        mpu_dmp_get_data(&pitch, &roll, &yaw);
        MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
        sum_roll  += roll;
        sum_gyrox += gyrox;
        HAL_Delay(5);
    }

    Med_Angle    = sum_roll  / 200.0f;
    gyrox_offset = (int)(sum_gyrox / 200);
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

    // Low-pass filter
    Err_LowOut = (int)((1.0f - a) * Err + a * Err_LowOut_last);
    Err_LowOut_last = Err_LowOut;

    // Integrate
    Encoder_S += Err_LowOut;

    // Anti-windup clamp
    if (Encoder_S >  3000) Encoder_S =  3000;
    if (Encoder_S < -3000) Encoder_S = -3000;

    // Clear integral — fires once per stop event (edge-triggered in Control)
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
    int PWM_out;
    uint8_t bt_timeout, neutral;

    // 1. Read sensors
    Encoder_Left  =  Read_Speed(&htim2);
    Encoder_Right = -Read_Speed(&htim4);
    mpu_dmp_get_data(&pitch, &roll, &yaw);
    MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
    MPU_Get_Accelerometer(&aacx, &aacy, &aacz);

    // Apply gyrox zero-bias correction (calibrated at startup)
    // Removes the ~55 LSB static offset so Kd term doesn't produce
    // a constant disturbing force when the car is stationary.
    gyrox -= gyrox_offset;

    // 2. Remote command handling
    bt_timeout = ((HAL_GetTick() - last_bt_cmd_tick) > CMD_TIMEOUT_MS) ? 1 : 0;

    if (bt_timeout)
    {
        Target_Speed = 0;
        Target_turn  = 0;
        // Trigger stop only on the FIRST loop after timeout (rising edge)
        if (!prev_bt_timeout) { stop = 1; }
    }
    else
    {
        neutral = ((Fore == 0) && (Back == 0)) ? 1 : 0;

        if (neutral)
        {
            Target_Speed = 0;
            // Trigger stop only when buttons are freshly released (rising edge)
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