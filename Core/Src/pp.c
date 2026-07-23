static void Update_Gyrox_Bias(short raw_gyrox)
{
    int     dev          = abs((int)raw_gyrox - (int)gyrox_offset_f);
    uint8_t wheels_still = (abs(Encoder_Left)  <= STATIC_ENC_THRESH) &&
}



void Calibrate_Med_Angle(void)
{
    const int WINDOW_SAMPLES = 100;
    const float CONVERGE_THRESH = 2.0f;
    const int REQUIRED_STABLE = 3;
    const int MAX_WINDOWS = 60;

    float window_avg_prev = 0.0f, window_avg_curr = 0.0f;
    int stable_windows = 0;
    int total_windows = 0;

    mpu_set_dmp_state(0);

    while (stable_windows < REQUIRED_STABLE && total_windows < MAX_WINDOWS)
    {
        long sum =0;
        for (int i = 0; i < WINDOW_SAMPLES; i++)
        {
            MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
            sum += gyrox;
            HAL_Delay(10);
        }
        window_avg_curr = sum / (float)WINDOW_SAMPLES;


    }
}