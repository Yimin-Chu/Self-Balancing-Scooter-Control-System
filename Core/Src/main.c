/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "IIC.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "stdio.h"
#include "sr04.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern float roll;
extern short gyrox, gyroy, gyroz;
extern int Encoder_Left, Encoder_Right;
extern int gyrox_offset;
uint8_t display_buf[24];
uint32_t sys_tick;
extern float distance;
extern uint8_t rx_buf[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Read(void);
/* If pid.h doesn't declare it, uncomment this:
   extern void Calibrate_Med_Angle(void);                                    */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  OLED_Clear();
  MPU_Init();
  mpu_dmp_init();
  OLED_ShowString(0, 0, "Init Sucess", 16);

  // -----------------------------------------------------------------------
  // STARTUP ORDER (critical!)
  //
  // The MPU INT (EXTI9_5) is what triggers Control(). Once enabled, Control()
  // starts firing every 10 ms and writes PWM to the motors via Load(). So
  // EVERYTHING that Control() depends on must be ready BEFORE we enable INT:
  //   - gyrox_offset must reflect the true settled bias  (Calibrate_Med_Angle)
  //   - encoders must be running                          (HAL_TIM_Encoder_Start)
  //   - motor PWM channels must be initialised to 0      (HAL_TIM_PWM_Start + Load(0,0))
  //   - UART RX must be armed                            (HAL_UART_Receive_IT)
  //
  // EXTI is enabled LAST.
  // -----------------------------------------------------------------------

  // 1. Calibrate raw gyrox bias. BLOCKS for ~18-22 s while the chip's gyro
  //    register settles after power-on. Keep the car upright and still.
  OLED_ShowString(0, 2, "Calibrating...", 16);
  OLED_ShowString(0, 4, "Hold still ~20s", 12);
  Calibrate_Med_Angle();
  OLED_Clear();
  OLED_ShowString(0, 0, "Ready", 16);
  sprintf((char *)display_buf, "gyrox_off:%d", gyrox_offset);
  OLED_ShowString(0, 2, display_buf, 12);
  HAL_Delay(1000);   // let user briefly see the calibrated offset
  OLED_Clear();

  // 2. Start encoders so Control() reads valid wheel counts immediately.
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  // 3. Start motor PWM channels, then force duty cycle to 0 (belt + suspenders).
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  Load(0, 0);

  // 4. Arm UART receive for remote command bytes.
  HAL_UART_Receive_IT(&huart3, rx_buf, 1);

  // 5. Clear the DMP FIFO *in thread context*, immediately before arming the INT.
  //
  //    Why this is mandatory: Calibrate_Med_Angle() above reads the RAW gyro
  //    registers for ~20 s and never drains the DMP FIFO, so the FIFO has
  //    overflowed by now. If we leave it overflowed, the very first Control()
  //    -- which runs inside the EXTI9_5 ISR at preempt priority 0 -- would go
  //    mpu_dmp_get_data -> dmp_read_fifo -> mpu_read_fifo_stream, hit the
  //    overflow bit, and call mpu_reset_fifo(), which contains delay_ms(50)
  //    == HAL_Delay(50). HAL_Delay waits on HAL_GetTick(), and the tick only
  //    advances in the SysTick ISR -- which cannot preempt a priority-0 ISR.
  //    The tick never moves, HAL_Delay never returns, and the CPU is wedged in
  //    the interrupt forever: OLED frozen black, motors never updated.
  //
  //    Doing the reset here (thread mode) makes that HAL_Delay harmless, and
  //    the first ISR read then sees a clean FIFO.
  mpu_reset_fifo();
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    sprintf((char *)display_buf, "R:%.1f   ", roll);
    OLED_ShowString(0, 0, display_buf, 12);
    sprintf((char *)display_buf, "Gx:%d  ", gyrox);
    OLED_ShowString(0, 1, display_buf, 12);
    sprintf((char *)display_buf, "Gy:%d  ", gyroy);
    OLED_ShowString(0, 2, display_buf, 12);
    sprintf((char *)display_buf, "Gz:%d  ", gyroz);
    OLED_ShowString(0, 3, display_buf, 12);
    sprintf((char *)display_buf, "L:%d  ", Encoder_Left);
    OLED_ShowString(0, 4, display_buf, 12);
    sprintf((char *)display_buf, "R:%d  ", Encoder_Right);
    OLED_ShowString(0, 5, display_buf, 12);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */