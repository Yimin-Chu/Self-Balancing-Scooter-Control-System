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
#if OLED_ENABLE
#include "stdio.h"
#endif
#include "sr04.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "comm.h"
#include "comm_echo.h"
#include "cali_store.h"
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
#if OLED_ENABLE
uint8_t display_buf[32];
#endif
uint32_t sys_tick;
extern float distance;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Read(void);
#if OLED_ENABLE
static void Display_Poll(void);
#endif
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
#if OLED_ENABLE
  /* I2C1 全工程只有 OLED 在用，关屏时连初始化一起省掉，HAL_I2C 驱动也就不会被链进来 */
  MX_I2C1_Init();
#endif
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
#if OLED_ENABLE
  OLED_Init();
  OLED_Clear();
#endif
  MPU_Init();
  mpu_dmp_init();
#if OLED_ENABLE
  OLED_ShowString(0, 0, "Init Sucess", 16);
#endif

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

  // 1. 取得 gyrox 零偏与平衡中值角。优先用 Flash 里存的上次标定结果，
  //    省掉每次开机 18-22 s 的静置等待；Flash 里没有(首次上电/数据损坏)才现场标定。
  //
  //    代价：陀螺零偏本来就随温度和上电次数漂移，复用旧值不如现场标定准。
  //    pid.c 的运行时零偏跟踪器(Update_Gyrox_Bias)会慢慢把差值吸收掉，
  //    但如果发现小车静止时缓慢跑偏，用串口敲 'cali' 重新标定一次即可。
  if (0 == CaliStore_Load())
  {
    Calibrate_Apply(CaliStore_GetMedAngle(), (int)CaliStore_GetGyroxOffset());
#if OLED_ENABLE
    OLED_ShowString(0, 2, "Cali from flash", 12);
#endif
  }
  else
  {
#if OLED_ENABLE
    OLED_ShowString(0, 2, "Calibrating...", 16);
    OLED_ShowString(0, 4, "Hold still ~20s", 12);
#endif
    Calibrate_Med_Angle();
    CaliStore_Save(Med_Angle, (int32_t)gyrox_offset);
  }
#if OLED_ENABLE
  OLED_Clear();
  OLED_ShowString(0, 0, "Ready", 16);
  sprintf((char *)display_buf, "gyrox_off:%d", gyrox_offset);
  OLED_ShowString(0, 2, display_buf, 12);
  HAL_Delay(1000);   // let user briefly see the calibrated offset
  OLED_Clear();
#endif

  // 2. Start encoders so Control() reads valid wheel counts immediately.
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  // 3. Start motor PWM channels, then force duty cycle to 0 (belt + suspenders).
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  Load(0, 0);

  // 4. 初始化通信层(USART3 蓝牙)：武装接收中断 + 启动周期上报。
  //    帧格式参考追觅协议，见 Comm/comm_protocol_readme.md。
  Comm_Init();

  // 5. Arm the MPU data-ready interrupt.
  //
  //    P0: EXTI9_5 回调不再直接跑 Control()，只调用 Imu_DataReady_FromISR() 置标志；
  //        真正的 Control() 在下面的主循环(线程态)执行。因此 DMP 驱动内部即便触发
  //        HAL_Delay(如 mpu_reset_fifo)，SysTick 也能正常递增，不会再死锁。
  //    P1: Calibrate_Med_Angle() 内部已在 ~20s 原始陀螺settling期间关闭 DMP，
  //        标定期不再填充/溢出 FIFO；这里再复位一次 FIFO 作兜底，保证开中断后第一帧干净。
  //
  //    注: EXTI9_5 的 NVIC 优先级(0)保留即可——重活已移出中断，优先级高不再有害。
  mpu_reset_fifo();
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* P0: 事件驱动控制。每当 MPU 产生一帧 DMP 数据(EXTI 置标志)，主循环消费并执行
     *     Control()。现在运行在线程态，DMP 驱动内部即使 HAL_Delay 也不会死锁。
     *     RTOS 迁移: 把 Imu_ControlPending() 换成阻塞式信号量 take，此处结构不变。 */
    if (Imu_ControlPending())
    {
      Control();
    }

    /* 蓝牙通信轮询：解析收到的协议帧、按周期上报状态。中断只搬字节，解析在这里做。 */
    Comm_Poll();

#if OLED_ENABLE
    /* 显示节流(~10Hz)。OLED 刷新慢，不能每圈都刷，否则会挤占 10ms 控制时序。 */
    Display_Poll();
#endif
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

#if OLED_ENABLE

/**
  * @brief  OLED 刷新：每 100ms 最多刷一行
  * @note   一行整页突发写在 100kHz I2C 上约 12ms，已经比 10ms 的控制周期还长，
  *         所以一个 tick 只刷一行；四行一起刷等于制造 50ms 的失控窗口。
  *
  *         命令回显排在传感器行前面：敲下的字符要在 100ms 内出现在屏上才有实时感。
  *         没人敲命令时两行传感器交替刷新，各约 5Hz，读数够看。
  *
  *         版面(每行 16 字符)：
  *           page0  gyrox:<陀螺原始值>
  *           page2  roll:<角度>
  *           page4  ><正在输入的命令>
  *           page6  <最近一条命令/帧的结果>
  */
static void Display_Poll(void)
{
  static uint32_t tick       = 0;
  static uint8_t  sensorLine = 0;

  if ((HAL_GetTick() - tick) < 100U)
  {
    return;
  }
  tick = HAL_GetTick();

  if (0U != CommEcho_TakeInputDirty())
  {
    sprintf((char *)display_buf, ">%s", CommEcho_GetInput());
    OLED_ShowLine(4, (char *)display_buf);
    return;
  }

  if (0U != CommEcho_TakeEventDirty())
  {
    OLED_ShowLine(6, CommEcho_GetEvent());
    return;
  }

  if (0U == sensorLine)
  {
    sprintf((char *)display_buf, "gyrox:%d", gyrox);
    OLED_ShowLine(0, (char *)display_buf);
  }
  else
  {
    sprintf((char *)display_buf, "roll:%.2f", roll);
    OLED_ShowLine(2, (char *)display_buf);
  }

  sensorLine ^= 1U;
}

#endif /* OLED_ENABLE */

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