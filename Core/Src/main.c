/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "CAN_receive.h"
#include "bsp_can.h"
#include "motor.h"

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
RC_Ctl_t RC_Ctl;  //main.h中，遥控数据结构体
const fp32 PID[3]={5,0.1, 0};
 pid_type_def motor_pid[5];
const  motor_measure_t *motor_data[4];
const  motor_measure_t *motor_data_1;
 int set_speed[5] = {0, 0, 0, 0, 0}; // 定义+初始化
int16_t g_sbus_channels[18] = {0};  // 定义+初始化
uint8_t sbus_data[25];//通道值传输数组
	


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int map(int x,int k){
 return (x-992)*k;
}

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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
	can_filter_init();
	HAL_UART_Receive_DMA(&huart3,sbus_data,25);//启动DMA接收
	for (int i=0;i<5;i++){
	PID_init(&motor_pid[i],PID_POSITION,PID,16000,2000);
	
	}//循环初始化每一个马达pid
	motor_data[0] = get_chassis_motor_measure_point(0); 		
  motor_data[1] = get_chassis_motor_measure_point(1);
  motor_data[2] = get_chassis_motor_measure_point(2);
  motor_data[3] = get_chassis_motor_measure_point(3);
	
	motor_data_1 = get_yaw_gimbal_motor_measure_point();
	
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(g_sbus_channels[0]>992&g_sbus_channels[0]<1800){
		 forward();
		}
		else if(g_sbus_channels[0]<992&g_sbus_channels[0]>100){
		back();
		}
		else{
		 motor_init1();
		}
		if(g_sbus_channels[1]>992&g_sbus_channels[1]<1800){
		right();
		}
		else if(g_sbus_channels[1]<992&g_sbus_channels[1]>100){
		left();
		}
		else{
		 motor_init1();
		}
		
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
 for(int i=0;i<4;i++){
 PID_calc(&motor_pid[i],motor_data[i]->speed_rpm,set_speed[i]);
 }
PID_calc(&motor_pid[4],motor_data_1->speed_rpm,set_speed[4]);
 CAN_cmd_chassis(motor_pid[0].out,motor_pid[1].out,motor_pid[2].out,motor_pid[3].out);
 CAN_cmd_gimbal(motor_pid[4].out ,0,0,0);

HAL_Delay(20);








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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) 
{  
				// SBUS 解码：将 25 字节帧中的按位打包数据提取为 0..15 共 16 个 11 位通道值
				// 约定（常见发射机默认）：CH1~CH4 对应四个摇杆轴；具体映射可在发射机中自定义，以下为通用解码结果。
				//
				// 位打包原理：SBUS 将 16 个通道各 11 位紧凑连续存放（LSB 优先），无字节对齐。
				// - 每个通道有 11 位数据；通道 n 的起始位偏移为 `bit_off = 11 * n`（从数据区首字节开始算）。
				// - 起始字节索引 `byte_i = 1 + (bit_off / 8)`（`sbus_data[0]` 常为帧头，因此数据从 1 开始）。
				// - 起始位在该字节中的位置 `bit_in_byte = bit_off % 8`。
				// - 因为可能跨越 2~3 个字节，所以通过右移/左移把跨字节的片段对齐到同一 11 位，再用按位或 `|` 合成，最后用 `& 0x07FF` 取低 11 位。
				// 例如 CH1（n=0）：起始位偏移 0，取 b1 的 8 位 + b2 的低 3 位 -> `(b1 | b2<<8) & 0x07FF`
				//      CH2（n=1）：起始位偏移 11，跨 b2、b3 -> `(b2>>3 | b3<<5) & 0x07FF`
				//      CH3（n=2）：起始位偏移 22，跨 b3、b4、b5 -> `(b3>>6 | b4<<2 | b5<<10) & 0x07FF`
				// 依此类推，下面对每个通道给出对应的跨字节组合与移位说明：
				//
				// 符号与位段标记说明：
				// - 约定 `bX` 表示 `sbus_data[X]`（第 X 个字节）；例如 `b1` 即 `sbus_data[1]`。
				// - `b1[7:0]` 表示字节 b1 的位 7 到位 0（含端点，MSB 到 LSB）；`b2[2:0]` 表示 b2 的最低 3 位。
				// - `>>` 右移表示把某字节的高位段移到低位对齐（提取高位数据）；`<<` 左移表示把某字节的低位段移到高位对齐（填充到 11 位的高位部分）。
				// - `|` 为按位或，用于把跨字节的片段合成一个 11 位值；`& 0x07FF` 仅保留低 11 位有效数据。
				// - 为什么 CH1 用 b1 的 8 位 + b2 的 3 位：因为每通道 11 位且 CH1 从流的起点开始，首 11 位恰好覆盖 b1 的 8 位再接着 b2 的前 3 位（8+3=11）。
				g_sbus_channels[0]  = ((sbus_data[1]  | sbus_data[2]  << 8)                                & 0x07FF); // CH1：b1[7:0] + b2[2:0] -> 11位
	    g_sbus_channels[1]  = ((sbus_data[2]  >> 3 | sbus_data[3]  << 5)                               & 0x07FF); // CH2：b2[7:3] + b3[7:0] 的低 6 位 -> 11位
	    g_sbus_channels[2]  = ((sbus_data[3]  >> 6 | sbus_data[4]  << 2 | sbus_data[5]  << 10)         & 0x07FF); // CH3：b3[7:6] + b4[7:0] + b5[1:0] -> 11位
	    g_sbus_channels[3]  = ((sbus_data[5]  >> 1 | sbus_data[6]  << 7)                               & 0x07FF); // CH4：b5[7:1] + b6[7:0] 的低 4 位 -> 11位
	    g_sbus_channels[4]  = ((sbus_data[6]  >> 4 | sbus_data[7]  << 4)                               & 0x07FF); // CH5：b6[7:4] + b7[7:0] 的低 7 位 -> 11位
	    g_sbus_channels[5]  = ((sbus_data[7]  >> 7 | sbus_data[8]  << 1 | sbus_data[9]  << 9)          & 0x07FF); // CH6：b7[7] + b8[7:0] + b9[3:0] -> 11位
	    g_sbus_channels[6]  = ((sbus_data[9]  >> 2 | sbus_data[10] << 6)                               & 0x07FF); // CH7：b9[7:2] + b10[7:0] 的低 5 位 -> 11位
	    g_sbus_channels[7]  = ((sbus_data[10] >> 5 | sbus_data[11] << 3)                               & 0x07FF); // CH8：b10[7:5] + b11[7:0] 的低 8 位 -> 11位
	    g_sbus_channels[8]  = ((sbus_data[12]      | sbus_data[13] << 8)                               & 0x07FF); // CH9
	    g_sbus_channels[9]  = ((sbus_data[13] >> 3 | sbus_data[14] << 5)                               & 0x07FF); // CH10
	    g_sbus_channels[10] = ((sbus_data[14] >> 6 | sbus_data[15] << 2 | sbus_data[16] << 10)         & 0x07FF); // CH11
	    g_sbus_channels[11] = ((sbus_data[16] >> 1 | sbus_data[17] << 7)                               & 0x07FF); // CH12
		  g_sbus_channels[12] = ((sbus_data[17] >> 4 | sbus_data[18] << 4)                               & 0x07FF); // CH13：同 CH5 模式（位移一轮）
			g_sbus_channels[13] = ((sbus_data[18] >> 7 | sbus_data[19] << 1 | sbus_data[20] << 9)          & 0x07FF); // CH14：同 CH6 模式
	    g_sbus_channels[14] = ((sbus_data[20] >> 2 | sbus_data[21] << 6)                               & 0x07FF); // CH15：同 CH7 模式
	    g_sbus_channels[15] = ((sbus_data[21] >> 5 | sbus_data[22] << 3)                               & 0x07FF);} // CH16：同 CH8 模式
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
#ifdef USE_FULL_ASSERT
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
