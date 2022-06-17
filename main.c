/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0x68 << 1
#define GYRO_CONFIG 0x1B
#define GYRO_XOUT_H_REG 0x43
#define ACCEL_XOUT_H_REG 0x3B
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
//someshit
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  char hello_world[] = "hello world\n";
  //HAL_UART_Transmit(&huart2, hello_world, sizeof(hello_world),10);
  char xaxis[] = "xaxis";
  char yaxis[] = "yaxis";
  char zaxis[] = "zaxis";
  char check_print[] = "check being printed:";

  //HAL_UART_Transmit(&huart2, xaxis, sizeof(xaxis),10);
  //HAL_UART_Transmit(&huart2, yaxis, sizeof(yaxis),10);
  //HAL_UART_Transmit(&huart2, zaxis, sizeof(zaxis),10);

uint8_t check;
HAL_I2C_Mem_Read(&hi2c3, 0xD0, WHO_AM_I_REG, 1, &check, 1, 1000);
HAL_Delay(2000);
//HAL_UART_Transmit(&huart2, check_print, sizeof(check_print),10);
HAL_UART_Transmit(&huart2, &check, sizeof(check),10);
uint8_t Data = 0;
uint8_t Rec_Data[6];
Data = 0;
char comma = ",";
HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);
//setting clock rate to 1 KHz
Data = 0x07;
HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);
//getting full scale gyro
Data = 0x00;
HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, 0x1B, 1, &Data, 1, 1000);

HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, 0x1C, 1, &Data, 1, 1000);
uint16_t Gyro_X_RAW;
uint16_t Gyro_Y_RAW;
uint16_t Gyro_Z_RAW;
float Gx;
float Gy;
float Gz;


float Accel_X_RAW;
float Accel_Y_RAW;
float Accel_Z_RAW;
float Ax;
float Ay;
float Az;

/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout)
		//HAL_UART_Transmit(&huart2, hello_world, sizeof(hello_world),10);
	  //HAL_GPIO_ReadPin(B1_GPIO_Port, GPIO_Pin);

	  //setting power to 1


		int ghj = 0;

		HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

		Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
		Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
		Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);
		Gx = Gyro_X_RAW/131.0;
		Gy = Gyro_Y_RAW/131.0;
		Gz = Gyro_Z_RAW/131.0;

		HAL_I2C_Mem_Read (&hi2c3, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
		Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
		Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
		Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);
		Ax = Accel_X_RAW/16384.0;
		Ay = Accel_Y_RAW/16384.0;
		Az = Accel_Z_RAW/16384.0;



		HAL_UART_Transmit(&huart2, &Ax, sizeof(Ax),10);
		HAL_UART_Transmit(&huart2, &Ay, sizeof(Ay),10);
		HAL_UART_Transmit(&huart2, &Az, sizeof(Az),10);

		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		//HAL_Delay(500);
		//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		//HAL_Delay(500);
		//HAL_UART_Transmit(&huart2, &val1, sizeof(val1),10);
		//HAL_UART_Transmit(&huart2, xaxis, sizeof(xaxis),10);
		//HAL_UART_Transmit(&huart2, &Gx, sizeof(Gx),10);
		//HAL_UART_Transmit(&huart2, yaxis, sizeof(yaxis),10);
		//HAL_UART_Transmit(&huart2, &Gy, sizeof(Gx),10);
		//HAL_UART_Transmit(&huart2, zaxis, sizeof(zaxis),10);
		//HAL_UART_Transmit(&huart2, &Gz, sizeof(Gz),10);
		//uint16_t test = 12;

		//HAL_UART_Transmit(&huart2, &test, sizeof(test),10);
		//HAL_UART_Transmit(&huart2, "abc", sizeof("abc"),10);
		//HAL_UART_Transmit(&huart2, &Gy, sizeof(Gy),10);
		//HAL_UART_Transmit(&huart2, ",", sizeof(","),10);
		//HAL_UART_Transmit(&huart2, &Gz, sizeof(Gz),10);
		//HAL_UART_Transmit(&huart2, "\n", sizeof("\n"),10);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		HAL_Delay(10);




		//HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDR,0x75,&check,1,1000);








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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
