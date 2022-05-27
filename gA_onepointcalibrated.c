/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
 I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define MPU6050_ADDR 0xD0


#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75


int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Position_X=0;
float Position_Y=0;
float Position_Z=0;

float Rotation_X=0;
float Rotation_Y=0;
float Rotation_Z=0;

float Ax, Ay, Az, Gx, Gy, Gz;
float Ax_cal, Ay_cal, Az_cal, Gx_cal, Gy_cal, Gz_cal, Gx_clean, Gy_clean, Gz_cean; /*calibration values*/
int wait_time=10;
char snum[7];				/*buffer for int to string trafo*/
char calibration_string[] = "calibration values are:";
char spacesPx[] = "Px  ";
char spacesPy[] = "Py  ";
char spacesPz[] = "Pz  ";

char spacesAx[] = "Ax  ";
char spacesAy[] = "Ay  ";
char spacesAz[] = "Az  ";

char spacesGx[] = "Gx  ";
char spacesGy[] = "Gy  ";
char spacesGz[] = "Gz  ";

char spacesDeltaGx[] = "ΔGx  ";
char spacesDeltaGy[] = "ΔGy  ";
char spacesDeltaGz[] = "ΔGz  ";

uint8_t Rec_Data[6];   /*receiving buffer for getACC&Gyro*/


char buffer[20];

void MPU6050_Init (void)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 °/s
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}
	/*char buffer[20];
*/
	 // convert 123 to string [buf]
	sprintf(buffer, "%x", check);

	 HAL_UART_Transmit(&huart2, buffer, sizeof(buffer), 10);
	 char Spaces[] = "   ";
	 HAL_UART_Transmit(&huart2, Spaces, sizeof(Spaces), 10);
}


void getAcc(void){


	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

		Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
		Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
		Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

		/*** convert the RAW values into acceleration in 'g'
		     we have to divide according to the Full scale value set in FS_SEL
		     I have configured FS_SEL = 0. So I am dividing by 16384.0
		     for more details check ACCEL_CONFIG Register              ****/

		Ax= Accel_X_RAW/16384.0;
		Ay= Accel_Y_RAW/16384.0;
		Az= Accel_Z_RAW/16384.0;

}

void calibration_mutiple (void){
	int x=1000;
	int y=x;
	while(x>0) {
		getAcc();
		getgyro();

		Gx_cal=Gx+Gx_cal;
		Gy_cal=Gy+Gy_cal;
		Gz_cal=Gz+Gz_cal;

		Ax_cal=Ax+Ax_cal;
		Ay_cal=Ay+Ay_cal;
		Az_cal=Az+Az_cal;

		HAL_Delay (wait_time/10);  // wait for 1 sec
		x=x-1;
	}
		Ax_cal=Ax_cal/y;
		Ay_cal=Ay_cal/y;
		Az_cal=Az_cal/y;

		Gx_cal=Gx_cal/y;
		Gy_cal=Gy_cal/y;
		Gz_cal=Gz_cal/y;

	HAL_UART_Transmit(&huart2, calibration_string, sizeof(calibration_string), 10);

	gcvt(Ax_cal, 15, snum);
	HAL_UART_Transmit(&huart2, snum, sizeof(snum), 10);
	HAL_UART_Transmit(&huart2, spacesAx, sizeof(spacesAx), 10);


	gcvt(Ay_cal, 15, snum);
	HAL_UART_Transmit(&huart2, snum, sizeof(snum), 10);
	HAL_UART_Transmit(&huart2, spacesAy, sizeof(spacesAx), 10);


	gcvt(Az_cal, 15, snum);
	HAL_UART_Transmit(&huart2, snum, sizeof(snum), 10);
	HAL_UART_Transmit(&huart2, spacesAz, sizeof(spacesAx), 10);

		gcvt(Gx_cal, 15, snum);
		HAL_UART_Transmit(&huart2, snum, sizeof(snum), 10);
		HAL_UART_Transmit(&huart2, spacesDeltaGx, sizeof(spacesDeltaGx), 10);


		gcvt(Gy_cal, 15, snum);
		HAL_UART_Transmit(&huart2, snum, sizeof(snum), 10);
		HAL_UART_Transmit(&huart2, spacesDeltaGy, sizeof(spacesDeltaGx), 10);


		gcvt(Gz_cal, 15, snum);
		HAL_UART_Transmit(&huart2, snum, sizeof(snum), 10);
		HAL_UART_Transmit(&huart2, spacesDeltaGz, sizeof(spacesDeltaGx), 10);

	HAL_UART_Transmit(&huart2, "\n\n\n", sizeof("\n\n\n"), 10);
}

void print_position_cal (void) /*look at /1000/1000 for format */
{
	getAcc();

	Ax = Ax-Ax_cal;
	Ay = Ay-Ay_cal;
	Az = Az-Az_cal;

/* devide by /1000/10 for cm*/
	Position_X=Position_X+0.5*Ax*wait_time*wait_time/1000/10;
	Position_Y=Position_Y+0.5*Ay*wait_time*wait_time/1000/10;
	Position_Z=Position_Z+0.5*Az*wait_time*wait_time/1000/10;



	gcvt(Position_X, 15, snum);
	HAL_UART_Transmit(&huart2, snum, sizeof(snum), 10);
	HAL_UART_Transmit(&huart2, spacesPx, sizeof(spacesPx), 10);


	gcvt(Position_Y, 15, snum);
	HAL_UART_Transmit(&huart2, snum, sizeof(snum), 10);
	HAL_UART_Transmit(&huart2, spacesPy, sizeof(spacesPx), 10);


	gcvt(Position_Z, 15, snum);
	HAL_UART_Transmit(&huart2, snum, sizeof(snum), 10);
	HAL_UART_Transmit(&huart2, spacesPz, sizeof(spacesPx), 10);

}

void print_Accel_cal (void){

	getAcc();
	Ax = Ax-Ax_cal;
	Ay = Ay-Ay_cal;
	Az = Az-Az_cal;





	gcvt(Ax, 15, snum);
	HAL_UART_Transmit(&huart2, snum, sizeof(snum), 10);
	HAL_UART_Transmit(&huart2, spacesAx, sizeof(spacesAx), 10);


	gcvt(Ay, 15, snum);
	HAL_UART_Transmit(&huart2, snum, sizeof(snum), 10);
	HAL_UART_Transmit(&huart2, spacesAy, sizeof(spacesAx), 10);


	gcvt(Az, 15, snum);
	HAL_UART_Transmit(&huart2, snum, sizeof(snum), 10);
	HAL_UART_Transmit(&huart2, spacesAz, sizeof(spacesAx), 10);

}

void print_Accel (void){

	getAcc();

	gcvt(Ax, 15, snum);
	HAL_UART_Transmit(&huart2, snum, sizeof(snum), 10);
	HAL_UART_Transmit(&huart2, spacesAx, sizeof(spacesAx), 10);


	gcvt(Ay, 15, snum);
	HAL_UART_Transmit(&huart2, snum, sizeof(snum), 10);
	HAL_UART_Transmit(&huart2, spacesAy, sizeof(spacesAx), 10);


	gcvt(Az, 15, snum);
	HAL_UART_Transmit(&huart2, snum, sizeof(snum), 10);
	HAL_UART_Transmit(&huart2, spacesAz, sizeof(spacesAx), 10);

}

void getgyro(void){

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into dps (°/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

	Gx = Gyro_X_RAW/131.0;
	Gy = Gyro_Y_RAW/131.0;
	Gz = Gyro_Z_RAW/131.0;

}

void print_delta_gyro(){
	getgyro();

	gcvt(Gx, 15, snum);
	HAL_UART_Transmit(&huart2, snum, sizeof(snum), 10);
	HAL_UART_Transmit(&huart2, spacesDeltaGx, sizeof(spacesDeltaGz), 10);

	gcvt(Gy, 15, snum);
	HAL_UART_Transmit(&huart2, snum, sizeof(snum), 10);
	HAL_UART_Transmit(&huart2, spacesDeltaGy, sizeof(spacesDeltaGz), 10);

	gcvt(Gz, 15, snum);
	HAL_UART_Transmit(&huart2, snum, sizeof(snum), 10);
	HAL_UART_Transmit(&huart2, spacesDeltaGz, sizeof(spacesDeltaGz), 10);


}




void print_rotation (float x, float y, float z) /*xyz sind calib konstanten 0 für reine werte */
{
	getgyro();

/* devide by x/1000/10 for cm*/
	Rotation_X=Rotation_X+(Gx-x)*wait_time/1000;
	Rotation_Y=Rotation_Y+(Gy-y)*wait_time/1000;
	Rotation_Z=Rotation_Z+(Gz-z)*wait_time/1000;

	if (Rotation_X>359.99){
		Rotation_X=0;
	}
	if (Rotation_Y>359.99){
			Rotation_Y=0;
		}
	if (Rotation_Y>359.99){
			Rotation_Y=0;
		}


	gcvt(Rotation_X, 15, snum);
	HAL_UART_Transmit(&huart2, snum, sizeof(snum), 10);
	HAL_UART_Transmit(&huart2, spacesGx, sizeof(spacesGx), 10);


	gcvt(Rotation_Y, 15, snum);
	HAL_UART_Transmit(&huart2, snum, sizeof(snum), 10);
	HAL_UART_Transmit(&huart2, spacesGy, sizeof(spacesGx), 10);


	gcvt(Rotation_Z, 15, snum);
	HAL_UART_Transmit(&huart2, snum, sizeof(snum), 10);
	HAL_UART_Transmit(&huart2, spacesGz, sizeof(spacesGx), 10);

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Transmit(&huart2, "\n\n\n", sizeof("\n\n\n"), 10);
  MPU6050_Init();
  calibration_mutiple();




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // read the Accelerometer and Gyro values

	  print_rotation(Gx_cal,Gy_cal,Gz_cal);
	  // print the Acceleration and Gyro values on the LCD 20x4



	  HAL_Delay (wait_time);  // wait for a while
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

