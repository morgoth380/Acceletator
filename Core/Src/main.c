/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
typedef enum{
	MOVE_WAIT = 0,
	LED_ON = 1
}ledStateType;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FIFO_CTRL 0x2E

#define OUT_X_L_A 0x28
#define OUT_X_H_A 0x29

#define OUT_Y_L_A 0x2A
#define OUT_Y_H_A 0x2B

#define OUT_Z_L_A 0x2C
#define OUT_Z_H_A 0x2D

#define CTRL_REG4 0x1E
#define CTRL_REG1_A 0x20
#define CTRL_REG5_XL 0x1F

#define INT_GEN_THS_Z_XL 0x09
#define INT_GEN_THS_Y_XL 0x08
#define INT_GEN_THS_X_XL 0x07
#define INT_GEN_CFG_XL 0x06
#define INT1_CTRL 0x0C
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
uint16_t cmd;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void lsm303CInit(void);
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
  uint16_t DevAddress;
  uint16_t data;
  uint8_t Rdata;
  uint8_t RdataH;
  uint16_t writeData;
  int16_t accel_X, accel_Y, accel_Z;
  float accelX, accelY, accelZ;
  static ledStateType state = MOVE_WAIT;
  static uint32_t startTick = 0;
  uint32_t tickVal;
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
  lsm303CInit();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  DevAddress = 0x3A; //Адрес акселерометра по умолчанию

	  //Ускорение по оси X
	  data = OUT_X_L_A;
	  HAL_I2C_Master_Transmit(&hi2c1, DevAddress, (uint8_t *)(&data), 1, 1000);
	  HAL_I2C_Master_Receive(&hi2c1, DevAddress, (uint8_t *)&Rdata, 1, 1000);
	  data = OUT_X_H_A;
	  HAL_I2C_Master_Transmit(&hi2c1, DevAddress, (uint8_t *)(&data), 1, 1000);
	  HAL_I2C_Master_Receive(&hi2c1, DevAddress, (uint8_t *)&RdataH, 1, 1000);
	  accel_X = (int16_t)((uint16_t)Rdata | ((uint16_t)RdataH << 8));
	  accelX = (2.0 / 32767) * accel_X; //Ускорение по оси X

	  //Ускорение по оси Y
	  data = OUT_Y_L_A;
	  HAL_I2C_Master_Transmit(&hi2c1, DevAddress, (uint8_t *)(&data), 1, 1000);
	  HAL_I2C_Master_Receive(&hi2c1, DevAddress, (uint8_t *)&Rdata, 1, 1000);
	  data = OUT_Y_H_A;
	  HAL_I2C_Master_Transmit(&hi2c1, DevAddress, (uint8_t *)(&data), 1, 1000);
	  HAL_I2C_Master_Receive(&hi2c1, DevAddress, (uint8_t *)&RdataH, 1, 1000);
	  accel_Y = (int16_t)((uint16_t)Rdata | ((uint16_t)RdataH << 8));
	  accelY = (2.0 / 32767) * accel_Y; //Ускорение по оси Y

	  //Ускорение по оси Z
	  data = OUT_Z_L_A;
	  HAL_I2C_Master_Transmit(&hi2c1, DevAddress, (uint8_t *)(&data), 1, 1000);
	  HAL_I2C_Master_Receive(&hi2c1, DevAddress, (uint8_t *)&Rdata, 1, 1000);
	  data = OUT_Z_H_A;
	  HAL_I2C_Master_Transmit(&hi2c1, DevAddress, (uint8_t *)(&data), 1, 1000);
	  HAL_I2C_Master_Receive(&hi2c1, DevAddress, (uint8_t *)&RdataH, 1, 1000);
	  accel_Z = (int16_t)((uint16_t)Rdata | ((uint16_t)RdataH << 8));
	  accelZ = (2.0 / 32767) * accel_Z; //Ускорение по оси Y

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

//Настройка режима работы чипа акселерометра
void lsm303CInit(void)
{
	uint8_t DevAddress;
	uint16_t writeData;

	DevAddress = 0x3A;
	writeData = CTRL_REG1_A;
	writeData = (uint16_t)writeData | ((uint16_t)0x17 << 8);
	HAL_I2C_Master_Transmit(&hi2c1, DevAddress, (uint8_t *)(&writeData), 2, 1000);
	return;
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
