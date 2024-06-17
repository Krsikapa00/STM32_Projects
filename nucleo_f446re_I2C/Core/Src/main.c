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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdbool.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADXL345_ADDRESS 0x53 << 1
#define ADXL345_DEVID 0x00
#define ADXL345_THRESH_TAP 0x1D
#define ADXL345_TAP_DUR 0x21 //Max time for something to be considered a tap, any more then it isnt?
#define ADXL345_TAP_LAT 0x22 //
#define ADXL345_WINDOW 0x23
#define ADXL345_TAP_AXES 0x2A
#define ADXL345_ACT_TAP_STATUS 0x2B
#define ADXL345_POWER_CTL 0x2D
#define ADXL345_INT_ENABLE 0x2E //Enable tap interupts
#define ADXL345_INT_MAP 0x2F //Map which interupts go to which INT pin
#define ADXL345_INT_SOURCE 0x30 //Status of interupts?
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_DATAX0 0x32
#define GRAVITY_SCALE 0.003906
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float	latest_X;
float	latest_Y;
float	latest_Z;
bool	measuring_data;
char 	char_buff[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

HAL_StatusTypeDef ADXL345_Write(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return HAL_I2C_Master_Transmit(&hi2c1, ADXL345_ADDRESS, data, 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ADXL345_Read(uint8_t reg, uint8_t *buffer, uint16_t size) {
//	Check to see if sensor is ready to communicate. If so then read sensor
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, ADXL345_ADDRESS, &reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;
    return HAL_I2C_Master_Receive(&hi2c1, ADXL345_ADDRESS, buffer, size, HAL_MAX_DELAY);
}

void ADXL345_Init(void) {
    uint8_t buffer;
    // Check device ID
    ADXL345_Read(ADXL345_DEVID, &buffer, 1);
    if (buffer == 0xE5) {

        //Map Interrupts (Single tap on Interrupt 1, Double on Interrupt 2)
		ADXL345_Write(ADXL345_INT_MAP, 0x40);
        //Enable Interrupts
		ADXL345_Write(ADXL345_INT_ENABLE, 0x60);
        // Set Tap threshold (2g = 0.0625* 32) = 0x20
        ADXL345_Write(ADXL345_THRESH_TAP, 0x18);
        // Set Tap duration 10ms
        ADXL345_Write(ADXL345_TAP_DUR, 0x18);
        // Set Tap latency
        ADXL345_Write(ADXL345_TAP_LAT, 0x20);
        // Set Double Tap Window
		ADXL345_Write(ADXL345_WINDOW, 0x18);
		// Enable Tap Axes (Only Z-Axis)
		ADXL345_Write(ADXL345_TAP_AXES, 0x01);
		// Enable measurement mode
		ADXL345_Write(ADXL345_POWER_CTL, 0x08);
		// Set data format to full resolution, +-16g
		ADXL345_Write(ADXL345_DATA_FORMAT, 0x0B);

		snprintf(char_buff, sizeof(char_buff),"CONNECTED & Setup\r\n");

    } else
    {
		snprintf(char_buff, sizeof(char_buff),"ERROR\r\n");
    }
	HAL_UART_Transmit(&huart2, (uint8_t *)char_buff, strlen(char_buff), HAL_MAX_DELAY);


}

void ADXL345_Get_Data(void)
{
	/*Setup buffer and variables for different directions*/
	uint8_t data_buff[6];
	int16_t x_raw, y_raw, z_raw;
	//	Read data from sensor
	HAL_StatusTypeDef status = ADXL345_Read(ADXL345_DATAX0, data_buff, 6);
	/* X0 = data_buff[0], X1 = data_buff[1]
	 * Y0 = data_buff[0], Y1 = data_buff[2]
	 * Z0 = data_buff[0], Z1 = data_buff[5]
	 */
	if (status == HAL_OK) //Make sure the reading is valid
	{
		x_raw = ((int16_t)data_buff[1] << 8) | (data_buff[0]);
		y_raw = ((int16_t)data_buff[3] << 8) | (data_buff[2]);
		z_raw = ((int16_t)data_buff[5] << 8) | (data_buff[4]);

		latest_X = x_raw * GRAVITY_SCALE;
		latest_Y = y_raw * GRAVITY_SCALE;
		latest_Z = z_raw * GRAVITY_SCALE;
        snprintf(char_buff, sizeof(char_buff), "RAW DATE: [X: %d |Y: %d | Z:%d] \r\n", x_raw, y_raw, z_raw);
		HAL_UART_Transmit(&huart2, (uint8_t *)char_buff, strlen(char_buff), HAL_MAX_DELAY);

	}


}

/* Interrupt Functions START*/
void EXTI15_10_IRQHandler(void)
{
	// ISR for Interrupt 1 on ADXL (Double Tap)
	uint8_t adxl_source;
	// ISR for Interrupt 2 on ADXL (Single Tap)
	if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_10) != RESET) //Check if pin is raised
	{
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_10); //Clear pin


		if (ADXL345_Read(ADXL345_INT_SOURCE, adxl_source, 1) == HAL_OK)
		{
			//ADXL interrupt flags is in ADXL_source. Check if single tap bit is set
			if (adxl_source & 0x40) {
//				Handle the Single tap logic
				snprintf(char_buff, sizeof(char_buff), "Single Tap\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t *)char_buff, strlen(char_buff), HAL_MAX_DELAY);

//				Toggle if the sensor is recording data or not
				measuring_data = !measuring_data;

//				If the sensor is currently measuring data, Save the latest recorded data
//				if (measuring_data)
//				{
//					snprintf(char_buff, sizeof(char_buff), "CONVERTED:[X: %.3f |Y: %.3f | Z:%.3f] \r\n", latest_X, latest_Y, latest_Z);
//					HAL_UART_Transmit(&huart2, (uint8_t *)char_buff, strlen(char_buff), HAL_MAX_DELAY);
//				}

			}
		} else {

			snprintf(char_buff, sizeof(char_buff), "Could not read Interrupt source\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t *)char_buff, strlen(char_buff), HAL_MAX_DELAY);

		}
	} else if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_12) != RESET) //Check if pin is raised
	{
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12); //Clear pin


		if (ADXL345_Read(ADXL345_INT_SOURCE, adxl_source, 1) == HAL_OK)
		{
			//ADXL interrupt flags is in ADXL_source. Check if single tap bit is set
			if (adxl_source & 0x20) {
//				Handle the Double tap logic
				snprintf(char_buff, sizeof(char_buff), "Double Tap\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t *)char_buff, strlen(char_buff), HAL_MAX_DELAY);




			}
		} else {
			snprintf(char_buff, sizeof(char_buff), "Could not read Interrupt source\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t *)char_buff, strlen(char_buff), HAL_MAX_DELAY);

		}
	}

}



/* Interrupt Functions END*/


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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  ADXL345_Init();
  measuring_data = true;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (measuring_data)
	  {
		  ADXL345_Get_Data();
	  }
	  HAL_Delay(500);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Interrupt_2_Pin Interrupt_1_Pin */
  GPIO_InitStruct.Pin = Interrupt_2_Pin|Interrupt_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
/* USER CODE END MX_GPIO_Init_2 */
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
