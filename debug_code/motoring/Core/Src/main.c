/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
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
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)

{   int DataIdx;

    for(DataIdx = 0; DataIdx < len; DataIdx ++)

    {
    	ITM_SendChar(*ptr ++);
    }

     return len;

}

void arprint(uint8_t* ar, uint8_t len)
{
	for (int i = 0; i < len; i++)
	{
		printf("%x ", ar[i]);
	}
	printf("\n");
}

void set_nucleo_baud(UART_HandleTypeDef *huart, uint32_t br)
{
  //HAL_Delay(100);

  if (HAL_UART_DeInit(huart) != HAL_OK) {
	  printf("Failed to deinit UART\n");
  }
  huart->Init.BaudRate = br;
  if (HAL_UART_Init(huart) != HAL_OK) {
	  printf("Failed to set baud rate %" PRIu32 "\n", br);
  } else {
	  printf("baud rate now %" PRIu32 "\n", br);
  }

  //HAL_Delay(100);
}

uint8_t bio_chksm(uint8_t* buffer)
{
	uint32_t sm = 0;
	for (int i = 0; i < buffer[3] + 1; i++) {
		sm += buffer[2 + i];
	}
	//printf("got checksum %x\n", ~((uint8_t)sm));
	return ~((uint8_t)sm);
}

void set_servo_baud(UART_HandleTypeDef *huart)
{
  uint8_t *test = (uint8_t*)calloc(8, sizeof(uint8_t));
  test[0] = 0xFF;
  test[1] = 0xFF;
  test[2] = 0xFE;
  test[3] = 0x04;
  test[4] = 0x03;
  test[5] = 0x04;
  test[6] = 0x01;
  test[7] = 0xF5;

  HAL_HalfDuplex_EnableTransmitter(huart);

  if (HAL_UART_Transmit(huart, test, 8, 2000) != HAL_OK) {
	  printf("Failed to send set baud mode command via uart\n");
  } else {
	  printf("Baud rate set via broadcast mode to %u\n", test[6]);
  }
}

void baud_set_loop(UART_HandleTypeDef *huart)
{
  uint32_t br_v[] = {1000000, 500000, 400000, 250000, 200000, 115200, 57600, 19200, 9600};
  for (int i = 0; i < 9; i++) {
	  set_nucleo_baud(huart, br_v[i]);
	  set_servo_baud(huart);
  }
}

void set_servo_id(UART_HandleTypeDef *huart)
{
  uint8_t *test = (uint8_t*)calloc(8, sizeof(uint8_t));
  test[0] = 0xFF;
  test[1] = 0xFF;
  test[2] = 0xFE;
  test[3] = 0x04;
  test[4] = 0x03;
  test[5] = 0x03;
  test[6] = 0x02;
  test[7] = 0xF5;

  HAL_HalfDuplex_EnableTransmitter(huart);

  if (HAL_UART_Transmit(huart, test, 8, 2000) != HAL_OK) {
	  printf("Failed to send set id command via uart\n");
  } else {
	  printf("ID set via broadcast mode to %u\n", test[6]);
  }
}

void move_servo(UART_HandleTypeDef *huart)
{
  uint8_t *test = (uint8_t*)calloc(9, sizeof(uint8_t));
  test[0] = 0xFF;
  test[1] = 0xFF;
  test[2] = 0x03;
  test[3] = 0x05;
  test[4] = 0x03;
  test[5] = 0x1E;
  test[6] = 0x02;
  test[7] = 0x00;
  test[8] = bio_chksm(test);

  HAL_HalfDuplex_EnableTransmitter(huart);

  if (HAL_UART_Transmit(huart, test, 9, 2000) != HAL_OK) {
	  printf("Failed to send move command via uart\n");
  } else {
	  printf("Move goal set via broadcast mode");
  }
}


void set_servo_led(UART_HandleTypeDef *huart, uint8_t* ret)
{

  printf("value of ret before: ");
  arprint(ret, 6);

  uint8_t *test = (uint8_t*)calloc(8, sizeof(uint8_t));
  test[0] = 0xFF;
  test[1] = 0xFF;
  test[2] = 0x03;
  test[3] = 0x04;
  test[4] = 0x03;
  test[5] = 0x19;
  test[6] = 0x01;
  test[7] = bio_chksm(test);

  arprint(test, 8);

  HAL_HalfDuplex_EnableTransmitter(huart);
  if (HAL_UART_Transmit(huart, test, 8, 2000) != HAL_OK) {
	  //printf("Failed to send set led command via uart\n");
  } else {
	//  printf("Set led set via broadcast mode\n");
  }
  HAL_HalfDuplex_EnableReceiver(huart);
  uint8_t retval;
  if ((retval = HAL_UART_Receive(huart, ret, 6, 10000)) != HAL_OK) {
	  printf("Failed to read data via uart: %u\n", retval);
  } else {
	  printf("Read data via uart\n");
  }

  printf("value of ret after: ");
  arprint(ret, 6);

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
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  /*uint8_t *ret = (uint8_t*)calloc(9, sizeof(uint8_t));

  HAL_HalfDuplex_EnableTransmitter(&huart4);
  HAL_UART_Transmit (&huart4, test, 8, 2000);

  HAL_HalfDuplex_EnableReceiver(&huart4);
  HAL_UART_Receive(&huart4, ret, 9, 2000);*/

  //baud_set_loop(&huart4);
  //set_nucleo_baud(&huart4, 1000000);
  //set_servo_id(&huart4);
  //for (uint32_t b = 1500000; b > 2000; b -= 1000) {
	//  set_nucleo_baud(&huart4, b);
	 // set_servo_led(&huart4);
	//  HAL_Delay(150);
  //}

/*  uint32_t br_v[] = {1000000, 500000, 400000, 250000, 200000, 115200, 57600, 19200, 9600};
  for (int i = 0; i < 9; i++) {
	  set_nucleo_baud(&huart4, br_v[i]);
	  HAL_Delay(2000);
	  move_servo(&huart4);
	  HAL_Delay(2000);
  }*/

  //set_nucleo_baud(&huart4, 1000000);
  uint8_t *ret = (uint8_t*)calloc(6, sizeof(uint8_t));
  set_servo_led(&huart4, ret);
  HAL_Delay(2000);
  move_servo(&huart4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 1000000;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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

/* USER CODE BEGIN MX_GPIO_Init_2 */
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
	printf("Got error\n");
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
