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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "baro.h"
#include "lcd.h"
#include <stdio.h>	// snprintf
#include <string.h>	// memset
#include <math.h>	// sin
#include <stdlib.h>	// rand

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define countof(_a)	(sizeof(_a) / sizeof(_a[0]))
#define UART_RX_BUF_SIZE	200

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for taskLEDBlink */
osThreadId_t taskLEDBlinkHandle;
const osThreadAttr_t taskLEDBlink_attributes = {
  .name = "taskLEDBlink",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for taskButtonRead */
osThreadId_t taskButtonReadHandle;
const osThreadAttr_t taskButtonRead_attributes = {
  .name = "taskButtonRead",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for muxUART */
osMutexId_t muxUARTHandle;
const osMutexAttr_t muxUART_attributes = {
  .name = "muxUART"
};
/* Definitions for muxLCD */
osMutexId_t muxLCDHandle;
const osMutexAttr_t muxLCD_attributes = {
  .name = "muxLCD"
};
/* Definitions for sem1 */
osSemaphoreId_t sem1Handle;
osStaticSemaphoreDef_t sem1ControlBlock;
const osSemaphoreAttr_t sem1_attributes = {
  .name = "sem1",
  .cb_mem = &sem1ControlBlock,
  .cb_size = sizeof(sem1ControlBlock),
};
/* Definitions for sem2 */
osSemaphoreId_t sem2Handle;
osStaticSemaphoreDef_t sem2ControlBlock;
const osSemaphoreAttr_t sem2_attributes = {
  .name = "sem2",
  .cb_mem = &sem2ControlBlock,
  .cb_size = sizeof(sem2ControlBlock),
};
/* USER CODE BEGIN PV */
static uint8_t rx_buffer[UART_RX_BUF_SIZE];
static uint8_t *rx_head = rx_buffer, *rx_tail = rx_buffer;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
void StartTaskLEDBlink(void *argument);
void StartTaskButtonRead(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char text[100];

int __io_getchar(void) {
	rx_head = &rx_buffer[UART_RX_BUF_SIZE - hdma_usart1_rx.Instance->NDTR];

	while (rx_tail == rx_head) {
		rx_head = &rx_buffer[UART_RX_BUF_SIZE - hdma_usart1_rx.Instance->NDTR];
	}

	uint8_t b = *rx_tail;

	if (++rx_tail == (rx_buffer + UART_RX_BUF_SIZE))
		rx_tail = rx_buffer;

	return (int)b;
}

int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 100);
	return 0;
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  if (HAL_UART_Receive_DMA(&huart1, rx_buffer, sizeof(rx_buffer)) != HAL_OK)
  {
	  snprintf(text, countof(text), "Error start UART RX %d\n", __LINE__);
	  HAL_UART_Transmit(&huart1, (uint8_t*)text, strnlen(text, countof(text)), 1000);
	  while (1) {}
  }

  //snprintf(text, countof(text), "Start UART RX %d\n", __LINE__);
  //HAL_UART_Transmit(&huart1, (uint8_t*)text, strnlen(text, countof(text)), 1000);

  if (baro_init() != BARO_OK)
  {
	  snprintf(text, countof(text), "Error init baro\n");
	  HAL_UART_Transmit(&huart1, (uint8_t*)text, strnlen(text, countof(text)), 1000);
	  while (1) {}
  }

//  for (uint16_t p = 0; p < 100; p++) {
//	  lcd_pixel(p, p+0, ST7735_RED);
//	  lcd_pixel(p, p+1, ST7735_GREEN);
//	  lcd_pixel(p, p+2, ST7735_BLUE);
//  }
//
//  lcd_fill_rect(10, 10, 50, 50, ST7735_CYAN);
//  lcd_fill_rect(50, 50, 150, 150, ST7735_MAGENTA);
//  lcd_rect(5, 5, 15, 15, ST77XX_ORANGE);
//  lcd_rect(200, 200, 15, 15, ST77XX_ORANGE);
//  lcd_line(13, 19, 37, 93, ST77XX_GREEN);
//  lcd_line(13, 19, 93, 37, ST77XX_GREEN);
//  lcd_circle(22, 55, 76, ST77XX_RED);
//  lcd_fill_circle(33, 66, 19, ST77XX_BLUE);
//  lcd_print("Hello LCD!");
//  lcd_set_text_color(ST7735_CYAN);
//  lcd_set_text_bg_color(ST7735_ORANGE);
//  lcd_print("\nNew line!");

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of muxUART */
  muxUARTHandle = osMutexNew(&muxUART_attributes);

  /* creation of muxLCD */
  muxLCDHandle = osMutexNew(&muxLCD_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of sem1 */
  sem1Handle = osSemaphoreNew(1, 1, &sem1_attributes);

  /* creation of sem2 */
  sem2Handle = osSemaphoreNew(1, 1, &sem2_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of taskLEDBlink */
  taskLEDBlinkHandle = osThreadNew(StartTaskLEDBlink, NULL, &taskLEDBlink_attributes);

  /* creation of taskButtonRead */
  taskButtonReadHandle = osThreadNew(StartTaskButtonRead, NULL, &taskButtonRead_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  //lcd_init();
  //baro_init();

  sem1Handle = osSemaphoreNew(1, 1, &sem1_attributes);//producer should starts first
  sem2Handle = osSemaphoreNew(1, 0, &sem2_attributes);

  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RESET_Pin|LCD_CS_Pin|LCD_A0_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RESET_Pin LCD_CS_Pin LCD_A0_Pin */
  GPIO_InitStruct.Pin = LCD_RESET_Pin|LCD_CS_Pin|LCD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTaskLEDBlink */
/**
  * @brief  Function implementing the taskLEDBlink thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTaskLEDBlink */
void StartTaskLEDBlink(void *argument)//producer
{
  /* USER CODE BEGIN 5 */

	uint32_t pressure = 0;
	uint32_t temperature = 0;

	//osSemaphoreRelease(sem2Handle);
	/* Infinite loop */
	for(;;)
	{
		pressure = baro_read_press();
		temperature = baro_read_temp();

		osSemaphoreAcquire(sem1Handle, osWaitForever);//sync with consumer

		osMutexAcquire(muxUARTHandle, osWaitForever);

		sprintf(text, "*+%ld.%02ld,%ld.%02ld+*\n", temperature/100, temperature%100, pressure/100, pressure%100);

		osMutexRelease(muxUARTHandle);


		osSemaphoreRelease(sem2Handle);
		osDelay(pdMS_TO_TICKS(100));

		//osSemaphoreAcquire(sem2Handle, osWaitForever);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskButtonRead */
/**
* @brief Function implementing the taskButtonRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskButtonRead */
void StartTaskButtonRead(void *argument)//consumer
{
  /* USER CODE BEGIN StartTaskButtonRead */
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreAcquire(sem2Handle, osWaitForever);//sync with producer

	  osMutexAcquire(muxUARTHandle, osWaitForever);

	  HAL_UART_Transmit(&huart1, (uint8_t *)text, strlen(text), HAL_MAX_DELAY);

	  osMutexRelease(muxUARTHandle);

	  osSemaphoreRelease(sem1Handle);
	  osDelay(pdMS_TO_TICKS(100));

  }
  /* USER CODE END StartTaskButtonRead */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
