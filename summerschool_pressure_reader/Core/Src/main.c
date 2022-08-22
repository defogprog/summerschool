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
#include <stdio.h>
#include <string.h>
#include "baro.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define countof(_a) (sizeof(_a)/sizeof(_a[0]))

#define MAX(a, b) (a > b) ? a : b

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for taskPressureOut */
osThreadId_t taskPressureOutHandle;
const osThreadAttr_t taskPressureOut_attributes = { .name = "taskPressureOut",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for taskPressureRea */
osThreadId_t taskPressureReaHandle;
const osThreadAttr_t taskPressureRea_attributes = { .name = "taskPressureRea",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for queueHandler */
osMessageQueueId_t queueHandlerHandle;
const osMessageQueueAttr_t queueHandler_attributes = { .name = "queueHandler" };
/* Definitions for muxUART */
osMutexId_t muxUARTHandle;
const osMutexAttr_t muxUART_attributes = { .name = "muxUART" };
/* Definitions for semButtonPressed */
osSemaphoreId_t semButtonPressedHandle;
osStaticSemaphoreDef_t semButtonPressedControlBlock;
const osSemaphoreAttr_t semButtonPressed_attributes = { .name =
		"semButtonPressed", .cb_mem = &semButtonPressedControlBlock, .cb_size =
		sizeof(semButtonPressedControlBlock), };
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);
void StartTaskPressureOutput(void *argument);
void StartTaskPressureRead(void *argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char text[100];
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USART1_UART_Init();
	MX_SPI1_Init();
	/* USER CODE BEGIN 2 */

	if (baro_init() != BARO_OK) {
		snprintf(text, strlen(text), "Error init baro\n");
		HAL_UART_Transmit(&huart1, (uint8_t*) text,
				strnlen(text, countof(text)), 1000);
		while (1) {
		}
	}

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();
	/* Create the mutex(es) */
	/* creation of muxUART */
	muxUARTHandle = osMutexNew(&muxUART_attributes);

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* creation of semButtonPressed */
	semButtonPressedHandle = osSemaphoreNew(1, 1, &semButtonPressed_attributes);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* creation of queueHandler */
	queueHandlerHandle = osMessageQueueNew(32, sizeof(uint32_t),
			&queueHandler_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	if (queueHandlerHandle == NULL) {
		char *str = "Unable to create Integer Queue\n\n";
		HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), HAL_MAX_DELAY);
	} else {
		char *str = "Integer Queue Created Successfully\n\n";
		HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), HAL_MAX_DELAY);
	}

	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
			&defaultTask_attributes);

	/* creation of taskPressureOut */
	taskPressureOutHandle = osThreadNew(StartTaskPressureOutput, NULL,
			&taskPressureOut_attributes);

	/* creation of taskPressureRea */
	taskPressureReaHandle = osThreadNew(StartTaskPressureRead, NULL,
			&taskPressureRea_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

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
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
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
static void MX_SPI1_Init(void) {

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
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
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
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LCD_RESET_Pin | LCD_CS_Pin | LCD_A0_Pin,
			GPIO_PIN_SET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_RESET_Pin LCD_CS_Pin LCD_A0_Pin */
	GPIO_InitStruct.Pin = LCD_RESET_Pin | LCD_CS_Pin | LCD_A0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : BUTTON_Pin */
	GPIO_InitStruct.Pin = BUTTON_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskPressureOutput */
/**
 * @brief Function implementing the taskPressureOut thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskPressureOutput */
void StartTaskPressureOutput(void *argument) {
	/* USER CODE BEGIN StartTaskPressureOutput */
	/* Infinite loop */
	for (;;) {
		osStatus_t status;
		int32_t pressure;

		// Read the value of the pressure from the queue
		status = osMessageQueueGet(queueHandlerHandle, &pressure, NULL, 100);

		// Write a formatted data to the string text
		snprintf(text, countof(text), "Pressure: %ld.%02ld\n", pressure / 100,
				pressure % 100);

		// Check whether the operation has been completed without failure and output the result to the UART port
		if (status == osOK) {
			HAL_UART_Transmit(&huart1, (uint8_t*) text, strlen(text), 1000);
		} else {
			snprintf(text, strlen(text),
					"Failure while acquiring data from the other task");
			HAL_UART_Transmit(&huart1, (uint8_t*) text, strlen(text), 1000);
		}

		// Make a delay of 500 ms after each iteration
		osDelay(pdMS_TO_TICKS(500));
	}
	/* USER CODE END StartTaskPressureOutput */
}

/* USER CODE BEGIN Header_StartTaskPressureRead */
/**
 * @brief Function implementing the taskPressureRea thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskPressureRead */
void StartTaskPressureRead(void *argument) {
	/* USER CODE BEGIN StartTaskPressureRead */
	/* Infinite loop */
	for (;;) {

		// Initialize the value of the variable with the atmospheric pressure from the BMP280 sensor
		int32_t pressure = baro_read_press();
		// Put the value with the pressure data from the sensor to the queue
		osMessageQueuePut(queueHandlerHandle, &pressure, 0U, 100);
		// Make a delay of 500 ms after each iteration
		osDelay(pdMS_TO_TICKS(500));
	}
	/* USER CODE END StartTaskPressureRead */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
