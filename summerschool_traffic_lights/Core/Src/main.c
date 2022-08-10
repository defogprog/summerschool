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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	false, true
} bool;
typedef enum {
	RED, GREEN, YELLOW
} LED;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
bool isButtonClicked = false;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
void setParalelTrafficLightsState(GPIO_TypeDef*, GPIO_TypeDef*, uint16_t,
		uint16_t, GPIO_PinState);
void pedestrianBlinkindGreenLight(uint16_t);
void setLedColorState(GPIO_PinState, LED);
void trafficLightsBlinking(uint16_t);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void setParalelTrafficLightsState(GPIO_TypeDef *GPIO_PORT_TF_1,
		GPIO_TypeDef *GPIO_PORT_TF_2, uint16_t GPIO_Pin_TF_1,
		uint16_t GPIO_Pin_TF_2, GPIO_PinState PinState) {
	HAL_GPIO_WritePin(GPIO_PORT_TF_1, GPIO_Pin_TF_1, PinState);
	HAL_GPIO_WritePin(GPIO_PORT_TF_2, GPIO_Pin_TF_2, PinState);
}
/**
 * Generate a blinking sequence of the green led of the pedestrian
 * traffic lights to mark that vehicles will be allowed to drive soon
 */
void pedestrianBlinkindGreenLight(uint16_t delayTimeGap) {
	for (int i = 0; i < 10; i++) {
		if (i % 2)
			HAL_GPIO_WritePin(GREEN_LED_OFF_GPIO_Port, GREEN_LED_OFF_Pin,
					GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GREEN_LED_OFF_GPIO_Port, GREEN_LED_OFF_Pin,
					GPIO_PIN_RESET);
		HAL_Delay(delayTimeGap * 0.05);
	}
}
/**
 * Change the state of the led elements to light specific LEDs on the circuit
 */
void setLedColorState(GPIO_PinState pinState, LED ledColor) {
	switch (ledColor) {
	case RED:
		setParalelTrafficLightsState(RED_LED_TRAFFIC_GPIO_Port,
		RED_LED_2_GPIO_Port, RED_LED_TRAFFIC_Pin, RED_LED_2_Pin, pinState);
		break;
	case YELLOW:
		setParalelTrafficLightsState(YELLOW_LED_TRAFFIC_GPIO_Port,
		YELLOW_LED_2_GPIO_Port, YELLOW_LED_TRAFFIC_Pin, YELLOW_LED_2_Pin,
				pinState);
		break;
	case GREEN:
		setParalelTrafficLightsState(GREEN_LED_TRAFFIC_GPIO_Port,
		GREEN_LED_2_GPIO_Port, GREEN_LED_TRAFFIC_Pin, GREEN_LED_2_Pin,
				pinState);
		break;
	}
}
/**
 * Define the blinking sequences of led elements according to the UK
 * traffic lights conventions
 */
void trafficLightsBlinking(uint16_t delayTimeGap) {
	// red off, yellow off, green on
	setLedColorState(GPIO_PIN_SET, RED);
	setLedColorState(GPIO_PIN_SET, YELLOW);
	setLedColorState(GPIO_PIN_RESET, GREEN);
	HAL_Delay(delayTimeGap * 0.8);

	// green off, yellow on
	setLedColorState(GPIO_PIN_RESET, YELLOW);
	setLedColorState(GPIO_PIN_SET, GREEN);
	HAL_Delay(delayTimeGap * 0.5);

	// yellow off, red on
	setLedColorState(GPIO_PIN_RESET, RED);
	setLedColorState(GPIO_PIN_SET, YELLOW);
	HAL_Delay(delayTimeGap * 0.1);

	HAL_GPIO_WritePin(GREEN_LED_OFF_GPIO_Port, GREEN_LED_OFF_Pin,
			GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RED_LED_ON_GPIO_Port, RED_LED_ON_Pin, GPIO_PIN_SET);
	HAL_Delay(delayTimeGap);

	// yellow on
	pedestrianBlinkindGreenLight(delayTimeGap);
	setLedColorState(GPIO_PIN_RESET, YELLOW);
	HAL_Delay(delayTimeGap * 0.1);

	HAL_GPIO_WritePin(GREEN_LED_OFF_GPIO_Port, GREEN_LED_OFF_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RED_LED_ON_GPIO_Port, RED_LED_ON_Pin, GPIO_PIN_RESET);
	HAL_Delay(delayTimeGap * 0.3);

	// green on, yellow off, red off
	setLedColorState(GPIO_PIN_SET, RED);
	setLedColorState(GPIO_PIN_SET, YELLOW);
	setLedColorState(GPIO_PIN_RESET, GREEN);
	HAL_Delay(delayTimeGap * 0.4);

}
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
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* check whether user has clicked a button, changing the state of a program after the click */
		if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin)) {
			isButtonClicked = !isButtonClicked;
			HAL_Delay(200);
		}

		/**
		 * if the button is clicked, generate the traffic lights sequence with specified timers to show
		 * drivers that pedestrians are going to cross the street.If no person intends to cross the
		 * street, the traffic lights fire only one green color bulb, indicating the free passage ahead.
		 */
		if (isButtonClicked) {
			trafficLightsBlinking(4000);
			isButtonClicked = !isButtonClicked;

		} else {
			// Switch the pedestrian signals to the forbidden state, meaning that no one is allowed to
			// go on the road.(GREEN is off, and RED is on)
			HAL_GPIO_WritePin(RED_LED_ON_GPIO_Port, RED_LED_ON_Pin,
					GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GREEN_LED_OFF_GPIO_Port, GREEN_LED_OFF_Pin,
					GPIO_PIN_SET);

			// Set free pass for the cars on the road
			setLedColorState(GPIO_PIN_RESET, GREEN);

		}

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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
			RED_LED_2_Pin | YELLOW_LED_2_Pin | GREEN_LED_2_Pin | RED_LED_ON_Pin
					| GREEN_LED_OFF_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	GREEN_LED_TRAFFIC_Pin | YELLOW_LED_TRAFFIC_Pin | RED_LED_TRAFFIC_Pin,
			GPIO_PIN_SET);

	/*Configure GPIO pins : RED_LED_2_Pin ORANGE_LED_2_Pin GREEN_LED_2_Pin RED_LED_ON_Pin
	 GREEN_LED_OFF_Pin */
	GPIO_InitStruct.Pin = RED_LED_2_Pin | YELLOW_LED_2_Pin | GREEN_LED_2_Pin
			| RED_LED_ON_Pin | GREEN_LED_OFF_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : BUTTON_Pin */
	GPIO_InitStruct.Pin = BUTTON_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : GREEN_LED_TRAFFIC_Pin YELLOW_LED_TRAFFIC_Pin RED_LED_TRAFFIC_Pin */
	GPIO_InitStruct.Pin = GREEN_LED_TRAFFIC_Pin | YELLOW_LED_TRAFFIC_Pin
			| RED_LED_TRAFFIC_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	HAL_GPIO_WritePin(RED_LED_ON_GPIO_Port, RED_LED_ON_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GREEN_LED_OFF_GPIO_Port, GREEN_LED_OFF_Pin,
			GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}
/* USER CODE END 4 */

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
