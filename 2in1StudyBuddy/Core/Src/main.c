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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
//#include "stm32f4xx_hal_rtc.h"
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
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
GPIO_PinState timeUpButton;
GPIO_PinState timeDownButton;
GPIO_PinState correctButton;
GPIO_PinState incorrectButton;

int correctCounter = 0;

typedef struct LEDPin_struct {
	GPIO_TypeDef* Port;
	uint16_t Pin;

} LEDPin;

uint8_t turnedOff = 0;

LEDPin LEDPinArray[4];
int currentLED = 0;

int timeInput = 0;

int prevTime = 0;
int timeOffset;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

//Time things
//void set_time(void);
int getTime();
void getTimeOffsetSec(int *prevTime, int *timeOffset);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
//time function declarations
/*void set_time(void){
	RTC_TimeTypeDef sTime = {0};
	//RTC_TimeTypeDef sDate;
	sTime.Hours = 0;
	sTime.Minutes = 0;
	sTime.Seconds = 0;
	  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  //sDate.WeekDay = RTC_WEEKDAY_MONDAY;
	  //sDate.Month = RTC_MONTH_JANUARY;
	  //sDate.Date = 0x1;
	  //sDate.Year = 0x0;
	  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
	  {
	    Error_Handler();
	  }
}*/
int getTime()
{
	RTC_TimeTypeDef gTime;
	RTC_DateTypeDef gDate;
	//This places the time data into gTime
	HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
	//This print requires a delay (5ms works)
	HAL_Delay(5);
	printf("\rTime: %02d:%02d:%02d\r\n", gTime.Hours, gTime.Minutes, gTime.Seconds);
	// *time = gTime.Seconds;
	//even though we don't need date, the time doesn't update without getting the date
	HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
	return gTime.Seconds;
}


void getTimeOffsetSec(int *prevTime, int *timeOffset){
	int currentTime = getTime();
	*timeOffset = currentTime - *prevTime;


	if(*timeOffset < 0){
		*timeOffset = 60 - *prevTime;
	}
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
  // from left to right LED
  LEDPin D7LED;
  D7LED.Port = GPIOA;
  D7LED.Pin = D7_Pin;

  LEDPin D2LED;
  D2LED.Port = GPIOA;
  D2LED.Pin = D2_Pin;

  LEDPin D4LED;
  D4LED.Port = GPIOB;
  D4LED.Pin = D4_Pin;

  LEDPin D10LED;
  D10LED.Port = GPIOB;
  D10LED.Pin = D10_Pin;


  LEDPinArray[0] = D7LED;
  LEDPinArray[1] = D2LED;
  LEDPinArray[2] = D4LED;
  LEDPinArray[3] = D10LED;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
// huart2.Instance = USARTx; used for print
huart2.Init.BaudRate = 115200;
// huart2.Init.WordLength = UART_WORDLENGTH_88; used for print
huart2.Init.StopBits = UART_STOPBITS_1;
huart2.Init.Parity = UART_PARITY_ODD;
huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
huart2.Init.Mode = UART_MODE_TX_RX;
huart2.Init.OverSampling = UART_OVERSAMPLING_16;
//call set time to initialize time
//set_time is breaking things
//printf("setting time\r\n");
//set_time();
//printf("done setting time\r\n");
/* if(HAL_UART_INIT(&huart2) != HAL_OK) {
	  Error_Handler();
} */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1 == 1)
{
	  /*stateOfButton = HAL_GPIO_ReadPin(GPIOC, D9_Pin);
	  HAL_GPIO_WritePin(GPIOA, D7_Pin, stateOfButton);
	  HAL_GPIO_WritePin(GPIOB, D5_Pin, stateOfButton);
	  HAL_GPIO_WritePin(GPIOA, D2_Pin, stateOfButton); */
	  //printf("getting time\n\r");
	  //currentTime = getTime();
	  //time offset example
	  //built-in led will blink for a duration of 2 sec every 15 sec
	  //at new minute it messes up because goes over (last time was 45) new time is 0.
	  //timeOffset = currentTime - endTime;
	  //fix time offset
	  //if(timeOffset < 0){HAL_GPIO_WritePin(GPIOA, D7_Pin, 1);
		 // timeOffset = 60 - endTime;
	  //}


	 // getTimeOffsetSec(&prevTime, &timeOffset);
	  //printf("%d\r\n", timeOffset);

	 //turn on LEDs

	 timeUpButton = HAL_GPIO_ReadPin(GPIOA, D8_Pin);
	 timeDownButton = HAL_GPIO_ReadPin(GPIOC, D9_Pin);
	 correctButton = HAL_GPIO_ReadPin(GPIOA, D11_Pin);
	 incorrectButton = HAL_GPIO_ReadPin(GPIOA, D12_Pin);


	 if (timeUpButton == 1) {
		 htim2.Instance->CCR3 = 75; // set servo to 90 degrees

		 prevTime = getTime();

		 timeInput += 15;

		 ++currentLED;
		 HAL_GPIO_WritePin(LEDPinArray[currentLED].Port, LEDPinArray[currentLED].Pin, 1);


		 if (currentLED > 3) {
			 currentLED = 3;
		 }
		 if (timeInput > 60) {
		 	timeInput = 60;
		 }

		 HAL_Delay(200);
	 }


	 if (timeDownButton == 1) {
		 prevTime = getTime();


		 timeInput -= 15;
		 HAL_GPIO_WritePin(LEDPinArray[currentLED].Port, LEDPinArray[currentLED].Pin, 0);
		 --currentLED;

		 if (currentLED <= 0) {
		 		currentLED = -1;
		 }
		 if (timeInput < 0) {
		 	 timeInput = 0;
		 }
	 	 HAL_Delay(200);
	 }

	getTimeOffsetSec(&prevTime, &timeOffset);
	if ( (timeOffset == 15 || timeOffset == 30 || timeOffset == 45 || timeOffset == 60) && (turnedOff == 0) ) {
		HAL_GPIO_WritePin(LEDPinArray[currentLED].Port, LEDPinArray[currentLED].Pin, 0);
		--currentLED;
		if (currentLED < 0) {
			currentLED = 0;
		}

		turnedOff = 1; // flag to make sure that the if statement only runs for the current LED once.
	}

	if ( (timeOffset == 14 || timeOffset == 29 || timeOffset == 44 || timeOffset == 59) && (turnedOff == 1) ) {
			turnedOff = 0; // reset the flag when not at the specified timeframes
		}

	if (correctButton == 1) {
		++correctCounter;
		HAL_Delay(200);
	}

	if (correctCounter == 4 ) {


		timeInput -= 15;
		HAL_GPIO_WritePin(LEDPinArray[currentLED].Port, LEDPinArray[currentLED].Pin, 0);
		--currentLED;

		if (currentLED < 0) {
			currentLED = 0;
		}

		correctCounter = 0;

	}

	if (incorrectButton == 1) {
		correctCounter = 0;
		HAL_Delay(200);
	}

	if ( HAL_GPIO_ReadPin(LEDPinArray[0].Port, LEDPinArray[0].Pin) == 0) {
		correctCounter = 0;
		currentLED = -1;
		timeInput = 0;
		htim2.Instance->CCR3 = 25; // set servo to 0 degrees
	}

	printf("\r%d %d\n", correctCounter, currentLED);






	//



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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 90;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 900-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  HAL_GPIO_WritePin(GPIOA, Green_LED_Pin|D7_Pin|D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D5_Pin|D4_Pin|D10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Green_LED_Pin D7_Pin D2_Pin */
  GPIO_InitStruct.Pin = Green_LED_Pin|D7_Pin|D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D12_Pin D11_Pin D8_Pin */
  GPIO_InitStruct.Pin = D12_Pin|D11_Pin|D8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : D9_Pin */
  GPIO_InitStruct.Pin = D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(D9_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D5_Pin D4_Pin D10_Pin */
  GPIO_InitStruct.Pin = D5_Pin|D4_Pin|D10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}
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
while (1 == 1)
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
