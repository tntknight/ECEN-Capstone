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
#include "motors.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COUNT 7
#define KD 2
#define READY_MSG "READY"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* Macro: 		PRINT_STR2
 * Description:	Takes an 8 bit array and transmits via UART2
 * Parameters:	An array to be sent via the UART2
 */
#define PRINT_STR2(msg) do  { \
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100); \
} while(0)

/* Macro: 		PRINT_STR1
 * Description:	Takes an 8 bit array and transmits via UART1
 * Parameters:	An array to be sent via the UART1
 */
#define PRINT_STR1(msg) do  { \
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100); \
} while(0)


/* Macro: 		NANO_RECEIVE
 * Description:	Receives up to 30 bits from UART2 and stores them using the provided pointer
 * Parameters:	A pointer to an array to hold the transmitted message
 */
#define NANO_RECEIVE(str) do { \
	HAL_UART_Receive (&huart2, str, 30, 100);\
} while(0)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;
DMA_HandleTypeDef hdma_tim1_ch4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
// Global Variables
uint8_t msg[60];		// Generic 8 bit array to store messages in.
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */
void LED_On(void);
void LED_Off(void);
void print_network_information(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char *data = "Hello World from USB CDC\n";

/*
 * Function:	LED_On
 * Description:	Turns the status LED on.
 */
void LED_On() {
	HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
}

/*
 * Function:	LED_Off
 * Description:	Turns the status LED off.
 */
void LED_Off() {
	HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
}

/*
 * Function: 	getXY
 * Description: Takes a string and processes out the X and Y coordinates.
 * Parameters:	uint8_t* 	str	Message string formated in x|y format
 * Returns;		uint16_t* 		A two item array with X stored at the 0 spot and Y stored in 1
 */
uint16_t * getXY(uint8_t *str) {
	// Function variables
	uint8_t i, m = 0;
	static  uint16_t xy[2];
    char strX[30], strY[30];

    // Loops through the array
	for (i = 0; i < 100; i++){
		// Copies first part of string into X string
		if (m == 0) {
			strX[i] = str[i];
		// Copies second part of string into Y string
		} else {
			strY[i-(m+1)] = str[i];
		}
		// If the character '|' is found this the end of the X string
		if (str[i] == '|') {
			m = i;
			strX[i] = '\0'; // Replaces '|' with end of string character
		}
		// Once we have reached the end of the input sting we break the loop
		if (str[i] == '\0')
			break;
	}
	xy[0] = atoi(strX);	// Convert the X string to an number
	xy[1] = atoi(strY); // Convert the Y string to an number

	return xy; // Return both X and Y as numbers in an array
}
/*
 * Function: 	launcher_On
 * Description: Sets control line to launcher high.
 */
void launcher_On() {
	HAL_GPIO_WritePin(GPIOB, LAUNCHER_Pin, GPIO_PIN_SET);
}

/* Function: 	launcher_Off
 * Description: Sets control line to launcher low.
 */
void launcher_Off() {
	HAL_GPIO_WritePin(GPIOB, LAUNCHER_Pin, GPIO_PIN_RESET);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	// Main function variables
	uint8_t str[30];				// Array to store received messages in
	uint16_t *xy; 					// Array to store XY coordinates pulled from message
	uint16_t x[COUNT], y[COUNT];	// Arrays to store tracked XY coordinates
	uint16_t delta_sumX, delta_sumY;// Arrays to store total change between tracked points
	uint16_t tXd, tYd;				// Average change between tracked points
	uint16_t lY, lX; 				// Last tracked XY coordinates
	uint16_t tarY, tarX;			// Target XY coordinates;
	int elev, rot;					// Control values for motors
	uint8_t noTarget = 0;			// Wether or not an object is being trakced
	uint8_t dir;					// Direction the object was tracked moving
	uint8_t dirList[4] = {0,0,0,0};	// Array storing the counts for which direction the object is moving
	uint8_t dirLarg = 0; 			// Largest direction counted yet
	uint8_t j = 0;					// General purpose variable
	uint8_t ready[strlen(READY_MSG) +1];// Ready message
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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
	// Set GPIO pins being used for ground to ground
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 | GPIO_PIN_9, GPIO_PIN_RESET);

  // Initialize stating position
	set_LiftMotor(0);		// Set elevation motor to half elevation
	launcher_Off();			// Set launcher to off
	LED_On();					// Pulse LED for 50 ms to signal end of initialization
	HAL_Delay(50);			//
	LED_Off();				//

	HAL_Delay(2000);
	move_To_Position(1000, 70);

	// Fire Launcher
	LED_Off();
	launcher_On();
	HAL_Delay(100);	// VERY IMPORTANT DELAY CHANGING THIS COULD DESTROY ACTURATOR CONTROL
	launcher_Off();
	LED_On();
	HAL_Delay(500);

	move_To_Position(-1000, 0);

	while(1);

	// Wait for ready message from Jetson Nano
	do{
		HAL_UART_Receive (&huart2, ready, strlen(READY_MSG), 100);
		ready[5] = '\0';
	} while (strcmp((char*)ready,READY_MSG) != 0);
	// Wait for Jetson to be ready to send data
	LED_On();
	HAL_Delay(8000);
	LED_Off();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	// Main control loop
	while (1) {
		// Set total change to zero
		delta_sumX = 0;
		delta_sumY = 0;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// Wait as long as received data is nothing
	  	do {
	  		NANO_RECEIVE(str);
	  	} while (str[0] == '0');

	  	// Once target is acquired set value and turn on LED
	  	noTarget = 1;
	  	LED_On();

	  	xy = getXY(str);	// Get XY coordinates from data string
	  	x[0] = xy[0];		// Save X coordinate
	  	y[0] = xy[1];		// Save Y coordinate

	  	// Print out data received to serial terminal
	  	sprintf((char*)msg, "%d|%d\r\n", x[0],y[0]);
	  	PRINT_STR1((char*)msg);

	  	// Continue collecting points until we have enough to plot track
	  	for (int i = 1; i < COUNT; i++) {
	  		NANO_RECEIVE(str);				// Receive XY data string
		  	if (str[0] == '0') {			// If no object detected
		  		i--;						// Reduce count by one
		  	} else {
		  		xy = getXY(str);			// Get XY coordinates from data string
		  		if (xy[0] == 0 || xy[1] == 0) {
		  			i--;					// If both coordinates are zero reduce count by one
		  		} else {
		  			x[i] = xy[0];			// Save X coordinate
		  			y[i] = xy[1];			// Save Y coordinate

		  			// Compute which direction the object is tracking in
		  			if (x[i-1] > x[i])
		  				if(y[i-1] > y[i])
		  					dirList[0]++;	// Tracking down and to the left
		  				else
		  					dirList[1]++;	// Tracking up and to the left
		  			else
		  				if(y[i-1] > y[i])
		  					dirList[3]++;	// Tracking down and to the right
		  				else
		  					dirList[4]++;	// Tracking up and to the right

		  			// Print out data received to serial terminal
		  			sprintf((char*)msg, "%d|%d\r\n", x[i],y[i]);
		  			PRINT_STR1((char*)msg);

		  			// Update sum of directional change
		  			delta_sumX += abs(x[i] - x[i-1]);
		  			delta_sumY += abs(y[i] - y[i-1]);
		  		} // End of valid data else
		  	} // End of detection data else
	  	} // End of tracking loop

	  	// Tracking complete turn off LED
	  	LED_Off();

	  	// Compute average change in tracked points and multiply by constant
	  	tXd = (delta_sumX / COUNT) * KD;
	  	tYd = (delta_sumY / COUNT) * KD;

	  	// Save last positions tracked
	  	lX = x[COUNT - 1];
	  	lY = y[COUNT - 1];

	  	// Derive  which direction the tracking tended to
	  	dir = 0; // Set default direction
	  	for (uint8_t i = 0; i < 4; i++){	// Loop through counts of direction
	  		if (dirList[i] > dirLarg){		// If direction has larger count
	  			dir = i;					// save it as the new direction
	  		}
	  	}

	  	// Computes XY target based on direction
	  	switch(dir) {
	  	case 0:				// Up and to the left
	  		tarX = lX - tXd;
	  		tarY = lY - tYd;
	  		break;
	  	case 1:				// Down and to the left
	  		tarX = lX - tXd;
	  		tarY = lY + tYd;
	  		break;
	  	case 2:				// Up and to the right
	  		tarX = lX + tXd;
	  		tarY = lY - tYd;
	  		break;
	  	case 3:				// Down and to the right
	  		tarX = lX + tXd;
	  		tarY = lY + tYd;
	  		break;
	  	}

	  	// Output all target and tracking data to serial terminal
	  	sprintf((char*)msg, "T:%d|%d\r\n", tarX,tarY);
	  	PRINT_STR1((char*)msg);
	  	sprintf((char*)msg, "AveDX: %d AveDY: %d SumDX: %d SumDY: %d Dir: %d\r\n", tXd, tYd, delta_sumX, delta_sumY, dir);
	  	PRINT_STR1((char*)msg);

	  	// Calculate elevation control value from Y target coordinate
	  	elev = 100 - (tarY / 10.8); // new value is 0 to 100 percent extension
	  	if (elev < 50)				// If less than zero
	  		elev = 50;				// value is zero
	  	if (elev > 100)				// If greater than 100
	  		elev = 100;				// value is 100

	  	// Calculate rotation control value from X target coordinate
	  	rot = (tarX - 1920)/2.4;	// Compute rotation motor control value
	  	if (rot < -800)				// If less than -400
	  		rot = -800;				// value is -400
	  	else if (rot > 800)			// If greater than 400
	  		rot = 800;				// value is 400

	  	// Output motor control values to serial terminal
	  	sprintf((char*)msg, "rot: %d elev: %d \r\n", rot, elev);
	  	PRINT_STR1((char*)msg);

	  	// Turn on status light to show motor movement is imminent
	  	LED_On();

	  	move_To_Position(rot, (uint8_t)elev);	// Send motor control values this
	  											// This function ends when motors have reached there target

	  	// Fire Launcher
	  	LED_Off();
	  	launcher_On();
	  	HAL_Delay(100);	// VERY IMPORTANT DELAY CHANGING THIS COULD DESTROY ACTURATOR CONTROL
	  	launcher_Off();
	  	LED_On();

	  	HAL_Delay(3000);			// Wait three seconds
	  	LED_Off();					// Turn off LED to signal end of launching process
	  	move_To_Position(-rot, 0);	// Return motor to home position

	  	while(1);					// Keep the microprocessor busy
	} // End of main control loop
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 64-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 5000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 64;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_STATUS_Pin|W5500_RST_Pin|W5500_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIR__Pin|LAUNCHER_Pin|UNUSED_Pin|DIR_B5_Pin
                          |GROUND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_STATUS_Pin */
  GPIO_InitStruct.Pin = LED_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_STATUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR__Pin LAUNCHER_Pin DIR_B5_Pin GROUND_Pin */
  GPIO_InitStruct.Pin = DIR__Pin|LAUNCHER_Pin|DIR_B5_Pin|GROUND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : W5500_RST_Pin W5500_CS_Pin */
  GPIO_InitStruct.Pin = W5500_RST_Pin|W5500_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : UNUSED_Pin */
  GPIO_InitStruct.Pin = UNUSED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(UNUSED_GPIO_Port, &GPIO_InitStruct);

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
