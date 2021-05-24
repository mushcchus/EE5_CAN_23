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
#include "stdio.h"
#include "string.h"
#include "CAN_ID.h"
#include "dgus.h"
#include "DFPLAYER_MINI.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define NODE_ID  NODE_DASHBOARD //dec node 6
#define NODE_ID NODE_EXTRA1 //dec node 23
#define NODE_ID_MASK 0x3E0

//#define CAN_DEBUG 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// Debugging:
#define ITM_Port32(n)	(*((volatile unsigned long *)(0xE0000000+4*n)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

	CAN_RxHeaderTypeDef CAN_RxHeader;
	CAN_FilterTypeDef filter;
	uint8_t CAN_RxBuffer[6];
	uint8_t bestLap = 0xFF;
	uint8_t bestLapC = 0x01;
	uint8_t numLap = 0x00;
	uint8_t currentLap = 0xFF;
	int8_t	steering = 0x00;
	uint8_t steeringL = 0xFF;
	uint8_t steeringR = 0xFF;
	uint8_t speed = 0x00;
	int8_t steer_left = 0;
	int8_t steer_right = 0;
	int8_t no_steer = 0;

	//for printf debug
	uint8_t msg_rcvd = 0;
	uint8_t no_msgs_rcvd = 0;
	uint8_t msg_length = 0;
	uint8_t no_msgs_rcvd;

	//	for CAN debug.
	uint32_t pTxMailbox_debug;
	CAN_TxHeaderTypeDef pTxHeader_debug;
	CAN_RxHeaderTypeDef pRxHeader_debug;
	CAN_FilterTypeDef sFilterConfig_debug;
	uint8_t data_out_debug, data_in_debug;

	uint8_t speed_count = 0;
	uint8_t totalLaps_count = 0;
	uint8_t lastlaptime_count = 0;
	uint8_t currentlaptime_count = 0;
	uint8_t bestlaptime_count = 0;
	uint8_t carposition_count = 0;
	uint8_t currentlapnum_count= 0;
	uint8_t throttle_count = 0;
	uint8_t brake_count = 0;
	uint8_t gear_count = 0;
	uint8_t lights_count = 0;
	uint8_t tiressurfacetempRL_count = 0;
	uint8_t tiressurfacetempRR_count = 0;
	uint8_t tiressurfacetempFR_count = 0;
	uint8_t tiressurfacetempFL_count = 0;
	uint8_t enginetemperature_count = 0;
	uint8_t safetycarstatus_count = 0;
	uint8_t steer_count = 0;
	uint8_t brake_value = 0;
	uint8_t throttle_value = 0;
	uint16_t speed_value  = 0;
	uint16_t speed_value_MSB = 0;
	uint16_t speed_value_LSB = 0;
	uint8_t steer_value = 0;


	uint16_t CCR1print = 0;
	uint16_t CCR2print = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
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
  DF_Init(0x10);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  //	Debug purposes for printf() debug:
  ITM_Port32(31) = 1;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  //	Debug purposes for printf() debug:
  ITM_Port32(31) = 2;


  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterIdHigh = NODE_ID <<5;
  filter.FilterMaskIdLow = 0x000;
  filter.FilterMaskIdHigh = NODE_ID_MASK;
  filter.FilterMaskIdLow = 0;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterActivation = ENABLE;

  HAL_CAN_ConfigFilter(&hcan, &filter);



//	  //	pTxHeader configuration for CAN debug
//	  pTxHeader_debug.DLC = 1;
// 	  pTxHeader_debug.IDE = CAN_ID_STD;
// 	  pTxHeader_debug.RTR = CAN_RTR_DATA;
// 	  pTxHeader_debug.StdId = 0x245;
//
// 	  //	sFilter configuration for CAN debug.
// 	  sFilterConfig_debug.FilterFIFOAssignment = CAN_FILTER_FIFO0;
// 	  sFilterConfig_debug.FilterIdHigh = 0x244 <<5;
// 	  sFilterConfig_debug.FilterMaskIdLow = 0;
// 	  sFilterConfig_debug.FilterMaskIdHigh = 0x000;
// 	  sFilterConfig_debug.FilterMaskIdLow = 0;
// 	  sFilterConfig_debug.FilterScale = CAN_FILTERSCALE_32BIT;
// 	  sFilterConfig_debug.FilterActivation = ENABLE;
//// 	  	Setup the filter in the registers and enable interrupt to receive CAN packets for CAN debug.
// 	  HAL_CAN_ConfigFilter(&hcan, &sFilterConfig_debug);



   HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
   //	Setup remaining CAN configurations.
   HAL_CAN_Start(&hcan);

   HAL_TIM_PWM_Start (&htim2, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start (&htim2, TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //printf("Listening..\n");
	  if (msg_rcvd == 1) {
		switch (*CAN_RxBuffer)   {

		  case  0x76 :
			 printf("Received  Safety Car Status: ");
			   if (CAN_RxBuffer[1]>0){
			 	  DF_SafetyCar();
			 	  }
			  break;
		  case  0x75 :
			 printf("Received Total Laps: ");
			  Send_totalLap(CAN_RxBuffer[1]);
			//  totallaps_count++;
			  break;

		  case  0x70 :
			 printf("Received  Last Lap Time: ");
			  Send_pTime(CAN_RxBuffer);
			 // lastlap_count++;
			  break;

		  case 0x71 :
			  printf("Received Current Lap Time: ");
			  Send_cTime(CAN_RxBuffer);
			  //currentlap_count++;
			  break;

		  case  0x72 :
			  printf("Received Best Lap Time: ");
			  Send_bTime(CAN_RxBuffer);
			  bestLapC = Get_time(CAN_RxBuffer);
			  if (bestLapC < bestLap){
			 	  bestLap = bestLapC;
			 	  DF_BestLap();
			  }
			  break;

		  case  0x73 :
			  printf("Received Car Position: ");
			  Send_position(CAN_RxBuffer[1]);
			  break;

		  case  0x74 :
			  printf("Received Current Lap Number: ");
			  Send_currentLap(CAN_RxBuffer[1]);
			  currentLap = CAN_RxBuffer[1];
			  if (currentLap +1 == numLap){
			  	  DF_LastLap();
			  }
			  break;

		  case  0x77 :
			  speed_count++;
			  if (0x00 == CAN_RxBuffer[2]){
			  			  speed = CAN_RxBuffer[1];
			  }
			  else {
			    speed = 0xFF;
			  }

			  speed_count++;
			  speed_value_MSB = CAN_RxBuffer[2];
			  speed_value_MSB >>= 7;
			  speed_value_LSB = CAN_RxBuffer[1];
			  speed_value = speed_value_LSB + speed_value_MSB;
//			  if(speed_count == 100) {
//				  speed_count = 0;
//			  }
			  printf("Received Speed: ");
			  Send_speed(CAN_RxBuffer[2], CAN_RxBuffer[1]);

			  CCR1print = speed * (100 + steering) / 200;
			  CCR2print = speed * (100 - steering) / 200;

			  htim2.Instance->CCR1 = (speed/8) + (speed * (100 + steering) / 200); //speed * (100 + steering) / 200 ;
			  htim2.Instance->CCR2 = (speed/8) + (speed * (100 - steering) / 200);

			  break;

		  case  0x06d :
			  printf("Received Throttle: ");
			  Send_throttle(CAN_RxBuffer[1]);
			  throttle_value = CAN_RxBuffer[1];
			  break;

		  case  0x3 :
			  printf("Received Brake: ");
			  for (int i = 0; i < (msg_length - 1); i++) {
			  			  printf("%x ", CAN_RxBuffer[i + 1]);
			  		  }
			  		  printf("\n");
			  Send_brake(CAN_RxBuffer[1]);
			  brake_count++;
			  brake_value = CAN_RxBuffer[1];
			  if(brake_count == 100) {
				  brake_count = 0;
			  }
			  break;

		  case  0x6E :
			  printf("Received Gear: ");
			  Send_gear(CAN_RxBuffer[1]);
			  break;

		  case  0x6F :
			  printf("Received Rev Lights Indicator: ");
			  Send_light(CAN_RxBuffer[1]);
			  lights_count++;
			  if(lights_count == 100) {
				  lights_count = 0;
			  			  }
			  break;

		  case  0x7 :
			 printf("Received Tire Temperature FR: ");
			  break;

		  case  0xA :
			  printf("Received Tire Temperature FL: ");
			  break;

		  case  0xD :
			  printf("Received Tire Temperature RR: ");
			  break;

		  case  0x10 :
			  printf("Received Tire Temperature RL: ");
			  break;

		  case  0x43 :
			  printf("Received Engine Temperature: ");
			  break;

		  case  0x6 :
		 	  printf("Received Steer: ");
		 	  steering = CAN_RxBuffer[1];

   		      if (steering < 100) {
		 		  steer_left = steering;
		 	  }
		 	  if (steering >0) {
		 		  steer_right = steering;
		 			 	  }
		 	  if (steering == 0) {
		 		  no_steer = steering;
		 			 	  }

		 	  steer_value = CAN_RxBuffer[1];
		 	  break;

		  default :
			  printf("Received variable not recognized. HEADER: %x. Header + Data content: ", *CAN_RxBuffer);
			  for (int i = 0; i < (msg_length); i++) {
			  			  printf("%x ", CAN_RxBuffer[i]);
			  		  }
			  		  printf("\n");
			  break;
			}

		  for (int i = 0; i < (msg_length - 1); i++) {
			  printf("%x ", CAN_RxBuffer[i + 1]);
		  }
		  printf("\n");
		  msg_rcvd = 0;
	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 15;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
//	Function for debugging with printf()
int _write(int file, char *ptr, int len)
	{
	  /* Implement your write code here, this is used by puts and printf for example */
	  int DataIdx;
	  for(DataIdx=0 ; DataIdx<len ; DataIdx++)
	    ITM_SendChar(*ptr++);
	  return len;
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
