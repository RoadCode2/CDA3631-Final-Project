/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file : main.c
* @brief : Main program body
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "event_groups.h"
#include "stdlib.h"
//PA7 = g, PA9 = f, PA8 = a, PA10 = b, PA0 = e, PA1 = d, PA6 = c
#define a GPIO_PIN_8// Defining variables
#define b GPIO_PIN_10
#define c GPIO_PIN_6
#define d GPIO_PIN_1
#define e GPIO_PIN_0
#define f GPIO_PIN_9
#define g GPIO_PIN_7
#define ReverseLED GPIO_PIN_5
#define ForwardLED GPIO_PIN_4

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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

osThreadId DirectionTaskHandle;
osThreadId SpeedTaskHandle;
osThreadId SegmentTaskHandle;
osThreadId DisplayTaskHandle;
/* USER CODE BEGIN PV */
volatile uint8_t Movement; //Variables for tasks
volatile uint8_t PowerLevel;
volatile uint8_t Velocity;
char buff[20];
char Way[56];
// uart related
uint8_t rx_data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDirectionTask(void const * argument);
void StartSpeedTask(void const * argument);
void StartSegmentTask(void const * argument);
void StartDisplayTask(void const * argument);

/* USER CODE BEGIN PFP */
void UART_SEND(UART_HandleTypeDef *huart, char buffer[]) //Transmitting UART Data
{
  HAL_UART_Transmit(huart, (uint8_t*) buffer, strlen(buffer), HAL_MAX_DELAY);
}
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); //PB10 TIM2 CH3
  HAL_UART_Receive_IT(&huart2, &rx_data, 1);
  UART_SEND(&huart2, "Starting....\r\n");
  while (1)
  {
      /* USER CODE END WHILE */

  	/* USER CODE BEGIN 3 */
  }
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  /* definition and creation of DirectionTask */
  osThreadDef(DirectionTask, StartDirectionTask, osPriorityAboveNormal, 0, 128);
  DirectionTaskHandle = osThreadCreate(osThread(DirectionTask), NULL);

  /* definition and creation of SpeedTask */
  osThreadDef(SpeedTask, StartSpeedTask, osPriorityNormal, 0, 128);
  SpeedTaskHandle = osThreadCreate(osThread(SpeedTask), NULL);

  /* definition and creation of SegmentTask */
  osThreadDef(SegmentTask, StartSegmentTask, osPriorityBelowNormal, 0, 128);
  SegmentTaskHandle = osThreadCreate(osThread(SegmentTask), NULL);

  /* definition and creation of DisplayTask */
  osThreadDef(DisplayTask, StartDisplayTask, osPriorityLow, 0, 128);
  DisplayTaskHandle = osThreadCreate(osThread(DisplayTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
{
    /* USER CODE END WHILE */
	  for(int j = 0; j < 6; j++)
	  {

	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 89;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|LD2_Pin
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 LD2_Pin
                           PA6 PA7 PA8 PA9
                           PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|LD2_Pin
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDirectionTask */
/**
  * @brief  Function implementing the DirectionTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDirectionTask */

void StartDirectionTask(void const * argument) // Which way will the motor spin
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  switch (Movement)
  {
  // GPIOB controls the motor direction
  // GPIOA controls which Led is turned on
   case 0: // backwards
	   HAL_GPIO_WritePin(GPIOB, ForwardLED, 0);
	   HAL_GPIO_WritePin(GPIOB, ReverseLED, 1);
	   HAL_GPIO_WritePin(GPIOA, ForwardLED, 0);
	   HAL_GPIO_WritePin(GPIOA, ReverseLED, 1);
	   strcpy(Way,"Reverse");
   break;
   case 1: // forwards
	   HAL_GPIO_WritePin(GPIOB, ForwardLED, 1);
	   HAL_GPIO_WritePin(GPIOB, ReverseLED, 0);
	   HAL_GPIO_WritePin(GPIOA, ForwardLED, 1);
	   HAL_GPIO_WritePin(GPIOA, ReverseLED, 0);
	   strcpy(Way,"Forward");
   break;
   default:
   break;
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSpeedTask */
/**
* @brief Function implementing the SpeedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSpeedTask */
void StartSpeedTask(void const * argument)
{
  /* USER CODE BEGIN StartSpeedTask */
	switch(PowerLevel){
	case 0: // Stopping the motor
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3, 0);
     break;
    case 1: // Power level 1
    	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3, 400);
     break;
    case 2: // Power level 2
    	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3, 800);
	 break;
	case 3: // Power level 3
        __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3, 1200);
     break;
    case 4: // Power level 4
    	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3, 1600);
     break;
    case 5: // Power level 5
    	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3, 2000);
	 break;
	default:
     break;
	  }
  /* USER CODE END StartSpeedTask */
}

/* USER CODE BEGIN Header_StartSegmentTask */
/**
* @brief Function implementing the SegmentTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSegmentTask */
void StartSegmentTask(void const * argument)
/* Displaying the power level on the seven
 *segment display*/
{
	/* USER CODE BEGIN StartSegmentTask */
	HAL_GPIO_WritePin(GPIOA,a|b|c|d|e|f|g,0); // Reset the pins after each iteration
  switch(PowerLevel){
   case 0: // Power level 0
	HAL_GPIO_WritePin(GPIOA,a|b|c|d|e|f,1);
   break;
   case 1: // Power level 1
	HAL_GPIO_WritePin(GPIOA,b|c,1);
   break;
   case 2: // Power level 2
	HAL_GPIO_WritePin(GPIOA,a|b|d|e|g,1);
   break;
   case 3: // Power level 3
	HAL_GPIO_WritePin(GPIOA,a|b|c|d|g,1);
   break;
   case 4: // Power level 4
	HAL_GPIO_WritePin(GPIOA,b|c|f|g,1);
   break;
   case 5: // Power level 5
	HAL_GPIO_WritePin(GPIOA,a|c|d|f|g,1);
   break;
   default:
   break;
  }
  /* USER CODE END StartSegmentTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the DisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void const * argument) // Sending data to the terminal
{
  /* USER CODE BEGIN StartDisplayTask */
	UART_SEND(&huart2, "StartDisplayTask\r\n");
  /* Infinite loop */

  /* USER CODE END StartDisplayTask */
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	char Message[100];
	HAL_UART_Receive_IT(huart, &rx_data, 1);
	if(rx_data == '0')
	{
		PowerLevel = 0;
	}
	else if(rx_data == '1')
	{
		PowerLevel = 1;
	}
	else if(rx_data == '2')
	{
		PowerLevel = 2;
	}
	else if(rx_data == '3')
	{
		PowerLevel = 3;
	}
	else if(rx_data == '4')
	{
		PowerLevel = 4;
	}
	else if(rx_data == '5')
	{
		PowerLevel = 5;
	}
	else if(rx_data == '-')
	{
		Movement = 0;
	}
	else if(rx_data == '+')
	{
		Movement = 1;
	}
	StartDirectionTask(Movement);
	StartSpeedTask(PowerLevel);
	StartSegmentTask(PowerLevel);
	sprintf(Message, "Direction: %s\r\nPower Level: %d\r\n", Way, PowerLevel);
	HAL_UART_Transmit(&huart2, (uint8_t *) Message, strlen(Message), HAL_MAX_DELAY);
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
  if (htim->Instance == TIM1)
  {
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
