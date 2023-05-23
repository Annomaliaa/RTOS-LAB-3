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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct means_t{
	float meanA;
	float squareA;
	float meanB;
	float squareB;
	float meanC;
	float squareC;
	uint8_t flagA;
	uint8_t flagB;
	uint8_t flagC;
}means_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskA */
osThreadId_t TaskAHandle;
const osThreadAttr_t TaskA_attributes = {
  .name = "TaskA",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for TaskB */
osThreadId_t TaskBHandle;
const osThreadAttr_t TaskB_attributes = {
  .name = "TaskB",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskC */
osThreadId_t TaskCHandle;
const osThreadAttr_t TaskC_attributes = {
  .name = "TaskC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for PrintTask */
osThreadId_t PrintTaskHandle;
const osThreadAttr_t PrintTask_attributes = {
  .name = "PrintTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Mutex */
osMutexId_t MutexHandle;
const osMutexAttr_t Mutex_attributes = {
  .name = "Mutex"
};
/* Definitions for Sem */
osSemaphoreId_t SemHandle;
const osSemaphoreAttr_t Sem_attributes = {
  .name = "Sem"
};
/* USER CODE BEGIN PV */
means_t means;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void StartTaskA(void *argument);
void StartTaskB(void *argument);
void StartTaskC(void *argument);
void StartPrintTask(void *argument);

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of Mutex */
  MutexHandle = osMutexNew(&Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of Sem */
  SemHandle = osSemaphoreNew(1, 1, &Sem_attributes);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of TaskA */
  TaskAHandle = osThreadNew(StartTaskA, NULL, &TaskA_attributes);

  /* creation of TaskB */
  TaskBHandle = osThreadNew(StartTaskB, NULL, &TaskB_attributes);

  /* creation of TaskC */
  TaskCHandle = osThreadNew(StartTaskC, NULL, &TaskC_attributes);

  /* creation of PrintTask */
  PrintTaskHandle = osThreadNew(StartPrintTask, NULL, &PrintTask_attributes);

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

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskA */
/**
* @brief Function implementing the TaskA thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskA */
void StartTaskA(void *argument)
{
  /* USER CODE BEGIN StartTaskA */
	float mean;
	float square;
	float random;
	srand(osKernelSysTick());
	means.flagA = 0;
  /* Infinite loop */
  for(;;)
  {
	  if(!means.flagA)
	  {
		  osSemaphoreAcquire(SemHandle, osWaitForever);
//		  osMutexAcquire(MutexHandle, osWaitForever);

		  mean = 0;
		  square = 0;

		  for ( uint16_t i = 0; i < 1000; i++)
		  {
			  random = (float)rand() / (float)RAND_MAX * 2 - 1;
			  mean += random;
			  square += random * random;
		  }

		  means.meanA = mean / 1000;
		  means.squareA = sqrt( square / 1000 );
		  means.flagA = 1;

		  osSemaphoreRelease(SemHandle);
//		  osMutexRelease(MutexHandle);
	  }
    osDelay(1);
  }
  /* USER CODE END StartTaskA */
}

/* USER CODE BEGIN Header_StartTaskB */
/**
* @brief Function implementing the TaskB thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskB */
void StartTaskB(void *argument)
{
  /* USER CODE BEGIN StartTaskB */
	float mean;
	float square;
	float random;
	srand(osKernelSysTick());
	means.flagB = 0;
  /* Infinite loop */
  for(;;)
  {
	  if(!means.flagB)
	  {
		  osSemaphoreAcquire(SemHandle, osWaitForever);
//		  osMutexAcquire(MutexHandle, osWaitForever);

		  mean = 0;
		  square = 0;

		  for ( uint16_t i = 0; i < 1000; i++)
		  {
			  random = (float)rand() / (float)RAND_MAX * 2 - 1;
			  mean += random;
			  square += random * random;
		  }

		  means.meanB = mean / 1000;
		  means.squareB = sqrt( square / 1000 );
		  means.flagB = 1;

		  osSemaphoreRelease(SemHandle);
//		  osMutexRelease(MutexHandle);
	  }
    osDelay(1);
  }
  /* USER CODE END StartTaskB */
}

/* USER CODE BEGIN Header_StartTaskC */
/**
* @brief Function implementing the TaskC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskC */
void StartTaskC(void *argument)
{
  /* USER CODE BEGIN StartTaskC */
	float mean;
	float square;
	float random;
	srand(osKernelSysTick());
	means.flagC = 0;
  /* Infinite loop */
  for(;;)
  {
	  if(!means.flagC)
	  {
		  osSemaphoreAcquire(SemHandle, osWaitForever);
//		  osMutexAcquire(MutexHandle, osWaitForever);

		  mean = 0;
		  square = 0;

		  for ( uint16_t i = 0; i < 1000; i++)
		  {
			  random = (float)rand() / (float)RAND_MAX * 2 - 1;
			  mean += random;
			  square += random * random;
		  }

		  means.meanC = mean / 1000;
		  means.squareC = sqrt( square / 1000 );
		  means.flagC = 1;

		  osSemaphoreRelease(SemHandle);
//		  osMutexRelease(MutexHandle);
	  }
    osDelay(1);
  }
  /* USER CODE END StartTaskC */
}

/* USER CODE BEGIN Header_StartPrintTask */
/**
* @brief Function implementing the PrintTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPrintTask */
void StartPrintTask(void *argument)
{
  /* USER CODE BEGIN StartPrintTask */
	float mean;
	float square;
	char message[64];
  /* Infinite loop */
  for(;;)
  {

	  if ( means.flagA && means.flagB && means.flagC )
	  {
		  osSemaphoreAcquire(SemHandle, osWaitForever);
//		  osMutexAcquire(MutexHandle, osWaitForever);

		  mean = means.meanA;
		  mean += means.meanB;
		  mean += means.meanC;
		  mean /= 3;

		  square = means.squareA * means.squareA;
		  square += means.squareB * means.squareB;
		  square += means.squareC * means.squareC;
		  square = sqrt( square / 3 );

		  sprintf ( message, "Mean of means = %f and Mean of square means = %f", mean, square);
		  HAL_UART_Transmit(&huart2, message, strlen(message), HAL_MAX_DELAY);
		  HAL_UART_Transmit(&huart2, "\n", strlen("\n"), HAL_MAX_DELAY);

		  //10ms delay for 64 characters
		  osDelay(640);

		  means.flagA = 0;
		  means.flagB = 0;
		  means.flagC = 0;

		  osSemaphoreRelease(SemHandle);
//		  osMutexRelease(MutexHandle);
	  }
	  //1000 - 640
    osDelay(360);
  }
  /* USER CODE END StartPrintTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
