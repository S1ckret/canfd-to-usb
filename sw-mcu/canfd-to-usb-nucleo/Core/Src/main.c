/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "heartbeatTask.h"
#include "uartTask.h"
#include "usbTask.h"

#include "utlFDCAN.h"
#include "utlFDCAN_Task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define FDCAN_COUNT 3
#define FDCAN_QUEUE_SIZE 3
#define FDCAN_TASK_STACK_SIZE 192
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef hlpuart1;


/* USER CODE BEGIN PV */
utlFDCAN_CanModule_t utlFDCAN1;
utlFDCAN_CanModule_t utlFDCAN2;
utlFDCAN_CanModule_t utlFDCAN3;

QueueHandle_t queueToUSB;
QueueHandle_t queueToUART;

QueueHandle_t queueToFDCAN1;
QueueHandle_t queueToFDCAN2;
QueueHandle_t queueToFDCAN3;

utlFDCAN_FDCAN_Queue_Bundle FDCAN_Queue_Bundle1;
utlFDCAN_FDCAN_Queue_Bundle FDCAN_Queue_Bundle2;
utlFDCAN_FDCAN_Queue_Bundle FDCAN_Queue_Bundle3;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
void StartHeartbeatTask(void *argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
static void Init_FDCAN(void);
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
  MX_LPUART1_UART_Init();
//  MX_USB_Device_Init();
  Init_FDCAN();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  /* Init scheduler */
  osKernelInitialize();

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
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  queueToUSB = xQueueCreate(FDCAN_COUNT * FDCAN_QUEUE_SIZE, sizeof(utlFDCAN_Data_t));
  queueToUART = xQueueCreate(FDCAN_COUNT * FDCAN_QUEUE_SIZE, sizeof(utlFDCAN_Data_t));
  queueToFDCAN1 = xQueueCreate(FDCAN_QUEUE_SIZE, sizeof(utlFDCAN_Data_t));
  queueToFDCAN2 = xQueueCreate(FDCAN_QUEUE_SIZE, sizeof(utlFDCAN_Data_t));
  queueToFDCAN3 = xQueueCreate(FDCAN_QUEUE_SIZE, sizeof(utlFDCAN_Data_t));

  createHeartbeatTask();
  createUartTask();

  FDCAN_Queue_Bundle1.FDCAN = &utlFDCAN1;
  FDCAN_Queue_Bundle1.Queue = queueToFDCAN1;

  osThreadAttr_t xFDCAN_attributes = {
    .name = "canfd1Task",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = FDCAN_TASK_STACK_SIZE
  };
  utlFDCAN_Task_Create(utlFDCAN_Task_Start, &FDCAN_Queue_Bundle1, &xFDCAN_attributes);

  FDCAN_Queue_Bundle2.FDCAN = &utlFDCAN2;
  FDCAN_Queue_Bundle2.Queue = queueToFDCAN2;

  xFDCAN_attributes.name = "canfd2Task";
  utlFDCAN_Task_Create(utlFDCAN_Task_Start, &FDCAN_Queue_Bundle2, &xFDCAN_attributes);

  FDCAN_Queue_Bundle3.FDCAN = &utlFDCAN3;
  FDCAN_Queue_Bundle3.Queue = queueToFDCAN3;

  xFDCAN_attributes.name = "canfd3Task";
  utlFDCAN_Task_Create(utlFDCAN_Task_Start, &FDCAN_Queue_Bundle3, &xFDCAN_attributes);
  /* USER CODE END RTOS_THREADS */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* FDCAN1_IT0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* FDCAN1_IT1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(FDCAN1_IT1_IRQn);
  /* FDCAN2_IT0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
  /* FDCAN2_IT1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(FDCAN2_IT1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(FDCAN2_IT1_IRQn);
  /* FDCAN3_IT0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(FDCAN3_IT0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(FDCAN3_IT0_IRQn);
  /* FDCAN3_IT1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(FDCAN3_IT1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(FDCAN3_IT1_IRQn);
  /* FDCAN3_IT1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(LPUART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(LPUART1_IRQn);
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STANDBY_2_Pin|STANDBY_1_Pin|LED_HEARTBEAT_Pin|STANDBY_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_STATUS1_GPIO_Port, LED_STATUS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_STATUS2_Pin|LED_STATUS3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STANDBY_2_Pin STANDBY_1_Pin LED_HEARTBEAT_Pin STANDBY_3_Pin */
  GPIO_InitStruct.Pin = STANDBY_2_Pin|STANDBY_1_Pin|LED_HEARTBEAT_Pin|STANDBY_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW3_Pin SW2_Pin SW1_Pin */
  GPIO_InitStruct.Pin = SW3_Pin|SW2_Pin|SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_STATUS1_Pin */
  GPIO_InitStruct.Pin = LED_STATUS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_STATUS1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_STATUS2_Pin LED_STATUS3_Pin */
  GPIO_InitStruct.Pin = LED_STATUS2_Pin|LED_STATUS3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void Init_FDCAN(void) {
  FDCAN_InitTypeDef Init;
  Init.ClockDivider = FDCAN_CLOCK_DIV1;
  Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
  Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
  Init.AutoRetransmission = DISABLE;
  Init.TransmitPause = DISABLE;
  Init.ProtocolException = DISABLE;
  Init.NominalPrescaler = 1;
  Init.NominalSyncJumpWidth = 1;
  Init.NominalTimeSeg1 = 2;
  Init.NominalTimeSeg2 = 2;
  Init.DataPrescaler = 1;
  Init.DataSyncJumpWidth = 1;
  Init.DataTimeSeg1 = 1;
  Init.DataTimeSeg2 = 1;
  Init.StdFiltersNbr = 0;
  Init.ExtFiltersNbr = 0;
  Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

  utlFDCAN_Init(&utlFDCAN1, FDCAN1, &Init);
  utlFDCAN_Init(&utlFDCAN2, FDCAN2, &Init);
  utlFDCAN_Init(&utlFDCAN3, FDCAN3, &Init);

  utlFDCAN_ActivateNotification(&utlFDCAN1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_TX_BUFFER0);
  utlFDCAN_ActivateNotification(&utlFDCAN2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_TX_BUFFER0);
  utlFDCAN_ActivateNotification(&utlFDCAN3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_TX_BUFFER0);

  utlFDCAN_Start(&utlFDCAN1);
  utlFDCAN_Start(&utlFDCAN2);
  utlFDCAN_Start(&utlFDCAN3);

  FDCAN_TxHeaderTypeDef TxHeader;
  TxHeader.Identifier = 0x529;
  TxHeader.IdType = FDCAN_EXTENDED_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_12;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_FD_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  utlFDCAN1.FDCAN_TxHeader = TxHeader;

  TxHeader.Identifier = 0x52A;

  utlFDCAN2.FDCAN_TxHeader = TxHeader;

  TxHeader.Identifier = 0x52B;

  utlFDCAN3.FDCAN_TxHeader = TxHeader;
}
/* USER CODE END 4 */


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
