/*
 * uartTask.c
 *
 *  Created on: 8 апр. 2020 г.
 *      Author: S1ckret
 */


#include "uartTask.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_uart.h"

#include "utlFDCAN.h"

#include "FreeRTOS.h"
#include "queue.h"

extern QueueHandle_t queueToFDCAN1;
extern QueueHandle_t queueToUART;
extern UART_HandleTypeDef hlpuart1;
extern void Error_Handler(void);

void StartUartTask(void *argument);

osThreadId_t * createUartTask() {
  static osThreadId_t uartTaskHandle;

  const osThreadAttr_t uartTask_attributes = {
    .name = "uartTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 128
  };

  uartTaskHandle = osThreadNew(StartUartTask, NULL, &uartTask_attributes);
  return &uartTaskHandle;
}


static utlFDCAN_Data_t receivedData;
void StartUartTask(void *argument)
{
  uint8_t uartStatus = HAL_ERROR;
  BaseType_t xStatus = pdFALSE;
  utlFDCAN_Data_t data;
  HAL_UART_Receive_IT(&hlpuart1, (uint8_t*)&receivedData, sizeof(utlFDCAN_Data_t));
  for(;;)
  {
    if (uxQueueMessagesWaiting(queueToUART)) {
      xStatus = xQueueReceive(queueToUART, &data, TICKS_TO_WAIT_FOR_RECEIVE);
      if (xStatus == pdPASS) {
        /*Message from queue has been received.*/
        uartStatus = HAL_UART_Transmit_IT(&hlpuart1, (uint8_t*)&data, sizeof(utlFDCAN_Data_t));
        if (uartStatus != HAL_OK) {
          /*Can not send UART packet.*/
          Error_Handler();
        }
      }
      else {
        /*Can not receive message from queue.*/
      }
    }
  }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  static uint8_t answer[] = "UART_RX\r\n";
  static BaseType_t xStatus = pdFALSE;
  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  HAL_UART_Transmit_IT(&hlpuart1, answer, sizeof(utlFDCAN_Data_t));
  xStatus = xQueueSendToBackFromISR(queueToFDCAN1, &receivedData, &xHigherPriorityTaskWoken);

  HAL_UART_Receive_IT(&hlpuart1, (uint8_t*)&receivedData, sizeof(utlFDCAN_Data_t));
}
