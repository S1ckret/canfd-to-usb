/*
 * fdCan1Task.c
 *
 *  Created on: 5 апр. 2020 г.
 *      Author: S1ckret
 */

#include "fdCan1Task.h"
#include "queues.h"
#include "queue.h"

#include "stm32g474xx.h"
#include "stm32g4xx_hal.h"

#define LED_Status1_Pin GPIO_PIN_7
#define LED_Status1_GPIO_Port GPIOC

extern QueueHandle_t queueToFDCAN1;
extern QueueHandle_t queueToUSB;
extern QueueHandle_t queueToUART;
extern FDCAN_HandleTypeDef hfdcan1;
extern void Error_Handler(void);

void StartFdCan1Task(void *argument);

osThreadId_t * createFdCan1Task() {
  static osThreadId_t canfd1TaskHandle;

  const osThreadAttr_t canfd1Task_attributes = {
    .name = "canfd1Task",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 192
  };

  canfd1TaskHandle = osThreadNew(StartFdCan1Task, NULL, &canfd1Task_attributes);
  return &canfd1TaskHandle;
}

void StartFdCan1Task(void *argument)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  uint8_t sendFromQueue = 0;
  BaseType_t xStatus = pdFALSE;
  Data_t data;

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

  for(;;)
  {
    if (uxQueueMessagesWaiting(queueToFDCAN1)) {
      xStatus = xQueueReceive(queueToFDCAN1, &data, TICKS_TO_WAIT_FOR_RECEIVE);
      if (xStatus == pdPASS) {
        /*Message from queue has been received.*/
        status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, (uint8_t*)&data);
        if (status != HAL_OK) {
          /*Can not send CAN frame.*/
          Error_Handler();
        }
      }
      else {
        /*Can not receive message from queue.*/
      }
     }
  }
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  static Data_t receivedData;
  static HAL_StatusTypeDef receivedStatus = HAL_ERROR;
  static BaseType_t xStatus = pdFALSE;
  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  static FDCAN_RxHeaderTypeDef RxHeader;

  receivedStatus = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, (uint8_t*)&receivedData);
  if (receivedStatus != HAL_OK) {
    Error_Handler();
  }

  xStatus = xQueueSendToBackFromISR(queueToUART, &receivedData, &xHigherPriorityTaskWoken);

  HAL_GPIO_TogglePin(LED_Status1_GPIO_Port, LED_Status1_Pin);

}
