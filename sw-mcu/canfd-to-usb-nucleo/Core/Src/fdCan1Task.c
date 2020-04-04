/*
 * fdCan1Task.c
 *
 *  Created on: 5 апр. 2020 г.
 *      Author: S1ckret
 */

#include "fdCan1Task.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal.h"

#define LED_Status1_Pin GPIO_PIN_7
#define LED_Status1_GPIO_Port GPIOC

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
  uint8_t data[1] = { 45 };

  FDCAN_TxHeaderTypeDef TxHeader;
  TxHeader.Identifier = 0x529;
  TxHeader.IdType = FDCAN_EXTENDED_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_1;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_FD_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  for(;;)
  {
    /* Send message every second */
    status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data);
    if (status != HAL_OK) {
      Error_Handler();
    }

    if (data[0] == 0) {
      data[0] = 1;
    } else {
      data[0] = 0;
    }

    osDelay(1000);
  }
}

static uint8_t receivedData[1] = { 0 };
static HAL_StatusTypeDef receivedStatus = HAL_ERROR;
static FDCAN_RxHeaderTypeDef RxHeader;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {

  receivedStatus = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, receivedData);

  if (receivedStatus != HAL_OK) {
    Error_Handler();
  }
  if (receivedData[0] == 0) {
    HAL_GPIO_WritePin(LED_Status1_GPIO_Port, LED_Status1_Pin, GPIO_PIN_RESET);
  }
  else if (receivedData[0] == 1) {
    HAL_GPIO_WritePin(LED_Status1_GPIO_Port, LED_Status1_Pin, GPIO_PIN_SET);
  }
}
