/*
 * utlFDCAN_Callbacks.c
 *
 *  Created on: Apr 17, 2020
 *      Author: S1ckret
 */

#include "utlFDCAN.h"

#include "FreeRTOS.h"
#include "queue.h"

#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_1
#define LED3_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_0
#define LED4_GPIO_Port GPIOB

extern QueueHandle_t xQueueToUSB;
extern QueueHandle_t xQueueToUART;

extern utlFDCAN_CanModule_t utlFDCAN1;
extern utlFDCAN_CanModule_t utlFDCAN2;
extern utlFDCAN_CanModule_t utlFDCAN3;

extern void Error_Handler(void);

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  static utlFDCAN_Data_t Data;
  static FDCAN_RxHeaderTypeDef RxHeader;
  static HAL_StatusTypeDef LastStatus_HAL = HAL_ERROR;
  static BaseType_t xLastStatus_Os = pdFALSE;
  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  /*
  FDCAN_IT_RX_FIFO0_MESSAGE_LOST
  FDCAN_IT_RX_FIFO0_FULL
  FDCAN_IT_RX_FIFO0_NEW_MESSAGE
  */
  LastStatus_HAL = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, (uint8_t*)&Data);
  if (LastStatus_HAL != HAL_OK) {
    Error_Handler();
  }

  xLastStatus_Os = xQueueSendToBackFromISR(xQueueToUART, &Data, &xHigherPriorityTaskWoken);
  if (xLastStatus_Os != pdPASS) {
    Error_Handler();
  }

  // TODO: Give semaphore to the led status task ???
  if (hfdcan == &utlFDCAN1.FDCAN_Handle) {
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
  }
  else if (hfdcan == &utlFDCAN2.FDCAN_Handle) {
    HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
  }
  else if (hfdcan == &utlFDCAN3.FDCAN_Handle) {
    HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
  }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan) {

}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {

}
