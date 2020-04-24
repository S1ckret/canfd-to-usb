/*
 * utlFDCAN_Task.c
 *
 *  Created on: 15 апр. 2020 г.
 *      Author: S1ckret
 */
#include "utlFDCAN_Task.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal.h"

extern void Error_Handler(void);

#define LED_HEARTBEAT_Pin GPIO_PIN_9
#define LED_HEARTBEAT_GPIO_Port GPIOC

osThreadId_t utlFDCAN_Task_Create(osThreadFunc_t function, void * argument, osThreadAttr_t * attr) {
  return osThreadNew(function, argument, attr);
}

void utlFDCAN_Task_Start(void * argument) {
  utlFDCAN_FDCAN_Queue_Bundle * pxCAN_Queue = (utlFDCAN_FDCAN_Queue_Bundle *)argument;

  utlFDCAN_CanModule_t * pxFDCAN = pxCAN_Queue->FDCAN;
  pxFDCAN->Data.Payload[0] = 'H';
  pxFDCAN->Data.Payload[1] = 'e';
  pxFDCAN->Data.Payload[2] = 'l';
  pxFDCAN->Data.Payload[3] = 'l';
  pxFDCAN->Data.Payload[4] = 'o';
  pxFDCAN->Data.Payload[5] = ' ';
  pxFDCAN->Data.Payload[6] = '<';
  pxFDCAN->Data.Payload[7] = '3';

  for(;;)
  {
    HAL_GPIO_TogglePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin);
    HAL_FDCAN_AddMessageToTxFifoQ(&pxFDCAN->FDCAN_Handle, &pxFDCAN->FDCAN_TxHeader, (uint8_t*)&pxFDCAN->Data);
    osDelay(1000);
  }
}

