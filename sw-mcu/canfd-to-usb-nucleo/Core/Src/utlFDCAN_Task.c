/*
 * utlFDCAN_Task.c
 *
 *  Created on: 15 апр. 2020 г.
 *      Author: S1ckret
 */
#include "utlFDCAN_Task.h"

extern void Error_Handler(void);

osThreadId_t utlFDCAN_Task_Create(osThreadFunc_t function, void * argument, osThreadAttr_t * attr) {
  return osThreadNew(function, argument, attr);
}

void utlFDCAN_Task_Start(void * argument) {
  utlFDCAN_FDCAN_Queue_Bundle * pxCAN_Queue = (utlFDCAN_FDCAN_Queue_Bundle *)argument;

  utlFDCAN_CanModule_t * pxFDCAN = pxCAN_Queue->FDCAN;
  QueueHandle_t xQueue = pxCAN_Queue->Queue;

  for(;;)
  {
    if (uxQueueMessagesWaiting(xQueue)) {
      pxFDCAN->LastStatus_Os = xQueueReceive(xQueue, &pxFDCAN->Data, TICKS_TO_WAIT_FOR_RECEIVE);
      if (pxFDCAN->LastStatus_Os == pdPASS) {
        /* Message from queue has been received. */
        /* Send message to the device. */
        pxFDCAN->LastStatus_HAL = HAL_FDCAN_AddMessageToTxFifoQ(&pxFDCAN->FDCAN_Handle, &pxFDCAN->FDCAN_TxHeader, (uint8_t*)&pxFDCAN->Data);
        utlFDCAN_Check_For_Error_HAL(pxFDCAN);
      }
      else {
        /* Can not receive message from queue.*/
      }
     }
  }
}

