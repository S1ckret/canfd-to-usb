/*
 * utlFDCAN_Task.h
 *
 *  Created on: 15 апр. 2020 г.
 *      Author: S1ckret
 */

#ifndef INC_UTLFDCAN_TASK_H_
#define INC_UTLFDCAN_TASK_H_

#include "utlFDCAN.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os2.h"

typedef struct {
  utlFDCAN_CanModule_t * FDCAN;
  QueueHandle_t Queue;
} utlFDCAN_FDCAN_Queue_Bundle;

osThreadId_t utlFDCAN_Task_Create(osThreadFunc_t function, void * argument, osThreadAttr_t * attr);

void utlFDCAN_Task_Start(void * argument);

#endif /* INC_UTLFDCAN_TASK_H_ */
