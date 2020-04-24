/*
 * usbTask.c
 *
 *  Created on: Apr 5, 2020
 *      Author: S1ckret
 */


/*
 * usbTask.c
 *
 *  Created on: Mar 31, 2020
 *      Author: s1ckret
 */

#include "usbTask.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal.h"

#include "utlFDCAN.h"

#include "usbd_cdc_if.h"
#include "FreeRTOS.h"
#include "queue.h"

extern QueueHandle_t xQueueToUSB;

extern void Error_Handler(void);

void StartUsbTask(void *argument);

osThreadId_t * createUsbTask() {
  static osThreadId_t usbTaskHandle;

  const osThreadAttr_t usbTask_attributes = {
    .name = "usbTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 128
  };

  usbTaskHandle = osThreadNew(StartUsbTask, NULL, &usbTask_attributes);
  return &usbTaskHandle;
}


void StartUsbTask(void *argument)
{
  uint8_t usbStatus = HAL_ERROR;
  BaseType_t xStatus = pdFALSE;
  static utlFDCAN_Data_t data;
  for(;;)
  {
    if (uxQueueMessagesWaiting(xQueueToUSB)) {
      xStatus = xQueueReceive(xQueueToUSB, &data, TICKS_TO_WAIT_FOR_RECEIVE);
      if (xStatus == pdPASS) {
        /*Message from queue has been received.*/
        while (CDC_Transmit_FS(&data.FDCAN_ID, sizeof(utlFDCAN_Data_t)));
      }
      else {
        /*Can not receive message from queue.*/
      }
    }
  }
}
