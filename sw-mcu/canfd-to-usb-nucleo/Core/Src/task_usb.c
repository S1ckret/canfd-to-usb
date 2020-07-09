/*
 * task_usb.c
 *
 *  Created on: 9 июл. 2020 г.
 *      Author: S1ckret
 */

#include "task_usb.h"

#include "usbd_cdc_if.h"

#include "utl_fdcan.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

extern QueueHandle_t queue_usb;

TaskHandle_t usbTaskHandle;

static void StartUsbTask(void *argument);

void run_task_usb(void)
{
  if (xTaskCreate((TaskFunction_t)StartUsbTask, "fdcanTask", 256U, NULL, 22U, &usbTaskHandle) != pdPASS) {
    usbTaskHandle = NULL;
  }
}

static void StartUsbTask(void *argument)
{
  uint32_t bytes_count = 0;
  uint8_t data_received[FDCAN_PAYLOAD];
  for(;;)
  {
    xTaskNotifyWait(pdFALSE, pdTRUE, &bytes_count, portMAX_DELAY);

    while (uxQueueMessagesWaiting(queue_usb)) {
      if (xQueueReceive(queue_usb, data_received, 0) == pdTRUE) {
        CDC_Transmit_FS(data_received, bytes_count);
      }
    }
  }
}
