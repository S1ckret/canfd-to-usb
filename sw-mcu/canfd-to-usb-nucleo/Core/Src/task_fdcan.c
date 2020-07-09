/*
 * task_fdcan.c
 *
 *  Created on: 2 июл. 2020 г.
 *      Author: S1ckret
 */


#include "task_fdcan.h"
#include "utl_fdcan.h"
#include "utl_fdcan_definition.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

extern QueueHandle_t queue_fdcan[FDCAN_COUNT];
extern struct utl_fdcan_handle_t * hfdcan[FDCAN_COUNT];

TaskHandle_t fdcanTaskHandle;

static void StartFdcanTask(void *argument);

void run_task_fdcan(void)
{
  if (xTaskCreate ((TaskFunction_t)StartFdcanTask, "fdcanTask", 128U, NULL, 22U, &fdcanTaskHandle) != pdPASS) {
    fdcanTaskHandle = NULL;
  }
}

static void StartFdcanTask(void *argument)
{
  uint32_t i = 0;
  for(;;)
  {
    /* Reset value, wait for ever. */
    xTaskNotifyWait(pdFALSE, pdTRUE, &i, portMAX_DELAY);

    while (uxQueueMessagesWaiting(queue_fdcan[i])) {
      if (xQueueReceive(queue_fdcan[i], utl_fdcan_get_payload(hfdcan[i]), 0) == pdTRUE) {
        utl_fdcan_send_payload(hfdcan[i]);
      }
    }
  }
}
