/*
 * heartbeat_task.c
 *
 *  Created on: 1 июл. 2020 г.
 *      Author: S1ckret
 */

#include "heartbeat_task.h"

#include "stm32g4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#define PERIOD (pdMS_TO_TICKS(800))
#define DUTY (pdMS_TO_TICKS(200))

TaskHandle_t heartbeatTaskHandle;

static void StartHeartbeatTask(void *argument);


void run_heartbeat_task(void)
{
  if (xTaskCreate ((TaskFunction_t)StartHeartbeatTask, "heartbeatTask", 128U, NULL, 24U, &heartbeatTaskHandle) != pdPASS) {
    heartbeatTaskHandle = NULL;
  }
}

static void StartHeartbeatTask(void *argument)
{
  for(;;)
  {
    GPIOC->BSRR = GPIO_BSRR_BR_9;
    vTaskDelay(PERIOD - DUTY);
    GPIOC->BSRR = GPIO_BSRR_BS_9;
    vTaskDelay(DUTY);
  }
}
