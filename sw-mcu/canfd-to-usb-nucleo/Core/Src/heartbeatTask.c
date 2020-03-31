/*
 * heartbeatTask.c
 *
 *  Created on: Mar 31, 2020
 *      Author: s1ckret
 */

#include "heartbeatTask.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal.h"

#define LED_HEARTBEAT_Pin GPIO_PIN_11
#define LED_HEARTBEAT_GPIO_Port GPIOB

void StartHeartbeatTask(void *argument);

osThreadId_t * createHeartbeatTask() {
  static osThreadId_t heartbeatTaskHandle;

  const osThreadAttr_t heartbeatTask_attributes = {
    .name = "heartbeatTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 128
  };

  heartbeatTaskHandle = osThreadNew(StartHeartbeatTask, NULL, &heartbeatTask_attributes);
  return &heartbeatTaskHandle;
}

void StartHeartbeatTask(void *argument)
{
  for(;;)
  {
    HAL_GPIO_TogglePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin);
    osDelay(500);
  }
}
