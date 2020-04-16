/*
 * utlFDCAN.h
 *
 *  Created on: 14 апр. 2020 г.
 *      Author: S1ckret
 */

#ifndef INC_UTLFDCAN_H_
#define INC_UTLFDCAN_H_

#include "FreeRTOS.h"
#include "stm32g4xx_hal.h"

#define TICKS_TO_WAIT_FOR_RECEIVE pdMS_TO_TICKS(100)

typedef struct {
  uint8_t payload[8];
} utlFDCAN_Data_t;

typedef struct {
  FDCAN_HandleTypeDef FDCAN_Handle;
  FDCAN_TxHeaderTypeDef FDCAN_TxHeader;
  HAL_StatusTypeDef LastStatus_HAL;
  BaseType_t LastStatus_Os;
  utlFDCAN_Data_t Data;
} utlFDCAN_CanModule_t;

void utlFDCAN_Init(utlFDCAN_CanModule_t * FDCAN_Module, FDCAN_GlobalTypeDef * Instance, FDCAN_InitTypeDef * Init);

void utlFDCAN_ActivateNotification(utlFDCAN_CanModule_t * FDCAN_Module, uint32_t ActiveITs, uint32_t BufferIndexes);

void utlFDCAN_Start(utlFDCAN_CanModule_t * FDCAN_Module);

void utlFDCAN_Check_For_Error_HAL(utlFDCAN_CanModule_t * FDCAN_Module);

void utlFDCAN_Check_For_Error_Os(utlFDCAN_CanModule_t * FDCAN_Module);

void utlFDCAN_Error_Handler(void);


#endif /* INC_UTLFDCAN_H_ */
