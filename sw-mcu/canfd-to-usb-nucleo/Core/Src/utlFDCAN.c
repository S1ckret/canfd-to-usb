/*
 * utlFDCAN.c
 *
 *  Created on: 14 апр. 2020 г.
 *      Author: S1ckret
 */

#include "utlFDCAN.h"
#include "projdefs.h"

void utlFDCAN_Init(utlFDCAN_CanModule_t * FDCAN_Module, FDCAN_GlobalTypeDef * Instance, FDCAN_InitTypeDef * Init) {
  FDCAN_Module->FDCAN_Handle.Instance = Instance;
  FDCAN_Module->FDCAN_Handle.Init = *Init;
  FDCAN_Module->LastStatus_HAL = HAL_FDCAN_Init(&FDCAN_Module->FDCAN_Handle);
  utlFDCAN_Check_For_Error_HAL(FDCAN_Module);
}

void utlFDCAN_ActivateNotification(utlFDCAN_CanModule_t * FDCAN_Module, uint32_t ActiveITs, uint32_t BufferIndexes) {
  FDCAN_Module->LastStatus_HAL = HAL_FDCAN_ActivateNotification(&FDCAN_Module->FDCAN_Handle, ActiveITs, BufferIndexes);
  utlFDCAN_Check_For_Error_HAL(FDCAN_Module);
}

void utlFDCAN_Start(utlFDCAN_CanModule_t * FDCAN_Module) {
  FDCAN_Module->LastStatus_HAL = HAL_FDCAN_Start(&FDCAN_Module->FDCAN_Handle);
  utlFDCAN_Check_For_Error_HAL(FDCAN_Module);

}

void utlFDCAN_Error_Handler(void) {
  for (;;) {

  }
}

void utlFDCAN_Check_For_Error_HAL(utlFDCAN_CanModule_t * FDCAN_Module) {
  if (FDCAN_Module->LastStatus_HAL != HAL_OK)
  {
    utlFDCAN_Error_Handler();
  }
}


void utlFDCAN_Check_For_Error_Os(utlFDCAN_CanModule_t * FDCAN_Module) {
  if (FDCAN_Module->LastStatus_Os != pdTRUE)
  {
    utlFDCAN_Error_Handler();
  }
}

