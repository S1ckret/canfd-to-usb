/*
 * utl_fdcan.c
 *
 *  Created on: Jun 30, 2020
 *      Author: S1ckret
 */

#include "utl_fdcan.h"

struct utl_fdcan_handle_t {
  uint8_t payload[FDCAN_PAYLOAD];
  FDCAN_HandleTypeDef handle;
  FDCAN_TxHeaderTypeDef tx_header;
  HAL_StatusTypeDef status_hal;
  uint8_t id;
};

static struct utl_fdcan_handle_t fdcan_modules[FDCAN_COUNT];

static uint8_t handle_count = 0;

static void utl_fdcan_error_handler(void);

void utl_fdcan_init(struct utl_fdcan_handle_t ** fdcan_module, FDCAN_GlobalTypeDef * Instance, FDCAN_InitTypeDef * Init) {
  fdcan_modules[handle_count].handle.Instance = Instance;
  fdcan_modules[handle_count].handle.Init = *Init;

  fdcan_modules[handle_count].status_hal = HAL_FDCAN_Init(&fdcan_modules[handle_count].handle);

  *fdcan_module = &fdcan_modules[handle_count];
  ++handle_count;

  utl_fdcan_check_for_error_HAL(fdcan_module);
}

void utl_fdcan_check_for_error_HAL(struct utl_fdcan_handle_t * fdcan_module) {
  if (fdcan_module->status_hal != HAL_OK)
  {
    utl_fdcan_error_handler();
  }
}

static void utl_fdcan_error_handler(void) {
  for (;;) {

  }
}
