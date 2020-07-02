/*
 * utl_fdcan.c
 *
 *  Created on: Jun 30, 2020
 *      Author: S1ckret
 */

#include "utl_fdcan.h"
#include "utl_fdcan_definition.h"

static struct utl_fdcan_handle_t fdcan_modules[FDCAN_COUNT];

static uint8_t handle_count = 0;

static void utl_fdcan_error_handler(void);

void utl_fdcan_init(struct utl_fdcan_handle_t ** fdcan_module, FDCAN_GlobalTypeDef * Instance, FDCAN_InitTypeDef * Init) {
  /* Do not call init function more than @FDCAN_COUNT times.*/
  if (handle_count == FDCAN_COUNT) return;
  fdcan_modules[handle_count].handle.Instance = Instance;
  fdcan_modules[handle_count].handle.Init = *Init;

  fdcan_modules[handle_count].status_hal = HAL_FDCAN_Init(&fdcan_modules[handle_count].handle);

  *fdcan_module = &fdcan_modules[handle_count];
  ++handle_count;

  utl_fdcan_check_for_error_HAL(fdcan_module);
}

void utl_fdcan_activate_notification(struct utl_fdcan_handle_t * fdcan_module, uint32_t ActiveITs, uint32_t BufferIndexes) {
  fdcan_module->status_hal = HAL_FDCAN_ActivateNotification(&fdcan_module->handle, ActiveITs, BufferIndexes);
  utl_fdcan_check_for_error_HAL(fdcan_module);
}

void utl_fdcan_start(struct utl_fdcan_handle_t * fdcan_module) {
  fdcan_module->status_hal = HAL_FDCAN_Start(&fdcan_module->handle);
  utl_fdcan_check_for_error_HAL(fdcan_module);
}

void utl_fdcan_set_tx_header(struct utl_fdcan_handle_t * fdcan_module, FDCAN_TxHeaderTypeDef * tx_header) {
  fdcan_module->tx_header = *tx_header;
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
