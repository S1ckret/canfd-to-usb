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

  utl_fdcan_check_for_error_HAL(*fdcan_module);
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

void utl_fdcan_send_payload(struct utl_fdcan_handle_t * fdcan_module) {
  fdcan_module->status_hal =
      HAL_FDCAN_AddMessageToTxFifoQ(&fdcan_module->handle, &fdcan_module->tx_header, fdcan_module->payload);

  utl_fdcan_check_for_error_HAL(fdcan_module);
}

uint8_t * const utl_fdcan_get_payload(struct utl_fdcan_handle_t * fdcan_module) {
  return fdcan_module->payload;
}

int8_t utl_fdcan_map_id_to_index(uint8_t id) {
  int8_t index = -1;
  switch (id) {
#if (FDCAN_COUNT > 0)
  case FDCAN_MODULE_1_ID: index = 0;
    break;
#endif
#if (FDCAN_COUNT > 1)
  case FDCAN_MODULE_2_ID: index = 1;
    break;
#endif
#if (FDCAN_COUNT > 2)
  case FDCAN_MODULE_3_ID: index = 2;
    break;
#endif
  default: index = -1;
    break;
  }
  return index;
}

uint8_t utl_fdcan_map_index_to_id(uint8_t index) {
  uint8_t id = UINT8_MAX;
  switch (index) {
#if (FDCAN_COUNT > 0)
  case 0: id = FDCAN_MODULE_1_ID;
    break;
#endif
#if (FDCAN_COUNT > 1)
  case 1: id = FDCAN_MODULE_2_ID;
    break;
#endif
#if (FDCAN_COUNT > 2)
  case 2: id = FDCAN_MODULE_3_ID;
    break;
#endif
  default: id = UINT8_MAX;
    break;
  }
  return id;
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
