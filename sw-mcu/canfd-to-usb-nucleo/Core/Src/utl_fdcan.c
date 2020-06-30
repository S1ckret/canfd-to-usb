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
