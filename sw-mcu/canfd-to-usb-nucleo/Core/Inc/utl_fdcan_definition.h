/*
 * utl_fdcan_definition.h
 *
 *  Created on: Jun 30, 2020
 *      Author: S1ckret
 */

#ifndef INC_UTL_FDCAN_DEFINITION_H_
#define INC_UTL_FDCAN_DEFINITION_H_

struct utl_fdcan_handle_t {
  uint8_t payload[FDCAN_PAYLOAD];
  FDCAN_HandleTypeDef handle;
  FDCAN_TxHeaderTypeDef tx_header;
  HAL_StatusTypeDef status_hal;
  uint8_t id;
};

#endif /* INC_UTL_FDCAN_DEFINITION_H_ */
