/*
 * utl_fdcan.h
 *
 *  Created on: Jun 30, 2020
 *      Author: S1ckret
 */

#ifndef INC_UTL_FDCAN_H_
#define INC_UTL_FDCAN_H_

#include "stm32g4xx_hal.h"

#define FDCAN_PAYLOAD 64
#define FDCAN_COUNT 3

struct utl_fdcan_handle_t;

/*
 * Inits @fdcan_module, namely sets address of @fdcan_module
 * */
void utl_fdcan_init(struct utl_fdcan_handle_t ** fdcan_module, FDCAN_GlobalTypeDef * Instance, FDCAN_InitTypeDef * Init);

void utl_fdcan_activate_notification(struct utl_fdcan_handle_t * fdcan_module, uint32_t ActiveITs, uint32_t BufferIndexes);

void utl_fdcan_start(struct utl_fdcan_handle_t * fdcan_module);

void utl_fdcan_set_tx_header(struct utl_fdcan_handle_t * fdcan_module, FDCAN_TxHeaderTypeDef * tx_header);

void utl_fdcan_check_for_error_HAL(struct utl_fdcan_handle_t * fdcan_module);

#endif /* INC_UTL_FDCAN_H_ */
