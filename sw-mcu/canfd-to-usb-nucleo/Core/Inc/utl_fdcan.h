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

/* Id is 1 byte (uint8_t)  */
#if (FDCAN_COUNT > 0)
#define FDCAN_MODULE_1_ID '1'
#endif

#if (FDCAN_COUNT > 1)
#define FDCAN_MODULE_2_ID '2'
#endif

#if (FDCAN_COUNT > 2)
#define FDCAN_MODULE_3_ID '3'
#endif

struct utl_fdcan_handle_t;

/*
 * Inits @fdcan_module, namely sets address of @fdcan_module
 * */
void utl_fdcan_init(struct utl_fdcan_handle_t ** fdcan_module, FDCAN_GlobalTypeDef * Instance, FDCAN_InitTypeDef * Init);

void utl_fdcan_activate_notification(struct utl_fdcan_handle_t * fdcan_module, uint32_t ActiveITs, uint32_t BufferIndexes);

void utl_fdcan_start(struct utl_fdcan_handle_t * fdcan_module);

void utl_fdcan_set_tx_header(struct utl_fdcan_handle_t * fdcan_module, FDCAN_TxHeaderTypeDef * tx_header);

void utl_fdcan_send_payload(struct utl_fdcan_handle_t * fdcan_module);

uint8_t * const utl_fdcan_get_payload(struct utl_fdcan_handle_t * fdcan_module);

int8_t utl_fdcan_map_id_to_index(uint8_t id);

uint8_t utl_fdcan_map_index_to_id(uint8_t index);

void utl_fdcan_check_for_error_HAL(struct utl_fdcan_handle_t * fdcan_module);

#endif /* INC_UTL_FDCAN_H_ */
