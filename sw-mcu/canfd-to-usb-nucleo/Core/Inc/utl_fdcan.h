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

#endif /* INC_UTL_FDCAN_H_ */
