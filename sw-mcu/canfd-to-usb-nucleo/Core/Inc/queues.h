/*
 * queues.h
 *
 *  Created on: 6 апр. 2020 г.
 *      Author: S1ckret
 */

#ifndef INC_QUEUES_H_
#define INC_QUEUES_H_

#define TICKS_TO_WAIT_FOR_RECEIVE pdMS_TO_TICKS(100)

#include "cmsis_os.h"

typedef struct Data_t {
  uint8_t uCanModuleNo;
  uint8_t payload[8];

} Data_t;

void createQueues();

#endif /* INC_QUEUES_H_ */
