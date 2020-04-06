/*
 * queues.c
 *
 *  Created on: 6 апр. 2020 г.
 *      Author: S1ckret
 */


#include "queues.h"
#include "queue.h"

#define FDCAN_COUNT 3
#define FDCAN_QUEUE_SIZE 3

QueueHandle_t queueToUSB;
QueueHandle_t queueToFDCAN1;

void createQueues() {
  queueToUSB = xQueueCreate(FDCAN_COUNT * FDCAN_QUEUE_SIZE, sizeof(Data_t));
  queueToFDCAN1 = xQueueCreate(FDCAN_QUEUE_SIZE, sizeof(Data_t));
}
