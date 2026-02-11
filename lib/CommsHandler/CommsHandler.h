#ifndef COMMS_HANDLER_H
#define COMMS_HANDLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

uint8_t init_comms_task(
    QueueHandle_t setpoint_q,
    QueueHandle_t pid_output_q
);

#endif
