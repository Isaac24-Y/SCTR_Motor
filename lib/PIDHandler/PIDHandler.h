#ifndef PID_HANDLER_H
#define PID_HANDLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>

// --------- Estructuras ---------
typedef struct {
    float kp;
    float ki;
    float kd;
} pid_params_t;

typedef struct {
    float setpoint;
} pid_setpoint_t;

typedef struct {
    float control;
    float error;
    float pv;
} pid_output_t;

// --------- API ---------
uint8_t init_pid_task(
    QueueHandle_t velocity_q,
    QueueHandle_t setpoint_q,
    QueueHandle_t output_q
);

void pid_update_params(pid_params_t params);

#endif
