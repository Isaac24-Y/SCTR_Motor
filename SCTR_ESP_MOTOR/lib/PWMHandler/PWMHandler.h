#ifndef PWM_HANDLER_H
#define PWM_HANDLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>

uint8_t init_pwm_task(QueueHandle_t pid_output_q);

#endif