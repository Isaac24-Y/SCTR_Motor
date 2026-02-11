#ifndef VELOCITY_HANDLER_H
#define VELOCITY_HANDLER_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct {
    float velocity;
    uint32_t period_us;
    uint32_t jitter_us;
    uint64_t timestamp_us;
} velocity_sample_t;

/**
 * @brief Inicializa la tarea de velocidad
 * @param queue queue donde se envía la velocidad
 * @return 1 si éxito, 0 si error
 */
uint8_t init_velocity_task(QueueHandle_t queue);

#endif
