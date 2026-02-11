#ifndef VELOCITY_HANDLER_H
#define VELOCITY_HANDLER_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * @brief Inicializa la tarea de velocidad
 * @param queue queue donde se envía la velocidad
 * @return 1 si éxito, 0 si error
 */
uint8_t init_velocity_task(QueueHandle_t queue);

#endif
