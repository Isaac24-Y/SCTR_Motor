#include "VelocityHandler.h"
#include "EncoderHandler.h"
#include <Arduino.h>

// ------------------- Configuración -------------------
#define VELOCITY_TASK_PERIOD_MS 5
#define VELOCITY_TASK_PRIORITY  3
#define VELOCITY_TASK_STACK     2048

// ------------------- Variables internas -------------------
static QueueHandle_t velocity_queue = NULL;

// ------------------- Tarea -------------------
static void velocity_task(void *parameter) {

    TickType_t last_wake_time = xTaskGetTickCount();
    int32_t last_ticks = 0;

    for (;;) {

        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(VELOCITY_TASK_PERIOD_MS)
        );

        int32_t current_ticks = encoder_get_ticks();
        int32_t delta_ticks   = current_ticks - last_ticks;
        last_ticks = current_ticks;

        // Velocidad cruda (ticks / Ts)
        float velocity = (float)delta_ticks;

        // Envío no bloqueante
        xQueueOverwrite(velocity_queue, &velocity);
    }
}

// ------------------- Inicialización -------------------
uint8_t init_velocity_task(QueueHandle_t queue) {

    if (queue == NULL) return 0;
    velocity_queue = queue;

    BaseType_t result = xTaskCreatePinnedToCore(
        velocity_task,
        "VelocityTask",
        VELOCITY_TASK_STACK,
        NULL,
        VELOCITY_TASK_PRIORITY,
        NULL,
        0 // Core 0
    );

    return (result == pdPASS);
}
