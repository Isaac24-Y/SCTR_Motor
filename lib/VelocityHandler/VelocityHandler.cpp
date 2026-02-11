#include "VelocityHandler.h"
#include "EncoderHandler.h"
#include <Arduino.h>
#include "esp_timer.h"

// ------------------- Configuración -------------------
#define VELOCITY_TASK_PERIOD_MS 5
#define VELOCITY_TASK_PRIORITY  3
#define VELOCITY_TASK_STACK     2048

// Encoder incremental: pulsos por revolución (ajustar al hardware real)
#define ENCODER_PPR 600.0f

// ------------------- Variables internas -------------------
static QueueHandle_t velocity_queue = NULL;

// ------------------- Tarea -------------------
static void velocity_task(void *parameter) {

    TickType_t last_wake_time = xTaskGetTickCount();
    int32_t last_ticks = 0;
    uint64_t last_exec_us = esp_timer_get_time();
    uint32_t max_jitter_us = 0;
    const uint32_t target_period_us = VELOCITY_TASK_PERIOD_MS * 1000U;

    for (;;) {

        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(VELOCITY_TASK_PERIOD_MS)
        );

        uint64_t now_us = esp_timer_get_time();
        uint32_t real_period_us = (uint32_t)(now_us - last_exec_us);
        last_exec_us = now_us;

        uint32_t jitter_us = (real_period_us > target_period_us)
            ? (real_period_us - target_period_us)
            : (target_period_us - real_period_us);
        if (jitter_us > max_jitter_us) {
            max_jitter_us = jitter_us;
        }

        int32_t current_ticks = encoder_get_ticks();
        int32_t delta_ticks   = current_ticks - last_ticks;
        last_ticks = current_ticks;

        float dt_s = real_period_us / 1000000.0f;
        float velocity_rps = (dt_s > 0.0f)
            ? (delta_ticks / ENCODER_PPR) / dt_s
            : 0.0f;

        velocity_sample_t sample = {
            .velocity = velocity_rps,
            .period_us = real_period_us,
            .jitter_us = max_jitter_us,
            .timestamp_us = now_us
        };

        xQueueOverwrite(velocity_queue, &sample);
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
