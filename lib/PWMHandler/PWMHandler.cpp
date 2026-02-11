#include "PWMHandler.h"
#include "PIDHandler.h"
#include <Arduino.h>
#include "esp_timer.h"

// ---------------- Pines ----------------
#define PWM_PIN        22
#define DIR_PIN        23

// ---------------- Configuración ----------------
#define PWM_FREQ       20000
#define PWM_CHANNEL    0
#define PWM_RES        8
#define PWM_MAX        255
#define PWM_TASK_PERIOD_MS 10
#define PWM_TASK_PRIORITY  2
#define PWM_TASK_STACK     3072

// ---------------- Queues ----------------
static QueueHandle_t pid_output_queue = NULL;
static QueueHandle_t telemetry_queue = NULL;

// ---------------- Tarea PWM ----------------
static void pwm_task(void *param) {

    TickType_t last_wake_time = xTaskGetTickCount();
    pid_output_t cmd = {0};
    bool has_cmd = false;

    for (;;) {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(PWM_TASK_PERIOD_MS));

        // Toma el comando más reciente si existe
        if (xQueueReceive(pid_output_queue, &cmd, 0) == pdTRUE) {
            has_cmd = true;
        }

        if (!has_cmd) {
            continue;
        }

        float u = cmd.control;

        // ---- Dirección ----
        if (u >= 0.0f) {
            digitalWrite(DIR_PIN, HIGH);
        } else {
            digitalWrite(DIR_PIN, LOW);
            u = -u;
        }

        // ---- Saturación ----
        if (u > PWM_MAX) {
            u = PWM_MAX;
        }

        ledcWrite(PWM_CHANNEL, (uint32_t)u);

        uint64_t apply_us = esp_timer_get_time();
        telemetry_t telemetry = {
            .sp = cmd.sp,
            .pv = cmd.pv,
            .op = cmd.control,
            .error = cmd.error,
            .velocity_period_us = cmd.velocity_period_us,
            .velocity_jitter_us = cmd.velocity_jitter_us,
            .pid_period_us = cmd.pid_period_us,
            .pid_jitter_us = cmd.pid_jitter_us,
            .pid_latency_us = cmd.pid_latency_us,
            .control_to_pwm_latency_us = (uint32_t)(apply_us - cmd.control_timestamp_us),
            .sensor_to_pwm_latency_us = (uint32_t)(apply_us - cmd.sample_timestamp_us)
        };

        xQueueOverwrite(telemetry_queue, &telemetry);
    }
}

// ---------------- Init ----------------
uint8_t init_pwm_task(QueueHandle_t pid_output_q, QueueHandle_t telemetry_q) {

    if (!pid_output_q || !telemetry_q) {
        return 0;
    }

    pid_output_queue = pid_output_q;
    telemetry_queue = telemetry_q;

    pinMode(DIR_PIN, OUTPUT);

    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
    ledcAttachPin(PWM_PIN, PWM_CHANNEL);

    BaseType_t result = xTaskCreatePinnedToCore(
        pwm_task,
        "PWMTask",
        PWM_TASK_STACK,
        NULL,
        PWM_TASK_PRIORITY,
        NULL,
        1 // Core 1
    );

    return (result == pdPASS);
}
