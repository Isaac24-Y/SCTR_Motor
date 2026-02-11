#include "PIDHandler.h"
#include <Arduino.h>

// ---------------- Configuración ----------------
#define PID_TASK_PERIOD_MS   10
#define PID_TASK_PRIORITY    2
#define PID_TASK_STACK       2048

// ---------------- Queues ----------------
static QueueHandle_t velocity_queue  = NULL;
static QueueHandle_t setpoint_queue  = NULL;
static QueueHandle_t output_queue    = NULL;

// ---------------- Parámetros PID ----------------
static pid_params_t pid = {1.0f, 0.0f, 0.0f};

// ---------------- Variables internas ----------------
static float integral = 0.0f;
static float prev_error = 0.0f;

// ---------------- Tarea PID ----------------
static void pid_task(void *parameter) {

    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t now;
    float pv = 0.0f;
    float sp = 0.0f;

    for (;;) {

        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(PID_TASK_PERIOD_MS)
        );

        now = xTaskGetTickCount();

        // Leer velocidad (no bloqueante)
        xQueueReceive(velocity_queue, &pv, 0);

        // Leer setpoint (no bloqueante)
        xQueueReceive(setpoint_queue, &sp, 0);

        float error = sp - pv;

        float dt = PID_TASK_PERIOD_MS / 1000.0f;

        integral += error * dt;
        float derivative = (error - prev_error) / dt;

        float output =
            pid.kp * error +
            pid.ki * integral +
            pid.kd * derivative;

        prev_error = error;

        pid_output_t out = {
            .control = output,
            .error   = error,
            .pv      = pv
        };

        xQueueOverwrite(output_queue, &out);
    }
}

// ---------------- Inicialización ----------------
uint8_t init_pid_task(
    QueueHandle_t velocity_q,
    QueueHandle_t setpoint_q,
    QueueHandle_t output_q
) {
    if (!velocity_q || !setpoint_q || !output_q)
        return 0;

    velocity_queue = velocity_q;
    setpoint_queue = setpoint_q;
    output_queue   = output_q;

    BaseType_t result = xTaskCreatePinnedToCore(
        pid_task,
        "PIDTask",
        PID_TASK_STACK,
        NULL,
        PID_TASK_PRIORITY,
        NULL,
        1 // Core 1
    );

    return (result == pdPASS);
}

// ---------------- API ----------------
void pid_update_params(pid_params_t params) {
    pid = params;
}
