#include "PWMHandler.h"
#include "PIDHandler.h"
#include <Arduino.h>

// ---------------- Pines ----------------
#define PWM_PIN        22
#define DIR_PIN        23

// ---------------- Configuración ----------------
#define PWM_FREQ       20000
#define PWM_CHANNEL    0
#define PWM_RES        8
#define PWM_MAX        255

#define PWM_TASK_PRIORITY  2
#define PWM_TASK_STACK     2048

// ---------------- Queue ----------------
static QueueHandle_t pid_output_queue = NULL;

// ---------------- Tarea PWM ----------------
static void pwm_task(void *param) {

    pid_output_t pid_out;

    for (;;) {

        if (xQueueReceive(pid_output_queue, &pid_out, portMAX_DELAY)) {

            float u = pid_out.control;

            // ---- Dirección ----
            if (u >= 0) {
                digitalWrite(DIR_PIN, HIGH);
            } else {
                digitalWrite(DIR_PIN, LOW);
                u = -u;
            }

            // ---- Saturación ----
            if (u > PWM_MAX) u = PWM_MAX;

            ledcWrite(PWM_CHANNEL, (uint32_t)u);
        }
    }
}

// ---------------- Init ----------------
uint8_t init_pwm_task(QueueHandle_t pid_output_q) {

    if (!pid_output_q) return 0;

    pid_output_queue = pid_output_q;

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
