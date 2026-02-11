#include "CommsHandler.h"
#include "PIDHandler.h"
#include <Arduino.h>

// ---------------- Config ----------------
#define COMMS_TASK_PERIOD_MS 100
#define COMMS_TASK_PRIORITY  1
#define COMMS_TASK_STACK     4096

// ---------------- Queues ----------------
static QueueHandle_t setpoint_queue = NULL;
static QueueHandle_t pid_output_queue = NULL;

// ---------------- Tarea ----------------
static void comms_task(void *param) {

    TickType_t last_wake = xTaskGetTickCount();
    float sp = 0.0f;
    pid_output_t out;

    for (;;) {

        vTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(COMMS_TASK_PERIOD_MS)
        );

        // ----- RX -----
        if (Serial.available()) {

            String cmd = Serial.readStringUntil('\n');
            cmd.trim();  // elimina \r y espacios

            // ---- Setpoint ----
            if (cmd.startsWith("SP:")) {

                float sp_rx = cmd.substring(3).toFloat();
                xQueueOverwrite(setpoint_queue, &sp_rx);
                sp = sp_rx; // mantener copia local para TX
            }

            // ---- PID ----
            else if (cmd.startsWith("PID:")) {

                float kc, ti, td;

                if (sscanf(cmd.c_str(), "PID:%f,%f,%f", &kc, &ti, &td) == 3) {
                    pid_params_t params = {
                        .kp = kc,
                        .ki = ti,
                        .kd = td
                    };
                    pid_update_params(params);
                }
            }
        }

        // ----- TX -----
        if (xQueueReceive(pid_output_queue, &out, 0)) {
            Serial.print("SP:");
            Serial.print(sp);
            Serial.print(",PV:");
            Serial.print(out.pv);
            Serial.print(",OP:");
            Serial.println(out.control);
        }
    }
}

// ---------------- Init ----------------
uint8_t init_comms_task(
    QueueHandle_t setpoint_q,
    QueueHandle_t pid_output_q
) {
    if (!setpoint_q || !pid_output_q)
        return 0;

    setpoint_queue = setpoint_q;
    pid_output_queue = pid_output_q;

    BaseType_t result = xTaskCreatePinnedToCore(
        comms_task,
        "CommsTask",
        COMMS_TASK_STACK,
        NULL,
        COMMS_TASK_PRIORITY,
        NULL,
        1 // Core 1
    );

    return (result == pdPASS);
}
