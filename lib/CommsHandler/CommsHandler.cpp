#include "CommsHandler.h"
#include "PIDHandler.h"
#include <Arduino.h>
#include <cstdio>

// ---------------- Config ----------------
#define COMMS_TASK_PERIOD_MS 100
#define COMMS_TASK_PRIORITY  1
#define COMMS_TASK_STACK     4096

// ---------------- Queues ----------------
static QueueHandle_t setpoint_queue = NULL;
static QueueHandle_t telemetry_queue = NULL;

// ---------------- Tarea ----------------
static void comms_task(void *param) {

    TickType_t last_wake = xTaskGetTickCount();
    float sp = 0.0f;
    telemetry_t telemetry = {0};

    for (;;) {

        vTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(COMMS_TASK_PERIOD_MS)
        );

        // ----- RX -----
        if (Serial.available()) {

            String cmd = Serial.readStringUntil('\n');
            cmd.trim();

            // ---- Setpoint ----
            if (cmd.startsWith("SP:")) {

                float sp_rx = cmd.substring(3).toFloat();
                xQueueOverwrite(setpoint_queue, &sp_rx);
                sp = sp_rx;
            }

            // ---- PID ----
            else if (cmd.startsWith("PID:")) {

                float kp, ki, kd;

                if (std::sscanf(cmd.c_str(), "PID:%f,%f,%f", &kp, &ki, &kd) == 3) {
                    pid_params_t params = {
                        .kp = kp,
                        .ki = ki,
                        .kd = kd
                    };
                    pid_update_params(params);
                }
            }
        }

        // ----- TX -----
        if (xQueueReceive(telemetry_queue, &telemetry, 0) == pdTRUE) {
            sp = telemetry.sp;
        }

        Serial.print("SP:");
        Serial.print(sp, 3);
        Serial.print(",PV:");
        Serial.print(telemetry.pv, 3);
        Serial.print(",OP:");
        Serial.print(telemetry.op, 3);
        Serial.print(",ERR:");
        Serial.print(telemetry.error, 3);
        Serial.print(",Tvel_us:");
        Serial.print(telemetry.velocity_period_us);
        Serial.print(",JvelMax_us:");
        Serial.print(telemetry.velocity_jitter_us);
        Serial.print(",Tpid_us:");
        Serial.print(telemetry.pid_period_us);
        Serial.print(",JpidMax_us:");
        Serial.print(telemetry.pid_jitter_us);
        Serial.print(",Lpid_us:");
        Serial.print(telemetry.pid_latency_us);
        Serial.print(",Lpwm_us:");
        Serial.print(telemetry.control_to_pwm_latency_us);
        Serial.print(",Ltotal_us:");
        Serial.println(telemetry.sensor_to_pwm_latency_us);
    }
}

// ---------------- Init ----------------
uint8_t init_comms_task(
    QueueHandle_t setpoint_q,
    QueueHandle_t telemetry_q
) {
    if (!setpoint_q || !telemetry_q)
        return 0;

    setpoint_queue = setpoint_q;
    telemetry_queue = telemetry_q;

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
