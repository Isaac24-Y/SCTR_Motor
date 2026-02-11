#include <Arduino.h>

#include "EncoderHandler.h"
#include "VelocityHandler.h"
#include "PIDHandler.h"
#include "PWMHandler.h"
#include "CommsHandler.h"

QueueHandle_t velocity_queue;
QueueHandle_t setpoint_queue;
QueueHandle_t pid_output_queue;
QueueHandle_t telemetry_queue;

void setup() {

    Serial.begin(115200);

    init_encoder();

    velocity_queue   = xQueueCreate(1, sizeof(velocity_sample_t));
    setpoint_queue   = xQueueCreate(1, sizeof(float));
    pid_output_queue = xQueueCreate(1, sizeof(pid_output_t));
    telemetry_queue  = xQueueCreate(1, sizeof(telemetry_t));

    init_velocity_task(velocity_queue);

    init_pid_task(
        velocity_queue,
        setpoint_queue,
        pid_output_queue
    );

    init_pwm_task(pid_output_queue, telemetry_queue);

    init_comms_task(
        setpoint_queue,
        telemetry_queue
    );
}

void loop() {
    vTaskDelete(NULL);
}
