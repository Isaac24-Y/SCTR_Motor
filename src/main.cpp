#include <Arduino.h>

#include "EncoderHandler.h"
#include "VelocityHandler.h"
#include "PIDHandler.h"
#include "PWMHandler.h"
#include "CommsHandler.h"

QueueHandle_t velocity_queue;
QueueHandle_t setpoint_queue;
QueueHandle_t pid_output_queue;

void setup() {

    Serial.begin(115200);

    init_encoder();

    velocity_queue   = xQueueCreate(1, sizeof(float));
    setpoint_queue   = xQueueCreate(1, sizeof(float));
    pid_output_queue = xQueueCreate(1, sizeof(pid_output_t));

    init_velocity_task(velocity_queue);

    init_pid_task(
        velocity_queue,
        setpoint_queue,
        pid_output_queue
    );

    init_pwm_task(pid_output_queue);

    init_comms_task(
        setpoint_queue,
        pid_output_queue
    );
}

void loop() {
    vTaskDelete(NULL);
}
