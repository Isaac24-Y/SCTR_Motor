#include "EncoderHandler.h"
#include <Arduino.h>

// ------------------- Pines -------------------
#define ENCODER_PIN_A 34
#define ENCODER_PIN_B 35

// ------------------- Variables internas -------------------
// volatile: cambia en ISR
// static: solo visible en este archivo
static volatile int32_t encoder_ticks = 0;

// ------------------- ISR -------------------
static void IRAM_ATTR encoder_isr(void) {
    encoder_ticks++;
}

// ------------------- Inicialización -------------------
uint8_t init_encoder(void) {

    pinMode(ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER_PIN_B, INPUT_PULLUP);

    attachInterrupt(
        digitalPinToInterrupt(ENCODER_PIN_A),
        encoder_isr,
        RISING
    );

    encoder_ticks = 0;
    return 1;
}

// ------------------- API pública -------------------
int32_t encoder_get_ticks(void) {
    return encoder_ticks;
}

void encoder_reset_ticks(void) {
    encoder_ticks = 0;
}
