#ifndef ENCODER_HANDLER_H
#define ENCODER_HANDLER_H

#include <stdint.h>

/**
 * @brief Inicializa el encoder y la ISR
 * @return 1 si éxito, 0 si error
 */
uint8_t init_encoder(void);

/**
 * @brief Obtiene el conteo actual del encoder
 * @return ticks del encoder
 */
int32_t encoder_get_ticks(void);

/**
 * @brief Reinicia el contador del encoder
 */
void encoder_reset_ticks(void);

#endif
