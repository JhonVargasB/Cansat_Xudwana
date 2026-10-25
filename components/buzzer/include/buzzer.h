#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "shared.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BUZZER_MAX_FREQUENCY_HZ UINT32_C(1024)

typedef struct
{
    gpio_num_t pin;
} buzzer_config_t;

#define BUZZER_DEFAULT_CONFIG() { .pin = PIN_BUZZER }

/* Inicializar con {0}. Mantener el objeto vivo hasta buzzer_deinit().
 * Las llamadas de control son desde tarea; serializarlas si hay varios usuarios.
 * El temporizador genera dos interrupciones por periodo (maximo 2048/s).
 */
typedef struct
{
    buzzer_config_t config;
    gptimer_handle_t timer;
    portMUX_TYPE lock;
    uint32_t frequency_hz;
    uint32_t level;
    bool active;
    bool running;
    bool enabled;
    bool initialized;
} buzzer_t;

/** Configura GPIO y temporizador; comienza apagado, con salida en LOW. */
esp_err_t buzzer_init(buzzer_t *device, const buzzer_config_t *config);

/** 0: apagar. 1..1024: frecuencia en Hz, onda cuadrada al 50 %.
 * Frecuencia aproximada por redondeo a microsegundos y latencia de interrupcion.
 * Para un buzzer pasivo; un buzzer activo no permite elegir el tono asi.
 */
esp_err_t buzzer_set_frequency(buzzer_t *device, uint32_t frequency_hz);
esp_err_t buzzer_off(buzzer_t *device);
esp_err_t buzzer_deinit(buzzer_t *device);

#ifdef __cplusplus
}
#endif
