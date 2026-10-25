#ifndef ALTITUDE_KALMAN_HELPERS_H
#define ALTITUDE_KALMAN_HELPERS_H

#include <stdbool.h>

#include "altitude_kalman.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Parametros usados al adaptar el filtro entre reposo y movimiento.
 *
 * Los valores permanecen fuera del nucleo matematico del Kalman porque
 * dependen del sensor, el montaje y la caracterizacion de cada aplicacion.
 */
typedef struct
{
    float stationary_acceleration_noise_std;
    float moving_acceleration_noise_std;
    float barometer_noise_std;
    float stationary_nis_threshold;
    float moving_nis_threshold;
} altitude_kalman_motion_config_t;

/**
 * @brief Ajusta ruido de aceleracion y compuerta NIS segun el movimiento.
 *
 * En reposo se habilita la compuerta NIS. En movimiento se deshabilita para
 * permitir que el barometro vuelva a anclar una prediccion inercial separada.
 */
esp_err_t altitude_kalman_configure_motion_state(
    altitude_kalman_t *filter,
    const altitude_kalman_motion_config_t *config,
    bool stationary);

/**
 * @brief Acepta una muestra barometrica deshabilitando temporalmente la NIS.
 *
 * Conserva el umbral configurado y deja la compuerta habilitada al terminar,
 * incluso si la correccion barometrica falla.
 */
esp_err_t altitude_kalman_progressive_baro_recovery(
    altitude_kalman_t *filter,
    float barometric_height_m);

#ifdef __cplusplus
}
#endif

#endif
