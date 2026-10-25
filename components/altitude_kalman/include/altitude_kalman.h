#ifndef ALTITUDE_KALMAN_H
#define ALTITUDE_KALMAN_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Medicion descartada por superar la compuerta de innovacion normalizada. */
#define ALTITUDE_KALMAN_ERR_OUTLIER ESP_ERR_INVALID_RESPONSE

/** Limite por defecto: equivale aproximadamente a una compuerta de 3 sigma. */
#define ALTITUDE_KALMAN_DEFAULT_NIS_THRESHOLD 9.0f
#define ALTITUDE_KALMAN_STATE_DIM 2

typedef enum
{
    ALTITUDE_KALMAN_HEIGHT = 0,
    ALTITUDE_KALMAN_VERTICAL_VELOCITY = 1,
} altitude_kalman_state_index_t;

/** Parametros persistentes usados para inicializar y ajustar el filtro. */
typedef struct
{
    float initial_height_m;
    float initial_velocity_mps;
    float initial_height_variance;
    float initial_velocity_variance;
    float acceleration_noise_std;
    float barometer_noise_std;
    float nis_threshold;
    bool nis_gate_enabled;
} altitude_kalman_config_t;

#define ALTITUDE_KALMAN_DEFAULT_CONFIG()                    \
    {                                                       \
        .initial_height_m = 0.0f,                           \
        .initial_velocity_mps = 0.0f,                       \
        .initial_height_variance = 1.0f,                    \
        .initial_velocity_variance = 1.0f,                  \
        .acceleration_noise_std = 0.10f,                    \
        .barometer_noise_std = 1.0f,                        \
        .nis_threshold = ALTITUDE_KALMAN_DEFAULT_NIS_THRESHOLD, \
        .nis_gate_enabled = true,                           \
    }

/** Estado x = [altura, velocidad vertical], positivo hacia arriba. */
typedef struct
{
    float height_m;
    float vertical_velocity_m_s;
} altitude_kalman_state_t;

/** Matrices del modelo: P y Q usan orden [altura, velocidad]. */
typedef struct
{
    float covariance_P[ALTITUDE_KALMAN_STATE_DIM]
                      [ALTITUDE_KALMAN_STATE_DIM];
    float process_noise_Q[ALTITUDE_KALMAN_STATE_DIM]
                         [ALTITUDE_KALMAN_STATE_DIM];
    float barometer_variance_R;
} altitude_kalman_matrices_t;

/** Valores de la ultima correccion, utiles para supervision y telemetria. */
typedef struct
{
    float last_innovation;
    float last_innovation_variance;
    float last_nis;
    float last_gain[ALTITUDE_KALMAN_STATE_DIM];

    int64_t last_predict_us;
    bool last_measurement_accepted;
} altitude_kalman_diagnostics_t;

/**
 * @brief Instancia completa del filtro lineal de altura.
 *
 * La aceleracion de entrada debe estar en el marco global, en m/s^2,
 * vertical positiva hacia arriba y con la gravedad retirada.
 */
typedef struct
{
    altitude_kalman_config_t config;
    altitude_kalman_state_t state;
    altitude_kalman_matrices_t matrices;
    altitude_kalman_diagnostics_t diagnostics;
    bool initialized;
} altitude_kalman_t;

/**
 * @brief Inicializa el filtro y sus covarianzas.
 * @param config Estado inicial, covarianzas, ruidos y compuerta NIS.
 */
esp_err_t altitude_kalman_init(altitude_kalman_t *kf,
                               const altitude_kalman_config_t *config);

/** @brief Propaga estado y covarianza con una muestra nueva de aceleracion. */
esp_err_t altitude_kalman_predict(altitude_kalman_t *kf,
                                  float vertical_acceleration_mps2,
                                  float dt_s);

/**
 * @brief Corrige el estado con una muestra barometrica nueva.
 * @return ESP_OK si se acepta, ALTITUDE_KALMAN_ERR_OUTLIER si la compuerta NIS
 *         la rechaza, u otro esp_err_t si el argumento/estado no es valido.
 */
esp_err_t altitude_kalman_correct_baro(altitude_kalman_t *kf,
                                       float barometric_height_m);

/** @brief Cambia las desviaciones estandar de los ruidos (no sus varianzas). */
esp_err_t altitude_kalman_set_noise(altitude_kalman_t *kf,
                                    float acceleration_noise_std,
                                    float barometer_noise_std);

/** @brief Habilita/deshabilita rechazo NIS y configura su umbral positivo. */
esp_err_t altitude_kalman_set_nis_gate(altitude_kalman_t *kf,
                                       bool enabled,
                                       float threshold);

/**
 * @brief Reinicia solamente estado y P, conservando ruidos y compuerta NIS.
 *
 * Resulta util para recuperar el filtro si el modelo inercial diverge y una
 * secuencia de mediciones barometricas coherentes queda fuera de la compuerta.
 */
esp_err_t altitude_kalman_reset(altitude_kalman_t *kf,
                                float height_m,
                                float velocity_mps,
                                float height_variance,
                                float velocity_variance);

float altitude_kalman_get_height(const altitude_kalman_t *kf);
float altitude_kalman_get_velocity(const altitude_kalman_t *kf);
float altitude_kalman_get_height_variance(const altitude_kalman_t *kf);
float altitude_kalman_get_velocity_variance(const altitude_kalman_t *kf);

#ifdef __cplusplus
}
#endif

#endif
