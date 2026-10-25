#include "altitude_kalman_helpers.h"

#include <math.h>

esp_err_t altitude_kalman_configure_motion_state(
    altitude_kalman_t *filter,
    const altitude_kalman_motion_config_t *config,
    bool stationary)
{
    if (filter == NULL || config == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    const float accel_noise_std =
        stationary ? config->stationary_acceleration_noise_std
                   : config->moving_acceleration_noise_std;
    const float nis_threshold =
        stationary ? config->stationary_nis_threshold
                   : config->moving_nis_threshold;

    esp_err_t err = altitude_kalman_set_noise(
        filter, accel_noise_std, config->barometer_noise_std);
    if (err != ESP_OK)
    {
        return err;
    }

    /* En movimiento la IMU puede separar mucho la prediccion. No se debe
       rechazar entonces al sensor que puede volver a anclar la altura. */
    return altitude_kalman_set_nis_gate(filter, stationary, nis_threshold);
}

esp_err_t altitude_kalman_progressive_baro_recovery(
    altitude_kalman_t *filter,
    float barometric_height_m)
{
    if (filter == NULL || !isfinite(barometric_height_m))
    {
        return ESP_ERR_INVALID_ARG;
    }

    const float threshold = filter->config.nis_threshold;
    esp_err_t err = altitude_kalman_set_nis_gate(filter, false, threshold);
    if (err == ESP_OK)
    {
        err = altitude_kalman_correct_baro(filter, barometric_height_m);
    }
    const esp_err_t restore_err =
        altitude_kalman_set_nis_gate(filter, true, threshold);
    return err != ESP_OK ? err : restore_err;
}
