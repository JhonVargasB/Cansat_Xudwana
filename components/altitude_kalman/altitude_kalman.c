#include "altitude_kalman.h"

#include <math.h>
#include <string.h>
#include "esp_log.h"

static const char *TAG = "altitude_kalman";

#define P_MIN_DIAGONAL 1.0e-9f
#define S_MIN_VALUE    1.0e-9f
#define OUTLIER_LOG_DECIMATION 20U

#ifndef ALTITUDE_KALMAN_DEBUG
#define ALTITUDE_KALMAN_DEBUG 0
#endif

static uint32_t rejected_measurements;

static bool finite_nonnegative(float value)
{
    return isfinite(value) && value >= 0.0f;
}

/*
 * Para matrices fijas de 2x2, las expresiones escalares son mas claras y usan
 * menos memoria que construir objetos Mat de ESP-DSP. ESP-DSP 1.8.2 ofrece
 * dspm_mult/dspm_add, pero requeriria buffers intermedios para apenas cuatro
 * elementos. No hay asignacion dinamica ni operaciones matriciales inventadas.
 */
static void stabilize_covariance(
    float covariance[ALTITUDE_KALMAN_STATE_DIM]
                    [ALTITUDE_KALMAN_STATE_DIM])
{
    const float off_diagonal =
        0.5f * (covariance[0][1] + covariance[1][0]);
    covariance[0][1] = off_diagonal;
    covariance[1][0] = off_diagonal;
    covariance[0][0] = fmaxf(covariance[0][0], P_MIN_DIAGONAL);
    covariance[1][1] = fmaxf(covariance[1][1], P_MIN_DIAGONAL);
}

esp_err_t altitude_kalman_set_noise(altitude_kalman_t *kf,
                                    float acceleration_noise_std,
                                    float barometer_noise_std)
{
    if (kf == NULL || !finite_nonnegative(acceleration_noise_std) ||
        !isfinite(barometer_noise_std) || barometer_noise_std <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    kf->config.acceleration_noise_std = acceleration_noise_std;
    kf->config.barometer_noise_std = barometer_noise_std;
    kf->matrices.barometer_variance_R =
        barometer_noise_std * barometer_noise_std;
    return isfinite(kf->matrices.barometer_variance_R) &&
                   kf->matrices.barometer_variance_R > 0.0f
               ? ESP_OK
               : ESP_ERR_INVALID_ARG;
}

esp_err_t altitude_kalman_set_nis_gate(altitude_kalman_t *kf,
                                       bool enabled,
                                       float threshold)
{
    if (kf == NULL || !isfinite(threshold) || threshold <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    kf->config.nis_gate_enabled = enabled;
    kf->config.nis_threshold = threshold;
    return ESP_OK;
}

esp_err_t altitude_kalman_reset(altitude_kalman_t *kf,
                                float height_m,
                                float velocity_mps,
                                float height_variance,
                                float velocity_variance)
{
    if (kf == NULL || !kf->initialized || !isfinite(height_m) ||
        !isfinite(velocity_mps) || !finite_nonnegative(height_variance) ||
        !finite_nonnegative(velocity_variance)) {
        return ESP_ERR_INVALID_ARG;
    }

    kf->state.height_m = height_m;
    kf->state.vertical_velocity_m_s = velocity_mps;
    kf->matrices.covariance_P[0][0] =
        fmaxf(height_variance, P_MIN_DIAGONAL);
    kf->matrices.covariance_P[0][1] = 0.0f;
    kf->matrices.covariance_P[1][0] = 0.0f;
    kf->matrices.covariance_P[1][1] =
        fmaxf(velocity_variance, P_MIN_DIAGONAL);
    memset(&kf->diagnostics, 0, sizeof(kf->diagnostics));
    return ESP_OK;
}

esp_err_t altitude_kalman_init(altitude_kalman_t *kf,
                               const altitude_kalman_config_t *config)
{
    if (kf == NULL || config == NULL ||
        !isfinite(config->initial_height_m) ||
        !isfinite(config->initial_velocity_mps) ||
        !finite_nonnegative(config->initial_height_variance) ||
        !finite_nonnegative(config->initial_velocity_variance) ||
        !isfinite(config->nis_threshold) || config->nis_threshold <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(kf, 0, sizeof(*kf));
    kf->config = *config;
    kf->state.height_m = config->initial_height_m;
    kf->state.vertical_velocity_m_s = config->initial_velocity_mps;
    kf->matrices.covariance_P[0][0] =
        fmaxf(config->initial_height_variance, P_MIN_DIAGONAL);
    kf->matrices.covariance_P[1][1] =
        fmaxf(config->initial_velocity_variance, P_MIN_DIAGONAL);

    esp_err_t err = altitude_kalman_set_noise(
        kf, config->acceleration_noise_std, config->barometer_noise_std);
    if (err != ESP_OK) {
        memset(kf, 0, sizeof(*kf));
        return err;
    }
    kf->initialized = true;
    return ESP_OK;
}

esp_err_t altitude_kalman_predict(altitude_kalman_t *kf,
                                  float vertical_acceleration_mps2,
                                  float dt_s)
{
    if (kf == NULL || !kf->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!isfinite(vertical_acceleration_mps2) || !isfinite(dt_s) || dt_s <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    const float dt2 = dt_s * dt_s;
    const float dt3 = dt2 * dt_s;
    const float dt4 = dt2 * dt2;
    const float sigma_a2 = kf->config.acceleration_noise_std *
                           kf->config.acceleration_noise_std;

    if (!isfinite(dt4) || !isfinite(sigma_a2)) {
        return ESP_ERR_INVALID_ARG;
    }

    /* A=[[1,dt],[0,1]], B=[dt^2/2,dt]. Prediccion solo con una IMU nueva. */
    const float old_height = kf->state.height_m;
    const float old_velocity = kf->state.vertical_velocity_m_s;
    const float new_height = old_height + dt_s * old_velocity +
                             0.5f * dt2 * vertical_acceleration_mps2;
    const float new_velocity = old_velocity + dt_s * vertical_acceleration_mps2;

    /* Q modela incertidumbre efectiva: sensor, orientacion, vibracion y
       retirada imperfecta de gravedad; por ello cambia con cada dt. */
    float (*Q)[ALTITUDE_KALMAN_STATE_DIM] =
        kf->matrices.process_noise_Q;
    Q[0][0] = sigma_a2 * 0.25f * dt4;
    Q[0][1] = sigma_a2 * 0.5f * dt3;
    Q[1][0] = Q[0][1];
    Q[1][1] = sigma_a2 * dt2;

    const float (*P)[ALTITUDE_KALMAN_STATE_DIM] =
        kf->matrices.covariance_P;
    const float p00 = P[0][0];
    const float p01 = P[0][1];
    const float p10 = P[1][0];
    const float p11 = P[1][1];
    float new_P[ALTITUDE_KALMAN_STATE_DIM]
               [ALTITUDE_KALMAN_STATE_DIM];
    new_P[0][0] = p00 + dt_s * (p01 + p10) + dt2 * p11 + Q[0][0];
    new_P[0][1] = p01 + dt_s * p11 + Q[0][1];
    new_P[1][0] = p10 + dt_s * p11 + Q[1][0];
    new_P[1][1] = p11 + Q[1][1];

    if (!isfinite(new_height) || !isfinite(new_velocity) ||
        !isfinite(new_P[0][0]) || !isfinite(new_P[0][1]) ||
        !isfinite(new_P[1][0]) || !isfinite(new_P[1][1])) {
        return ESP_ERR_INVALID_STATE;
    }
    kf->state.height_m = new_height;
    kf->state.vertical_velocity_m_s = new_velocity;
    memcpy(kf->matrices.covariance_P, new_P, sizeof(new_P));
    stabilize_covariance(kf->matrices.covariance_P);
#if ALTITUDE_KALMAN_DEBUG
    ESP_LOGI(TAG, "predict P=[%.6g %.6g; %.6g %.6g]",
             (double)kf->matrices.covariance_P[0][0],
             (double)kf->matrices.covariance_P[0][1],
             (double)kf->matrices.covariance_P[1][0],
             (double)kf->matrices.covariance_P[1][1]);
#endif
    return ESP_OK;
}

esp_err_t altitude_kalman_correct_baro(altitude_kalman_t *kf,
                                       float barometric_height_m)
{
    if (kf == NULL || !kf->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!isfinite(barometric_height_m)) {
        return ESP_ERR_INVALID_ARG;
    }

    /* H=[1,0]: el barometro observa unicamente la altura. */
    const float innovation = barometric_height_m - kf->state.height_m;
    const float S = kf->matrices.covariance_P[0][0] +
                    kf->matrices.barometer_variance_R;
    if (!isfinite(innovation) || !isfinite(S) || S <= S_MIN_VALUE) {
        return ESP_ERR_INVALID_STATE;
    }

    const float nis = (innovation * innovation) / S;
    if (!isfinite(nis)) {
        return ESP_ERR_INVALID_STATE;
    }
    kf->diagnostics.last_innovation = innovation;
    kf->diagnostics.last_innovation_variance = S;
    kf->diagnostics.last_nis = nis;
    kf->diagnostics.last_measurement_accepted = false;

    if (kf->config.nis_gate_enabled && nis > kf->config.nis_threshold) {
        rejected_measurements++;
        if (rejected_measurements == 1U ||
            (rejected_measurements % OUTLIER_LOG_DECIMATION) == 0U) {
            ESP_LOGW(TAG, "Barometro rechazado: NIS=%.3f > %.3f (rechazos=%lu)",
                     (double)nis, (double)kf->config.nis_threshold,
                     (unsigned long)rejected_measurements);
        }
        return ALTITUDE_KALMAN_ERR_OUTLIER;
    }

    const float (*P)[ALTITUDE_KALMAN_STATE_DIM] =
        kf->matrices.covariance_P;
    const float p00 = P[0][0];
    const float p01 = P[0][1];
    const float p10 = P[1][0];
    const float p11 = P[1][1];
    const float k0 = p00 / S;
    const float k1 = p10 / S;
    kf->diagnostics.last_gain[ALTITUDE_KALMAN_HEIGHT] = k0;
    kf->diagnostics.last_gain[ALTITUDE_KALMAN_VERTICAL_VELOCITY] = k1;

    const float new_height = kf->state.height_m + k0 * innovation;
    const float new_velocity =
        kf->state.vertical_velocity_m_s + k1 * innovation;

    /* Forma de Joseph: (I-KH)P(I-KH)^T + KRK^T. Mantiene P positiva
       semidefinida mejor que la resta directa bajo redondeo float. */
    const float m00 = 1.0f - k0;
    float new_P[ALTITUDE_KALMAN_STATE_DIM]
               [ALTITUDE_KALMAN_STATE_DIM];
    const float R = kf->matrices.barometer_variance_R;
    new_P[0][0] = m00 * m00 * p00 + k0 * k0 * R;
    new_P[0][1] = m00 * (p01 - k1 * p00) + k0 * k1 * R;
    new_P[1][0] = m00 * (p10 - k1 * p00) + k1 * k0 * R;
    new_P[1][1] = p11 - k1 * (p01 + p10) + k1 * k1 * p00 +
                  k1 * k1 * R;

    if (!isfinite(new_height) || !isfinite(new_velocity) ||
        !isfinite(new_P[0][0]) || !isfinite(new_P[0][1]) ||
        !isfinite(new_P[1][0]) || !isfinite(new_P[1][1])) {
        return ESP_ERR_INVALID_STATE;
    }
    kf->state.height_m = new_height;
    kf->state.vertical_velocity_m_s = new_velocity;
    memcpy(kf->matrices.covariance_P, new_P, sizeof(new_P));
    stabilize_covariance(kf->matrices.covariance_P);
    kf->diagnostics.last_measurement_accepted = true;
#if ALTITUDE_KALMAN_DEBUG
    ESP_LOGI(TAG, "correct K=[%.6g %.6g], innov=%.6g, S=%.6g, NIS=%.6g, P=[%.6g %.6g; %.6g %.6g]",
             (double)k0, (double)k1, (double)innovation, (double)S,
             (double)nis,
             (double)kf->matrices.covariance_P[0][0],
             (double)kf->matrices.covariance_P[0][1],
             (double)kf->matrices.covariance_P[1][0],
             (double)kf->matrices.covariance_P[1][1]);
#endif
    return ESP_OK;
}

float altitude_kalman_get_height(const altitude_kalman_t *kf)
{
    return (kf != NULL && kf->initialized) ? kf->state.height_m : NAN;
}

float altitude_kalman_get_velocity(const altitude_kalman_t *kf)
{
    return (kf != NULL && kf->initialized)
               ? kf->state.vertical_velocity_m_s
               : NAN;
}

float altitude_kalman_get_height_variance(const altitude_kalman_t *kf)
{
    return (kf != NULL && kf->initialized)
               ? kf->matrices.covariance_P[0][0]
               : NAN;
}

float altitude_kalman_get_velocity_variance(const altitude_kalman_t *kf)
{
    return (kf != NULL && kf->initialized)
               ? kf->matrices.covariance_P[1][1]
               : NAN;
}
