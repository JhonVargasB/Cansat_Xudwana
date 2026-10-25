#include "ekq_q.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "esp_timer.h"

#define EKF_Q_PI 3.14159265358979323846f

/*
 * mantiene el angulo en [-pi, pi]
 */

static float ekf_q_wrap_angle(float angle_rad)
{
    while (angle_rad > EKF_Q_PI)
    {
        angle_rad -= 2.0f * EKF_Q_PI;
    }
    while (angle_rad < -EKF_Q_PI)
    {
        angle_rad += 2.0f * EKF_Q_PI;
    }
    return angle_rad;
}

/*
 * Calcula si el vector de dimension count contiene solo valores finitos.
 */
static bool ekf_q_is_finite_vector(const float *values, size_t count)
{
    if (values == NULL)
        return false;

    for (size_t index = 0; index < count; index++)
    {
        if (!isfinite(values[index]))
            return false;
    }

    return true;
}

// Calcula la norma de un vector de 3 elementos
static float ekf_q_norm3(const float vector[3])
{
    return sqrtf(vector[0] * vector[0] +
                 vector[1] * vector[1] +
                 vector[2] * vector[2]);
}

// Calcula la norma de un vector de 3 elementos
static float ekf_q_norm4(const float quaternion[4])
{
    return sqrtf(quaternion[0] * quaternion[0] +
                 quaternion[1] * quaternion[1] +
                 quaternion[2] * quaternion[2] +
                 quaternion[3] * quaternion[3]);
}

// Rotacion de un cuaternion q = [w, x, y, z] por otro q' = [w', x', y', z']., doble rotacion
static void ekf_q_quaternion_multiply(const float left[4],
                                      const float right[4],
                                      float result[4])
{
    result[0] = left[0] * right[0] - left[1] * right[1] -
                left[2] * right[2] - left[3] * right[3];
    result[1] = left[0] * right[1] + left[1] * right[0] +
                left[2] * right[3] - left[3] * right[2];
    result[2] = left[0] * right[2] - left[1] * right[3] +
                left[2] * right[0] + left[3] * right[1];
    result[3] = left[0] * right[3] + left[1] * right[2] -
                left[2] * right[1] + left[3] * right[0];
}

/*
    Normaliza el cuaternion, los 3 primeros estados y ajusta la covarianza de estado.
*/
static bool ekf_q_normalize_candidate(float state[EKF_Q_STATE_DIM],
                                      float covariance[EKF_Q_STATE_DIM][EKF_Q_STATE_DIM],
                                      float epsilon,
                                      ekf_q_workspace_t *workspace)
{
    if (workspace == NULL)
        return false;
    // Saca la norma del cuaternion
    float norm = ekf_q_norm4(state);

    // verificamos si es válido
    if (!isfinite(norm) || norm <= epsilon)
        return false;
    
    float normalized[4];
    
    for (int index = 0; index < EKF_Q_QUAT_DIM; index++)
    {
        normalized[index] = state[index] / norm;
    }

    float (*transform)[EKF_Q_STATE_DIM] = workspace->normalize_transform;

    float (*temporary)[EKF_Q_STATE_DIM] = workspace->normalize_temporary;

    float (*normalized_covariance)[EKF_Q_STATE_DIM] =
        workspace->normalize_covariance;
    memset(transform, 0, sizeof(workspace->normalize_transform));
    memset(temporary, 0, sizeof(workspace->normalize_temporary));
    memset(normalized_covariance, 0,
           sizeof(workspace->normalize_covariance));

    for (int index = EKF_Q_BAX; index < EKF_Q_STATE_DIM; index++)
    {
        transform[index][index] = 1.0f;
    }
    for (int row = 0; row < EKF_Q_QUAT_DIM; row++)
    {
        for (int col = 0; col < EKF_Q_QUAT_DIM; col++)
        {
            float identity = (row == col) ? 1.0f : 0.0f;
            transform[row][col] =
                (identity - normalized[row] * normalized[col]) / norm;
        }
    }

    for (int row = 0; row < EKF_Q_STATE_DIM; row++)
    {
        for (int col = 0; col < EKF_Q_STATE_DIM; col++)
        {
            for (int inner = 0; inner < EKF_Q_STATE_DIM; inner++)
            {
                temporary[row][col] += transform[row][inner] *
                                       covariance[inner][col];
            }
        }
    }
    for (int row = 0; row < EKF_Q_STATE_DIM; row++)
    {
        for (int col = 0; col < EKF_Q_STATE_DIM; col++)
        {
            for (int inner = 0; inner < EKF_Q_STATE_DIM; inner++)
            {
                normalized_covariance[row][col] += temporary[row][inner] *
                                                   transform[col][inner];
            }
        }
    }

    for (int index = 0; index < EKF_Q_QUAT_DIM; index++)
    {
        state[index] = normalized[index];
    }
    memcpy(covariance, normalized_covariance,
           sizeof(workspace->normalize_covariance));
    return true;
}


static bool ekf_q_validate_covariance(
    float covariance[EKF_Q_STATE_DIM][EKF_Q_STATE_DIM],
    float epsilon)
{
    for (int row = 0; row < EKF_Q_STATE_DIM; row++)
    {
        for (int col = row; col < EKF_Q_STATE_DIM; col++)
        {
            float first = covariance[row][col];
            float second = covariance[col][row];
            if (!isfinite(first) || !isfinite(second))
            {
                return false;
            }

            float symmetric = 0.5f * (first + second);
            covariance[row][col] = symmetric;
            covariance[col][row] = symmetric;
        }

        if (covariance[row][row] < -epsilon)
        {
            return false;
        }
        if (covariance[row][row] < 0.0f)
        {
            covariance[row][row] = 0.0f;
        }
    }
    return true;
}

static void ekf_q_refresh_diagnostics(ekf_q_t *ekf)
{
    memcpy(ekf->diagnostics.quaternion, ekf->x, sizeof(float) * 4);
    ekf_q_get_euler(ekf,
                    &ekf->diagnostics.roll_rad,
                    &ekf->diagnostics.pitch_rad,
                    &ekf->diagnostics.yaw_rad);
    ekf->diagnostics.accel_bias_m_s2[0] = ekf->x[EKF_Q_BAX];
    ekf->diagnostics.accel_bias_m_s2[1] = ekf->x[EKF_Q_BAY];
    ekf->diagnostics.accel_bias_m_s2[2] = ekf->x[EKF_Q_BAZ];
    ekf->diagnostics.quaternion_norm = ekf_q_norm4(ekf->x);
    for (int index = 0; index < EKF_Q_STATE_DIM; index++)
    {
        ekf->diagnostics.covariance_diagonal[index] = ekf->P[index][index];
    }
}

static bool ekf_q_config_is_valid(const ekf_q_config_t *config)
{
    if (config == NULL)
    {
        return false;
    }

    const float values[] = {
        config->gravity_m_s2,
        config->gyro_noise_std_rad_s,
        config->accel_noise_std_m_s2,
        config->accel_bias_process_noise_std_m_s2_sqrt_s,
        config->accel_bias_alpha_s_inv,
        config->initial_quaternion_variance,
        config->initial_accel_bias_variance,
        config->innovation_threshold_m_s2,
        config->min_accel_norm_m_s2,
        config->min_dt_s,
        config->max_dt_s,
        config->numerical_epsilon,
    };
    if (!ekf_q_is_finite_vector(values, sizeof(values) / sizeof(values[0])))
    {
        return false;
    }

    return config->gravity_m_s2 > 0.0f &&
           config->gyro_noise_std_rad_s >= 0.0f &&
           config->accel_noise_std_m_s2 > 0.0f &&
           config->accel_bias_process_noise_std_m_s2_sqrt_s >= 0.0f &&
           config->accel_bias_alpha_s_inv >= 0.0f &&
           config->initial_quaternion_variance > 0.0f &&
           config->initial_accel_bias_variance > 0.0f &&
           config->innovation_threshold_m_s2 > 0.0f &&
           config->min_accel_norm_m_s2 >= 0.0f &&
           config->min_dt_s > 0.0f &&
           config->max_dt_s > config->min_dt_s &&
           config->numerical_epsilon > 0.0f;
}

static bool ekf_q_cholesky_solve(const float matrix[3][3],
                                 const float rhs[3],
                                 int dimension,
                                 float epsilon,
                                 float solution[3])
{
    float lower[3][3] = {0};
    for (int row = 0; row < dimension; row++)
    {
        for (int col = 0; col <= row; col++)
        {
            float sum = matrix[row][col];
            for (int inner = 0; inner < col; inner++)
            {
                sum -= lower[row][inner] * lower[col][inner];
            }

            if (row == col)
            {
                if (!isfinite(sum) || sum <= epsilon)
                {
                    return false;
                }
                lower[row][col] = sqrtf(sum);
            }
            else
            {
                lower[row][col] = sum / lower[col][col];
            }
        }
    }

    float intermediate[3] = {0};
    for (int row = 0; row < dimension; row++)
    {
        float sum = rhs[row];
        for (int col = 0; col < row; col++)
        {
            sum -= lower[row][col] * intermediate[col];
        }
        intermediate[row] = sum / lower[row][row];
    }

    for (int row = dimension - 1; row >= 0; row--)
    {
        float sum = intermediate[row];
        for (int col = row + 1; col < dimension; col++)
        {
            sum -= lower[col][row] * solution[col];
        }
        solution[row] = sum / lower[row][row];
    }

    return ekf_q_is_finite_vector(solution, (size_t)dimension);
}

esp_err_t ekf_q_init(ekf_q_t *ekf, const ekf_q_config_t *config)
{
    if (ekf == NULL || !ekf_q_config_is_valid(config))
    {
        return ESP_ERR_INVALID_ARG;
    }

    memset(ekf, 0, sizeof(*ekf));
    ekf->config = *config;
    ekf->x[EKF_Q_QW] = 1.0f;
    ekf->diagnostics.min_total_us = UINT32_MAX;

    float accel_variance = config->accel_noise_std_m_s2 *
                           config->accel_noise_std_m_s2;
    for (int axis = 0; axis < EKF_Q_MEAS_DIM; axis++)
    {
        ekf->R[axis][axis] = accel_variance;
    }
    ekf_q_refresh_diagnostics(ekf);
    return ESP_OK;
}

esp_err_t ekf_q_set_initial_attitude(ekf_q_t *ekf,
                                     const float accel_body_m_s2[3])
{
    if (ekf == NULL || !ekf_q_is_finite_vector(accel_body_m_s2, 3))
    {
        return ESP_ERR_INVALID_ARG;
    }

    float accel_norm = ekf_q_norm3(accel_body_m_s2);
    if (!isfinite(accel_norm) || accel_norm < ekf->config.min_accel_norm_m_s2)
    {
        return ESP_ERR_INVALID_STATE;
    }

    /* En NED, quieto: f_b = C_nb * [0,0,-g]. */
    float roll = atan2f(-accel_body_m_s2[1], -accel_body_m_s2[2]);
    float pitch = atan2f(accel_body_m_s2[0],
                         sqrtf(accel_body_m_s2[1] * accel_body_m_s2[1] +
                               accel_body_m_s2[2] * accel_body_m_s2[2]));
    float half_roll = 0.5f * roll;
    float half_pitch = 0.5f * pitch;
    float cosine_roll = cosf(half_roll);
    float sine_roll = sinf(half_roll);
    float cosine_pitch = cosf(half_pitch);
    float sine_pitch = sinf(half_pitch);

    memset(ekf->x, 0, sizeof(ekf->x));
    ekf->x[EKF_Q_QW] = cosine_roll * cosine_pitch;
    ekf->x[EKF_Q_QX] = sine_roll * cosine_pitch;
    ekf->x[EKF_Q_QY] = cosine_roll * sine_pitch;
    ekf->x[EKF_Q_QZ] = -sine_roll * sine_pitch;

    memset(ekf->P, 0, sizeof(ekf->P));
    for (int index = 0; index < EKF_Q_QUAT_DIM; index++)
    {
        ekf->P[index][index] = ekf->config.initial_quaternion_variance;
    }
    for (int index = EKF_Q_BAX; index < EKF_Q_STATE_DIM; index++)
    {
        ekf->P[index][index] = ekf->config.initial_accel_bias_variance;
    }

    if (!ekf_q_normalize_candidate(ekf->x, ekf->P,
                                   ekf->config.numerical_epsilon,
                                   &ekf->workspace) ||
        !ekf_q_validate_covariance(ekf->P, ekf->config.numerical_epsilon))
    {
        ekf->diagnostics.numerical_error_count++;
        ekf->diagnostics.initialized = false;
        return ESP_ERR_INVALID_RESPONSE;
    }

    ekf->diagnostics.accel_norm_m_s2 = accel_norm;
    ekf->diagnostics.initialized = true;
    ekf_q_refresh_diagnostics(ekf);
    return ESP_OK;
}

void ekf_q_set_gyro_bias(ekf_q_t *ekf,
                         const float gyro_bias_body_rad_s[3])
{
    if (ekf == NULL || !ekf_q_is_finite_vector(gyro_bias_body_rad_s, 3))
    {
        return;
    }
    memcpy(ekf->diagnostics.gyro_bias_rad_s,
           gyro_bias_body_rad_s,
           sizeof(ekf->diagnostics.gyro_bias_rad_s));
}

esp_err_t ekf_q_predict(ekf_q_t *ekf,
                        const float gyro_body_rad_s[3],
                        float dt_s)
{
    int64_t start_us = esp_timer_get_time();
    if (ekf == NULL || !ekf->diagnostics.initialized ||
        !ekf_q_is_finite_vector(gyro_body_rad_s, 3) || !isfinite(dt_s) ||
        dt_s < ekf->config.min_dt_s || dt_s > ekf->config.max_dt_s)
    {
        return ESP_ERR_INVALID_ARG;
    }

    float previous_quaternion[4] = {
        ekf->x[EKF_Q_QW],
        ekf->x[EKF_Q_QX],
        ekf->x[EKF_Q_QY],
        ekf->x[EKF_Q_QZ],
    };
    float angular_rate = ekf_q_norm3(gyro_body_rad_s);
    float delta_quaternion[4];
    if (angular_rate * dt_s < 1.0e-6f)
    {
        delta_quaternion[0] = 1.0f;
        delta_quaternion[1] = 0.5f * gyro_body_rad_s[0] * dt_s;
        delta_quaternion[2] = 0.5f * gyro_body_rad_s[1] * dt_s;
        delta_quaternion[3] = 0.5f * gyro_body_rad_s[2] * dt_s;
    }
    else
    {
        float half_angle = 0.5f * angular_rate * dt_s;
        float scale = sinf(half_angle) / angular_rate;
        delta_quaternion[0] = cosf(half_angle);
        delta_quaternion[1] = gyro_body_rad_s[0] * scale;
        delta_quaternion[2] = gyro_body_rad_s[1] * scale;
        delta_quaternion[3] = gyro_body_rad_s[2] * scale;
    }

    float predicted_quaternion[4];
    ekf_q_quaternion_multiply(previous_quaternion,
                              delta_quaternion,
                              predicted_quaternion);
    memcpy(ekf->x, predicted_quaternion, sizeof(predicted_quaternion));

    float bias_decay = expf(-ekf->config.accel_bias_alpha_s_inv * dt_s);
    ekf->x[EKF_Q_BAX] *= bias_decay;
    ekf->x[EKF_Q_BAY] *= bias_decay;
    ekf->x[EKF_Q_BAZ] *= bias_decay;

    memset(ekf->F, 0, sizeof(ekf->F));
    const float dw = delta_quaternion[0];
    const float dx = delta_quaternion[1];
    const float dy = delta_quaternion[2];
    const float dz = delta_quaternion[3];
    const float quaternion_transition[4][4] = {
        {dw, -dx, -dy, -dz},
        {dx, dw, dz, -dy},
        {dy, -dz, dw, dx},
        {dz, dy, -dx, dw},
    };
    for (int row = 0; row < EKF_Q_QUAT_DIM; row++)
    {
        for (int col = 0; col < EKF_Q_QUAT_DIM; col++)
        {
            ekf->F[row][col] = quaternion_transition[row][col];
        }
    }
    ekf->F[EKF_Q_BAX][EKF_Q_BAX] = bias_decay;
    ekf->F[EKF_Q_BAY][EKF_Q_BAY] = bias_decay;
    ekf->F[EKF_Q_BAZ][EKF_Q_BAZ] = bias_decay;

    memset(ekf->Q, 0, sizeof(ekf->Q));
    float gyro_mapping[4][3] = {
        {-previous_quaternion[1], -previous_quaternion[2], -previous_quaternion[3]},
        {previous_quaternion[0], -previous_quaternion[3], previous_quaternion[2]},
        {previous_quaternion[3], previous_quaternion[0], -previous_quaternion[1]},
        {-previous_quaternion[2], previous_quaternion[1], previous_quaternion[0]},
    };
    float gyro_variance = ekf->config.gyro_noise_std_rad_s *
                          ekf->config.gyro_noise_std_rad_s;
    float quaternion_noise_scale = 0.25f * dt_s * dt_s * gyro_variance;
    for (int row = 0; row < EKF_Q_QUAT_DIM; row++)
    {
        for (int col = 0; col < EKF_Q_QUAT_DIM; col++)
        {
            for (int axis = 0; axis < 3; axis++)
            {
                ekf->Q[row][col] += quaternion_noise_scale *
                                    gyro_mapping[row][axis] *
                                    gyro_mapping[col][axis];
            }
        }
    }

    float bias_noise_variance =
        ekf->config.accel_bias_process_noise_std_m_s2_sqrt_s *
        ekf->config.accel_bias_process_noise_std_m_s2_sqrt_s;
    float bias_process_variance;
    if (ekf->config.accel_bias_alpha_s_inv > ekf->config.numerical_epsilon)
    {
        float alpha = ekf->config.accel_bias_alpha_s_inv;
        bias_process_variance = bias_noise_variance *
                                (1.0f - expf(-2.0f * alpha * dt_s)) / (2.0f * alpha);
    }
    else
    {
        /* alpha=0: el bias se comporta como random walk. */
        bias_process_variance = bias_noise_variance * dt_s;
    }
    ekf->Q[EKF_Q_BAX][EKF_Q_BAX] = bias_process_variance;
    ekf->Q[EKF_Q_BAY][EKF_Q_BAY] = bias_process_variance;
    ekf->Q[EKF_Q_BAZ][EKF_Q_BAZ] = bias_process_variance;

    float (*transition_covariance)[EKF_Q_STATE_DIM] =
        ekf->workspace.predict_transition_covariance;
    float (*predicted_covariance)[EKF_Q_STATE_DIM] =
        ekf->workspace.predict_covariance;
    memset(transition_covariance, 0,
           sizeof(ekf->workspace.predict_transition_covariance));
    memset(predicted_covariance, 0,
           sizeof(ekf->workspace.predict_covariance));
    for (int row = 0; row < EKF_Q_STATE_DIM; row++)
    {
        for (int col = 0; col < EKF_Q_STATE_DIM; col++)
        {
            for (int inner = 0; inner < EKF_Q_STATE_DIM; inner++)
            {
                transition_covariance[row][col] += ekf->F[row][inner] *
                                                   ekf->P[inner][col];
            }
        }
    }
    for (int row = 0; row < EKF_Q_STATE_DIM; row++)
    {
        for (int col = 0; col < EKF_Q_STATE_DIM; col++)
        {
            predicted_covariance[row][col] = ekf->Q[row][col];
            for (int inner = 0; inner < EKF_Q_STATE_DIM; inner++)
            {
                predicted_covariance[row][col] +=
                    transition_covariance[row][inner] * ekf->F[col][inner];
            }
        }
    }

    memcpy(ekf->P, predicted_covariance,
           sizeof(ekf->workspace.predict_covariance));
    if (!ekf_q_normalize_candidate(ekf->x, ekf->P,
                                   ekf->config.numerical_epsilon,
                                   &ekf->workspace) ||
        !ekf_q_validate_covariance(ekf->P, ekf->config.numerical_epsilon))
    {
        ekf->diagnostics.numerical_error_count++;
        ekf->diagnostics.initialized = false;
        ekf->x[EKF_Q_QW] = 1.0f;
        ekf->x[EKF_Q_QX] = 0.0f;
        ekf->x[EKF_Q_QY] = 0.0f;
        ekf->x[EKF_Q_QZ] = 0.0f;
        return ESP_ERR_INVALID_RESPONSE;
    }

    ekf->diagnostics.last_dt_s = dt_s;
    ekf->diagnostics.prediction_count++;
    ekf->diagnostics.last_predict_us =
        (uint32_t)(esp_timer_get_time() - start_us);
    ekf_q_refresh_diagnostics(ekf);
    return ESP_OK;
}

esp_err_t ekf_q_correct_accel(ekf_q_t *ekf,
                              const float accel_body_m_s2[3])
{
    int64_t start_us = esp_timer_get_time();
    if (ekf == NULL || !ekf->diagnostics.initialized ||
        !ekf_q_is_finite_vector(accel_body_m_s2, 3))
    {
        return ESP_ERR_INVALID_ARG;
    }

    float accel_norm = ekf_q_norm3(accel_body_m_s2);
    ekf->diagnostics.accel_norm_m_s2 = accel_norm;
    memset(ekf->H, 0, sizeof(ekf->H));
    memset(ekf->K, 0, sizeof(ekf->K));
    memset(ekf->S, 0, sizeof(ekf->S));

    const float w = ekf->x[EKF_Q_QW];
    const float x = ekf->x[EKF_Q_QX];
    const float y = ekf->x[EKF_Q_QY];
    const float z = ekf->x[EKF_Q_QZ];
    const float gravity = ekf->config.gravity_m_s2;

    float predicted[3] = {
        2.0f * gravity * (w * y - x * z) + ekf->x[EKF_Q_BAX],
        -2.0f * gravity * (w * x + y * z) + ekf->x[EKF_Q_BAY],
        -gravity * (1.0f - 2.0f * (x * x + y * y)) + ekf->x[EKF_Q_BAZ],
    };

    ekf->H[0][EKF_Q_QW] = 2.0f * gravity * y;
    ekf->H[0][EKF_Q_QX] = -2.0f * gravity * z;
    ekf->H[0][EKF_Q_QY] = 2.0f * gravity * w;
    ekf->H[0][EKF_Q_QZ] = -2.0f * gravity * x;
    ekf->H[0][EKF_Q_BAX] = 1.0f;

    ekf->H[1][EKF_Q_QW] = -2.0f * gravity * x;
    ekf->H[1][EKF_Q_QX] = -2.0f * gravity * w;
    ekf->H[1][EKF_Q_QY] = -2.0f * gravity * z;
    ekf->H[1][EKF_Q_QZ] = -2.0f * gravity * y;
    ekf->H[1][EKF_Q_BAY] = 1.0f;

    ekf->H[2][EKF_Q_QW] = 0.0f;
    ekf->H[2][EKF_Q_QX] = 4.0f * gravity * x;
    ekf->H[2][EKF_Q_QY] = 4.0f * gravity * y;
    ekf->H[2][EKF_Q_QZ] = 0.0f;
    ekf->H[2][EKF_Q_BAZ] = 1.0f;

    int active_axes[3] = {0};
    int active_count = 0;
    bool accel_norm_valid = isfinite(accel_norm) &&
                            accel_norm >= ekf->config.min_accel_norm_m_s2;
    for (int axis = 0; axis < 3; axis++)
    {
        ekf->nu[axis] = accel_body_m_s2[axis] - predicted[axis];
        bool accepted = accel_norm_valid && isfinite(ekf->nu[axis]) &&
                        fabsf(ekf->nu[axis]) <= ekf->config.innovation_threshold_m_s2;
        ekf->diagnostics.innovation_m_s2[axis] = ekf->nu[axis];
        ekf->diagnostics.accel_axis_accepted[axis] = accepted;
        if (accepted)
        {
            active_axes[active_count++] = axis;
        }
    }

    if (active_count < 3)
    {
        ekf->diagnostics.accel_rejection_count++;
    }
    if (active_count == 0)
    {
        ekf->diagnostics.last_correct_us =
            (uint32_t)(esp_timer_get_time() - start_us);
        ekf_q_refresh_diagnostics(ekf);
        return ESP_OK;
    }

    float (*reduced_h)[EKF_Q_STATE_DIM] =
        ekf->workspace.correct_reduced_h;
    float *reduced_nu = ekf->workspace.correct_reduced_nu;
    float (*pht)[EKF_Q_MEAS_DIM] = ekf->workspace.correct_pht;
    float (*reduced_s)[EKF_Q_MEAS_DIM] =
        ekf->workspace.correct_reduced_s;
    float (*reduced_k)[EKF_Q_MEAS_DIM] =
        ekf->workspace.correct_reduced_k;
    float *corrected_state = ekf->workspace.corrected_state;
    float (*joseph_factor)[EKF_Q_STATE_DIM] =
        ekf->workspace.correct_joseph_factor;
    float (*factor_covariance)[EKF_Q_STATE_DIM] =
        ekf->workspace.correct_factor_covariance;
    float (*corrected_covariance)[EKF_Q_STATE_DIM] =
        ekf->workspace.corrected_covariance;

    memset(reduced_h, 0, sizeof(ekf->workspace.correct_reduced_h));
    memset(reduced_nu, 0, sizeof(ekf->workspace.correct_reduced_nu));
    memset(pht, 0, sizeof(ekf->workspace.correct_pht));
    memset(reduced_s, 0, sizeof(ekf->workspace.correct_reduced_s));
    memset(reduced_k, 0, sizeof(ekf->workspace.correct_reduced_k));
    memset(joseph_factor, 0,
           sizeof(ekf->workspace.correct_joseph_factor));
    memset(factor_covariance, 0,
           sizeof(ekf->workspace.correct_factor_covariance));
    memset(corrected_covariance, 0,
           sizeof(ekf->workspace.corrected_covariance));

    for (int measurement = 0; measurement < active_count; measurement++)
    {
        int axis = active_axes[measurement];
        reduced_nu[measurement] = ekf->nu[axis];
        memcpy(reduced_h[measurement], ekf->H[axis],
               sizeof(reduced_h[measurement]));
    }

    for (int row = 0; row < EKF_Q_STATE_DIM; row++)
    {
        for (int measurement = 0; measurement < active_count; measurement++)
        {
            for (int inner = 0; inner < EKF_Q_STATE_DIM; inner++)
            {
                pht[row][measurement] += ekf->P[row][inner] *
                                         reduced_h[measurement][inner];
            }
        }
    }

    float measurement_variance = ekf->config.accel_noise_std_m_s2 *
                                 ekf->config.accel_noise_std_m_s2;
    for (int row = 0; row < active_count; row++)
    {
        for (int col = 0; col < active_count; col++)
        {
            for (int inner = 0; inner < EKF_Q_STATE_DIM; inner++)
            {
                reduced_s[row][col] += reduced_h[row][inner] * pht[inner][col];
            }
            if (row == col)
            {
                reduced_s[row][col] += measurement_variance;
            }
            ekf->S[active_axes[row]][active_axes[col]] = reduced_s[row][col];
        }
    }

    for (int state = 0; state < EKF_Q_STATE_DIM; state++)
    {
        float rhs[3] = {pht[state][0], pht[state][1], pht[state][2]};
        if (!ekf_q_cholesky_solve(reduced_s, rhs, active_count,
                                  ekf->config.numerical_epsilon,
                                  reduced_k[state]))
        {
            ekf->diagnostics.numerical_error_count++;
            return ESP_ERR_INVALID_RESPONSE;
        }
        for (int measurement = 0; measurement < active_count; measurement++)
        {
            ekf->K[state][active_axes[measurement]] =
                reduced_k[state][measurement];
        }
    }

    memcpy(corrected_state, ekf->x,
           sizeof(ekf->workspace.corrected_state));
    for (int state = 0; state < EKF_Q_STATE_DIM; state++)
    {
        for (int measurement = 0; measurement < active_count; measurement++)
        {
            corrected_state[state] += reduced_k[state][measurement] *
                                      reduced_nu[measurement];
        }
    }

    for (int row = 0; row < EKF_Q_STATE_DIM; row++)
    {
        joseph_factor[row][row] = 1.0f;
        for (int col = 0; col < EKF_Q_STATE_DIM; col++)
        {
            for (int measurement = 0; measurement < active_count; measurement++)
            {
                joseph_factor[row][col] -= reduced_k[row][measurement] *
                                           reduced_h[measurement][col];
            }
        }
    }

    for (int row = 0; row < EKF_Q_STATE_DIM; row++)
    {
        for (int col = 0; col < EKF_Q_STATE_DIM; col++)
        {
            for (int inner = 0; inner < EKF_Q_STATE_DIM; inner++)
            {
                factor_covariance[row][col] += joseph_factor[row][inner] *
                                               ekf->P[inner][col];
            }
        }
    }
    for (int row = 0; row < EKF_Q_STATE_DIM; row++)
    {
        for (int col = 0; col < EKF_Q_STATE_DIM; col++)
        {
            for (int inner = 0; inner < EKF_Q_STATE_DIM; inner++)
            {
                corrected_covariance[row][col] +=
                    factor_covariance[row][inner] * joseph_factor[col][inner];
            }
            for (int measurement = 0; measurement < active_count; measurement++)
            {
                corrected_covariance[row][col] +=
                    reduced_k[row][measurement] * measurement_variance *
                    reduced_k[col][measurement];
            }
        }
    }

    if (!ekf_q_normalize_candidate(corrected_state, corrected_covariance,
                                   ekf->config.numerical_epsilon,
                                   &ekf->workspace) ||
        !ekf_q_validate_covariance(corrected_covariance,
                                   ekf->config.numerical_epsilon) ||
        !ekf_q_is_finite_vector(corrected_state, EKF_Q_STATE_DIM))
    {
        ekf->diagnostics.numerical_error_count++;
        return ESP_ERR_INVALID_RESPONSE;
    }

    memcpy(ekf->x, corrected_state,
           sizeof(ekf->workspace.corrected_state));
    memcpy(ekf->P, corrected_covariance,
           sizeof(ekf->workspace.corrected_covariance));
    ekf->diagnostics.correction_count++;
    ekf->diagnostics.last_correct_us =
        (uint32_t)(esp_timer_get_time() - start_us);
    ekf_q_refresh_diagnostics(ekf);
    return ESP_OK;
}

esp_err_t ekf_q_update(ekf_q_t *ekf,
                       const float gyro_body_rad_s[3],
                       const float accel_body_m_s2[3],
                       float dt_s)
{
    if (ekf == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    int64_t start_us = esp_timer_get_time();
    esp_err_t err = ekf_q_predict(ekf, gyro_body_rad_s, dt_s);
    if (err == ESP_OK)
    {
        err = ekf_q_correct_accel(ekf, accel_body_m_s2);
    }

    uint32_t elapsed_us = (uint32_t)(esp_timer_get_time() - start_us);
    ekf->diagnostics.last_total_us = elapsed_us;
    if (elapsed_us < ekf->diagnostics.min_total_us)
    {
        ekf->diagnostics.min_total_us = elapsed_us;
    }
    if (elapsed_us > ekf->diagnostics.max_total_us)
    {
        ekf->diagnostics.max_total_us = elapsed_us;
    }
    ekf->diagnostics.total_time_us += elapsed_us;
    ekf->diagnostics.timed_update_count++;
    return err;
}

esp_err_t ekf_q_correct_yaw(ekf_q_t *ekf,
                            float measured_yaw_rad,
                            float measurement_noise_std_rad)
{
    if (ekf == NULL || !ekf->diagnostics.initialized ||
        !isfinite(measured_yaw_rad) ||
        !isfinite(measurement_noise_std_rad) ||
        measurement_noise_std_rad <= 0.0f)
    {
        return ESP_ERR_INVALID_ARG;
    }

    const float w = ekf->x[EKF_Q_QW];
    const float x = ekf->x[EKF_Q_QX];
    const float y = ekf->x[EKF_Q_QY];
    const float z = ekf->x[EKF_Q_QZ];
    const float yaw_numerator = 2.0f * (w * z + x * y);
    const float yaw_denominator = 1.0f - 2.0f * (y * y + z * z);
    const float yaw_gradient_denominator =
        yaw_numerator * yaw_numerator +
        yaw_denominator * yaw_denominator;
    if (!isfinite(yaw_gradient_denominator) ||
        yaw_gradient_denominator <= ekf->config.numerical_epsilon)
    {
        return ESP_ERR_INVALID_STATE;
    }

    float *h = ekf->workspace.correct_reduced_h[0];
    float (*gain)[EKF_Q_MEAS_DIM] = ekf->workspace.correct_reduced_k;
    float *corrected_state = ekf->workspace.corrected_state;
    float (*joseph_factor)[EKF_Q_STATE_DIM] =
        ekf->workspace.correct_joseph_factor;
    float (*factor_covariance)[EKF_Q_STATE_DIM] =
        ekf->workspace.correct_factor_covariance;
    float (*corrected_covariance)[EKF_Q_STATE_DIM] =
        ekf->workspace.corrected_covariance;

    memset(h, 0, sizeof(ekf->workspace.correct_reduced_h[0]));
    memset(gain, 0, sizeof(ekf->workspace.correct_reduced_k));
    memset(joseph_factor, 0,
           sizeof(ekf->workspace.correct_joseph_factor));
    memset(factor_covariance, 0,
           sizeof(ekf->workspace.correct_factor_covariance));
    memset(corrected_covariance, 0,
           sizeof(ekf->workspace.corrected_covariance));

    const float numerator_derivative[EKF_Q_QUAT_DIM] = {
        2.0f * z,
        2.0f * y,
        2.0f * x,
        2.0f * w,
    };
    const float denominator_derivative[EKF_Q_QUAT_DIM] = {
        0.0f,
        0.0f,
        -4.0f * y,
        -4.0f * z,
    };
    for (int state = 0; state < EKF_Q_QUAT_DIM; state++)
    {
        h[state] =
            (yaw_denominator * numerator_derivative[state] -
             yaw_numerator * denominator_derivative[state]) /
            yaw_gradient_denominator;
    }

    const float measurement_variance =
        measurement_noise_std_rad * measurement_noise_std_rad;
    float innovation_variance = measurement_variance;
    for (int row = 0; row < EKF_Q_STATE_DIM; row++)
    {
        for (int col = 0; col < EKF_Q_STATE_DIM; col++)
        {
            innovation_variance += h[row] * ekf->P[row][col] * h[col];
        }
    }
    if (!isfinite(innovation_variance) ||
        innovation_variance <= ekf->config.numerical_epsilon)
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    for (int state = 0; state < EKF_Q_STATE_DIM; state++)
    {
        float covariance_times_h = 0.0f;
        for (int col = 0; col < EKF_Q_STATE_DIM; col++)
        {
            covariance_times_h += ekf->P[state][col] * h[col];
        }
        gain[state][0] = covariance_times_h / innovation_variance;
    }

    const float predicted_yaw = atan2f(yaw_numerator, yaw_denominator);
    const float innovation =
        ekf_q_wrap_angle(measured_yaw_rad - predicted_yaw);
    memcpy(corrected_state, ekf->x,
           sizeof(ekf->workspace.corrected_state));
    for (int state = 0; state < EKF_Q_STATE_DIM; state++)
    {
        corrected_state[state] += gain[state][0] * innovation;
    }

    for (int row = 0; row < EKF_Q_STATE_DIM; row++)
    {
        for (int col = 0; col < EKF_Q_STATE_DIM; col++)
        {
            joseph_factor[row][col] =
                (row == col ? 1.0f : 0.0f) - gain[row][0] * h[col];
        }
    }
    for (int row = 0; row < EKF_Q_STATE_DIM; row++)
    {
        for (int col = 0; col < EKF_Q_STATE_DIM; col++)
        {
            for (int inner = 0; inner < EKF_Q_STATE_DIM; inner++)
            {
                factor_covariance[row][col] +=
                    joseph_factor[row][inner] * ekf->P[inner][col];
            }
        }
    }
    for (int row = 0; row < EKF_Q_STATE_DIM; row++)
    {
        for (int col = 0; col < EKF_Q_STATE_DIM; col++)
        {
            for (int inner = 0; inner < EKF_Q_STATE_DIM; inner++)
            {
                corrected_covariance[row][col] +=
                    factor_covariance[row][inner] *
                    joseph_factor[col][inner];
            }
            corrected_covariance[row][col] +=
                gain[row][0] * measurement_variance * gain[col][0];
        }
    }

    if (!ekf_q_normalize_candidate(corrected_state, corrected_covariance,
                                   ekf->config.numerical_epsilon,
                                   &ekf->workspace) ||
        !ekf_q_validate_covariance(corrected_covariance,
                                   ekf->config.numerical_epsilon))
    {
        ekf->diagnostics.numerical_error_count++;
        return ESP_ERR_INVALID_RESPONSE;
    }

    memcpy(ekf->x, corrected_state,
           sizeof(ekf->workspace.corrected_state));
    memcpy(ekf->P, corrected_covariance,
           sizeof(ekf->workspace.corrected_covariance));
    ekf->diagnostics.last_yaw_innovation_rad = innovation;
    ekf->diagnostics.yaw_correction_count++;
    ekf_q_refresh_diagnostics(ekf);
    return ESP_OK;
}

void ekf_q_get_quaternion(const ekf_q_t *ekf, float quaternion_wxyz[4])
{
    if (ekf == NULL || quaternion_wxyz == NULL)
    {
        return;
    }
    memcpy(quaternion_wxyz, ekf->x, sizeof(float) * 4);
}

void ekf_q_get_euler(const ekf_q_t *ekf,
                     float *roll_rad,
                     float *pitch_rad,
                     float *yaw_rad)
{
    if (ekf == NULL)
    {
        return;
    }

    float w = ekf->x[EKF_Q_QW];
    float x = ekf->x[EKF_Q_QX];
    float y = ekf->x[EKF_Q_QY];
    float z = ekf->x[EKF_Q_QZ];
    if (roll_rad != NULL)
    {
        *roll_rad = atan2f(2.0f * (w * x + y * z),
                           1.0f - 2.0f * (x * x + y * y));
    }
    if (pitch_rad != NULL)
    {
        float sine_pitch = 2.0f * (w * y - z * x);
        sine_pitch = fmaxf(-1.0f, fminf(1.0f, sine_pitch));
        *pitch_rad = asinf(sine_pitch);
    }
    if (yaw_rad != NULL)
    {
        *yaw_rad = atan2f(2.0f * (w * z + x * y),
                          1.0f - 2.0f * (y * y + z * z));
    }
}

void ekf_q_get_rotation_body_to_navigation(const ekf_q_t *ekf,
                                           float rotation_bn[3][3])
{
    if (ekf == NULL || rotation_bn == NULL)
    {
        return;
    }

    float w = ekf->x[EKF_Q_QW];
    float x = ekf->x[EKF_Q_QX];
    float y = ekf->x[EKF_Q_QY];
    float z = ekf->x[EKF_Q_QZ];
    rotation_bn[0][0] = 1.0f - 2.0f * (y * y + z * z);
    rotation_bn[0][1] = 2.0f * (x * y - w * z);
    rotation_bn[0][2] = 2.0f * (x * z + w * y);
    rotation_bn[1][0] = 2.0f * (x * y + w * z);
    rotation_bn[1][1] = 1.0f - 2.0f * (x * x + z * z);
    rotation_bn[1][2] = 2.0f * (y * z - w * x);
    rotation_bn[2][0] = 2.0f * (x * z - w * y);
    rotation_bn[2][1] = 2.0f * (y * z + w * x);
    rotation_bn[2][2] = 1.0f - 2.0f * (x * x + y * y);
}

esp_err_t ekf_q_rotate_accel_to_navigation(
    const ekf_q_t *ekf,
    const float specific_force_body_m_s2[3],
    float specific_force_nav_m_s2[3])
{
    if (ekf == NULL || !ekf->diagnostics.initialized ||
        !ekf_q_is_finite_vector(specific_force_body_m_s2, 3) ||
        specific_force_nav_m_s2 == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    float rotation[3][3];
    ekf_q_get_rotation_body_to_navigation(ekf, rotation);
    for (int row = 0; row < 3; row++)
    {
        specific_force_nav_m_s2[row] = 0.0f;
        for (int col = 0; col < 3; col++)
        {
            specific_force_nav_m_s2[row] += rotation[row][col] *
                                            specific_force_body_m_s2[col];
        }
    }
    return ESP_OK;
}

esp_err_t ekf_q_get_linear_accel_navigation(
    const ekf_q_t *ekf,
    const float specific_force_body_m_s2[3],
    float linear_accel_nav_m_s2[3])
{
    if (ekf == NULL || linear_accel_nav_m_s2 == NULL ||
        !ekf_q_is_finite_vector(specific_force_body_m_s2, 3))
    {
        return ESP_ERR_INVALID_ARG;
    }

    float unbiased_body[3] = {
        specific_force_body_m_s2[0] - ekf->x[EKF_Q_BAX],
        specific_force_body_m_s2[1] - ekf->x[EKF_Q_BAY],
        specific_force_body_m_s2[2] - ekf->x[EKF_Q_BAZ],
    };
    esp_err_t err = ekf_q_rotate_accel_to_navigation(ekf,
                                                     unbiased_body,
                                                     linear_accel_nav_m_s2);
    if (err != ESP_OK)
    {
        return err;
    }

    /* a_nav = f_nav + g_nav, con g_nav=[0,0,+g] en NED. */
    linear_accel_nav_m_s2[2] += ekf->config.gravity_m_s2;
    return ESP_OK;
}

void ekf_q_get_diagnostics(const ekf_q_t *ekf,
                           ekf_q_diagnostics_t *diagnostics)
{
    if (ekf == NULL || diagnostics == NULL)
    {
        return;
    }
    *diagnostics = ekf->diagnostics;
    if (diagnostics->timed_update_count == 0)
    {
        diagnostics->min_total_us = 0;
    }
}
