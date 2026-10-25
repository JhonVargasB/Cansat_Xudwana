#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define EKF_Q_STATE_DIM 7
#define EKF_Q_MEAS_DIM 3
#define EKF_Q_QUAT_DIM 4

#define EKF_Q_QW 0
#define EKF_Q_QX 1
#define EKF_Q_QY 2
#define EKF_Q_QZ 3
#define EKF_Q_BAX 4
#define EKF_Q_BAY 5
#define EKF_Q_BAZ 6

    /*
     * Convenciones:
     * - q = [w, x, y, z].
     * - q rota vectores desde body hacia navigation (NED).
     * - body y navigation son derechos: X frente/norte, Y derecha/este, Z abajo.
     * - el acelerometro entrega fuerza especifica; quieto y nivelado: [0, 0, -g].
     */
    typedef struct
    {
        float gravity_m_s2;
        float gyro_noise_std_rad_s;
        float accel_noise_std_m_s2;
        float accel_bias_process_noise_std_m_s2_sqrt_s;
        float accel_bias_alpha_s_inv;
        float initial_quaternion_variance;
        float initial_accel_bias_variance;
        float innovation_threshold_m_s2;
        float min_accel_norm_m_s2;
        float min_dt_s;
        float max_dt_s;
        float numerical_epsilon;
    } ekf_q_config_t;

    typedef struct
    {
        float quaternion[4];
        float roll_rad;
        float pitch_rad;
        float yaw_rad;
        float accel_bias_m_s2[3];
        float gyro_bias_rad_s[3];
        float innovation_m_s2[3];
        float last_yaw_innovation_rad;
        bool accel_axis_accepted[3];
        float accel_norm_m_s2;
        float quaternion_norm;
        float covariance_diagonal[EKF_Q_STATE_DIM];
        float last_dt_s;

        uint64_t prediction_count;
        uint64_t correction_count;
        uint64_t accel_rejection_count;
        uint64_t yaw_correction_count;
        uint64_t numerical_error_count;

        uint32_t last_predict_us;
        uint32_t last_correct_us;
        uint32_t last_total_us;
        uint32_t min_total_us;
        uint32_t max_total_us;
        uint64_t total_time_us;
        uint64_t timed_update_count;

        bool initialized;
    } ekf_q_diagnostics_t;

    /*
     * Memoria de trabajo persistente. Mantenerla dentro de cada instancia hace al
     * filtro reentrante entre instancias y evita apilar matrices 7x7 a 100 Hz.
     */
    typedef struct
    {
        float normalize_transform[EKF_Q_STATE_DIM][EKF_Q_STATE_DIM];
        float normalize_temporary[EKF_Q_STATE_DIM][EKF_Q_STATE_DIM];
        float normalize_covariance[EKF_Q_STATE_DIM][EKF_Q_STATE_DIM];

        float predict_transition_covariance[EKF_Q_STATE_DIM][EKF_Q_STATE_DIM];
        float predict_covariance[EKF_Q_STATE_DIM][EKF_Q_STATE_DIM];

        float correct_reduced_h[EKF_Q_MEAS_DIM][EKF_Q_STATE_DIM];
        float correct_reduced_nu[EKF_Q_MEAS_DIM];
        float correct_pht[EKF_Q_STATE_DIM][EKF_Q_MEAS_DIM];
        float correct_reduced_s[EKF_Q_MEAS_DIM][EKF_Q_MEAS_DIM];
        float correct_reduced_k[EKF_Q_STATE_DIM][EKF_Q_MEAS_DIM];
        float corrected_state[EKF_Q_STATE_DIM];
        float correct_joseph_factor[EKF_Q_STATE_DIM][EKF_Q_STATE_DIM];
        float correct_factor_covariance[EKF_Q_STATE_DIM][EKF_Q_STATE_DIM];
        float corrected_covariance[EKF_Q_STATE_DIM][EKF_Q_STATE_DIM];
    } ekf_q_workspace_t;

    typedef struct
    {
        /* Parametros constantes del modelo y de los sensores. */
        ekf_q_config_t config;

        /*
         * Estado estimado:
         * x = [qw, qx, qy, qz, bax, bay, baz].
         * Los indices publicos EKF_Q_QW...EKF_Q_BAZ evitan numeros magicos.
         */
        float x[EKF_Q_STATE_DIM];

        /* Matrices que describen completamente cada ciclo del EKF. */
        /* P: covarianza del estado estimado. */
        float P[EKF_Q_STATE_DIM][EKF_Q_STATE_DIM];
        /* F: Jacobiano del modelo de transicion de estado. */
        float F[EKF_Q_STATE_DIM][EKF_Q_STATE_DIM];
        /* Q: covarianza del ruido de proceso. */
        float Q[EKF_Q_STATE_DIM][EKF_Q_STATE_DIM];
        /* H: Jacobiano del modelo de medicion del acelerometro. */
        float H[EKF_Q_MEAS_DIM][EKF_Q_STATE_DIM];
        /* R: covarianza del ruido de medicion del acelerometro. */
        float R[EKF_Q_MEAS_DIM][EKF_Q_MEAS_DIM];
        /* K: ganancia de Kalman calculada en la ultima correccion. */
        float K[EKF_Q_STATE_DIM][EKF_Q_MEAS_DIM];
        /* S: covarianza de la innovacion. */
        float S[EKF_Q_MEAS_DIM][EKF_Q_MEAS_DIM];
        /* nu: innovacion medida - predicha. */
        float nu[EKF_Q_MEAS_DIM];

        /* Resultados observables, contadores y memoria temporal por instancia. */
        ekf_q_diagnostics_t diagnostics;
        ekf_q_workspace_t workspace;
    } ekf_q_t;

#define EKF_Q_DEFAULT_CONFIG()                             \
    {                                                      \
        .gravity_m_s2 = 9.80665f,                          \
        .gyro_noise_std_rad_s = 0.0174532925f,             \
        .accel_noise_std_m_s2 = 0.0980665f,                \
        .accel_bias_process_noise_std_m_s2_sqrt_s = 0.01f, \
        .accel_bias_alpha_s_inv = 0.0f,                    \
        .initial_quaternion_variance = 0.0025f,            \
        .initial_accel_bias_variance = 0.25f,              \
        .innovation_threshold_m_s2 = 1.4709975f,           \
        .min_accel_norm_m_s2 = 1.96133f,                   \
        .min_dt_s = 0.00001f,                              \
        .max_dt_s = 0.1f,                                  \
        .numerical_epsilon = 1.0e-8f,                      \
    }

    esp_err_t ekf_q_init(ekf_q_t *ekf, const ekf_q_config_t *config);

    esp_err_t ekf_q_set_initial_attitude(ekf_q_t *ekf,
                                         const float accel_body_m_s2[3]);

    void ekf_q_set_gyro_bias(ekf_q_t *ekf,
                             const float gyro_bias_body_rad_s[3]);

    esp_err_t ekf_q_predict(ekf_q_t *ekf,
                            const float gyro_body_rad_s[3],
                            float dt_s);

    esp_err_t ekf_q_correct_accel(ekf_q_t *ekf,
                                  const float accel_body_m_s2[3]);

    /**
     * @brief Corrige solamente el yaw con una observacion absoluta.
     *
     * La medicion y el estado usan radianes en [-pi, pi]. La desviacion estandar
     * representa la incertidumbre efectiva del heading magnetico.
     */
    esp_err_t ekf_q_correct_yaw(ekf_q_t *ekf,
                                float measured_yaw_rad,
                                float measurement_noise_std_rad);

    esp_err_t ekf_q_update(ekf_q_t *ekf,
                           const float gyro_body_rad_s[3],
                           const float accel_body_m_s2[3],
                           float dt_s);

    void ekf_q_get_quaternion(const ekf_q_t *ekf, float quaternion_wxyz[4]);

    void ekf_q_get_euler(const ekf_q_t *ekf,
                         float *roll_rad,
                         float *pitch_rad,
                         float *yaw_rad);

    void ekf_q_get_rotation_body_to_navigation(const ekf_q_t *ekf,
                                               float rotation_bn[3][3]);

    esp_err_t ekf_q_rotate_accel_to_navigation(
        const ekf_q_t *ekf,
        const float specific_force_body_m_s2[3],
        float specific_force_nav_m_s2[3]);

    esp_err_t ekf_q_get_linear_accel_navigation(
        const ekf_q_t *ekf,
        const float specific_force_body_m_s2[3],
        float linear_accel_nav_m_s2[3]);

    void ekf_q_get_diagnostics(const ekf_q_t *ekf,
                               ekf_q_diagnostics_t *diagnostics);

#ifdef __cplusplus
}
#endif

