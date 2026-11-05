// Librerias de ANCI C
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
// Librerias de IDF
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"

// Librerias propias

#include "shared.h"

#include "nrf24_command_protocol.h"
#include "buzzer.h"
#include "nrf24l01.h"
#include "sd_lector.h"
#include "neo6m.h"
#include "bmp180.h"
#include "mpu6050.h"
#include "qmc5883p.h"
#include "ekq_q.h"
#include "altitude_kalman.h"

_Static_assert(EKF_LOG_STATE_DIM == EKF_Q_STATE_DIM,
               "El registro EKF debe coincidir con la dimension del filtro");

static const char *TAG = "main3";

/* state: bit0=grabando, bit1=calibrando. REC_OFF cierra los CSV,
 * pero conserva la tarjeta montada para volver a usar REC_ON.
 */
#define COMMAND_CALIBRATING BIT1
#define APP_INITIALIZING BIT2
#define COMMAND_QUEUE_LENGTH 1U
#define COMMAND_RESULTS_LENGTH 8U
#define COMMAND_RX_BATCH_SIZE 3U
#define SD_CLOSE_BUZZER_HZ 1000U
#define SD_CLOSE_BUZZER_MS 500U
#define SD_FILTER_LOGGING_READY BIT3

/* Telemetria prioritaria; el registro del filtro es un servicio opcional. */
static const struct {
    uint32_t telemetry_queue_length;
    uint32_t filter_batch_size;
    uint32_t telemetry_pause_level;
    int64_t filter_budget_us;
    uint64_t telemetry_reserve_bytes;
} s_sd_policy = {
    .telemetry_queue_length = 32U,
    .filter_batch_size = 2U,
    .telemetry_pause_level = 4U,
    .filter_budget_us = INT64_C(2000),
    .telemetry_reserve_bytes = UINT64_C(64) * 1024U * 1024U,
};

/* Parametros de esta prueba; no alteran main2 ni el protocolo de radio. */
static const struct {
    float stationary_gyro_rad_s;
    float stationary_accel_tolerance_m_s2;
    float stationary_baro_speed_m_s;
    float gravity_error_limit_m_s2;
    float moving_accel_noise_m_s2;
    float stationary_accel_noise_m_s2;
    float barometer_noise_m;
    float moving_nis_threshold;
    float stationary_nis_threshold;
    float baro_height_tau_s;
    float baro_velocity_tau_s;
    float max_baro_speed_m_s;
    float baro_jump_margin_m;
    float bias_tau_s;
    float max_vertical_bias_m_s2;
    int64_t stationary_confirmation_us;
    uint32_t recovery_rejections;
    uint32_t coherent_baro_samples;
    uint32_t settling_corrections;
} s_test_config = {
    .stationary_gyro_rad_s = 0.0174532925f,
    .stationary_accel_tolerance_m_s2 = 0.35f,
    .stationary_baro_speed_m_s = 0.35f,
    .gravity_error_limit_m_s2 = 0.70f,
    .moving_accel_noise_m_s2 = 3.0f,
    .stationary_accel_noise_m_s2 = 0.25f,
    .barometer_noise_m = 1.0f,
    .moving_nis_threshold = 25.0f,
    .stationary_nis_threshold = 9.0f,
    .baro_height_tau_s = 0.30f,
    .baro_velocity_tau_s = 0.75f,
    .max_baro_speed_m_s = 250.0f,
    .baro_jump_margin_m = 3.0f,
    .bias_tau_s = 10.0f,
    .max_vertical_bias_m_s2 = 0.75f,
    .stationary_confirmation_us = INT64_C(1000000),
    .recovery_rejections = 10U,
    .coherent_baro_samples = 5U,
    .settling_corrections = 3U,
};

/* La compilacion actual usa FATFS sin LFN: nombres 8.3, esquemas propios. */
static const char s_test_telemetry_path[] = "/sdcard/TELEM3.CSV";
static const char s_test_ekf_path[] = "/sdcard/EKF3.CSV";
static const char s_filter_sd_header[] =
    "timestamp_us,sequence,dt_s,ax_body_m_s2,ay_body_m_s2,az_body_m_s2,"
    "gx_body_rad_s,gy_body_rad_s,gz_body_rad_s,qw,qx,qy,qz,"
    "baro_timestamp_us,baro_relative_m,height_m,vertical_velocity_m_s,"
    "a_up_m_s2,a_used_m_s2,vertical_bias_m_s2,nis,accepted_axes,flags,"
    "dropped_records,recoveries";
static const char s_test_nav_columns[] =
    "baro_relative_m,height_m,vertical_velocity_m_s,a_up_m_s2,a_used_m_s2,"
    "vertical_bias_m_s2,nis,baro_accepted,rejections,recoveries,stationary,"
    "attitude_valid,vertical_valid,baro_age_ms,gravity_error_m_s2,attitude_resets";

typedef struct {
    nrf24_command_packet_t command;
    uint8_t result;
    bool completed;
} command_result_t;

typedef struct {
    nrf24_command_packet_t command;
    bool valid;
    bool pending;
    uint8_t result;
    uint8_t state;
} command_history_t;

typedef struct {
    FILE *telemetry;
    FILE *ekf;
    esp_err_t error;
    uint32_t telemetry_records;
    uint32_t ekf_records;
    uint32_t telemetry_drops;
    uint32_t ekf_drops;
} sd_logger_t;

typedef struct {
    bool anchored;
    bool pending;
    barometer_sample_t baro;
    int64_t last_consumed_us;
    int64_t last_accepted_us;
    int64_t last_seen_us;
    int64_t stationary_since_us;
    float last_height_m;
    float smoothed_height_m;
    float baro_velocity_m_s;
    float accel_bias_m_s2;
    float last_nis;
    uint32_t coherent_samples;
    uint32_t accepted_since_reset;
    uint32_t consecutive_rejections;
    uint32_t recoveries;
    bool stationary;
    bool last_baro_accepted;
} vertical_navigation_state_t;

typedef struct {
    uint32_t resets;
    float gravity_error_m_s2;
} attitude_supervision_t;

typedef struct {
    float baro_relative_m;
    float accel_up_m_s2;
    float accel_used_m_s2;
    float vertical_bias_m_s2;
    float nis;
    float gravity_error_m_s2;
    int64_t baro_age_us;
    uint32_t rejections;
    uint32_t recoveries;
    uint32_t attitude_resets;
    bool baro_accepted;
    bool stationary;
} navigation_diagnostics_t;

/* Mensaje local de las colas; telemetry_data_t sigue ocupando 32 bytes. */
typedef struct {
    ekf_log_record_t ekf;
    navigation_diagnostics_t navigation;
} filter_test_record_t;

/* Solo lo necesario para comparar filtros: entradas IMU, q y vertical.
 * flags: bit0 actitud valida, bit1 vertical valida, bit2 reposo,
 * bit3 barometro aceptado, bit4 MPU leido, bit5 inicializando, bit6 error.
 */
typedef struct {
    int64_t timestamp_us;
    int64_t baro_timestamp_us;
    uint32_t sequence;
    uint32_t dropped_records;
    uint32_t recoveries;
    float dt_s;
    float accel_body_m_s2[3];
    float gyro_body_rad_s[3];
    float quaternion[4];
    float baro_relative_m;
    float height_m;
    float velocity_m_s;
    float accel_up_m_s2;
    float accel_used_m_s2;
    float bias_m_s2;
    float nis;
    uint8_t accepted_axes;
    uint8_t flags;
} filter_storage_record_t;

typedef struct {
    int64_t next_sample_us;
    int64_t last_correction_us;
    float field_norm_ut;
    float heading_rad;
    uint32_t rejected_samples;
    esp_err_t last_error;
} magnetic_yaw_state_t;

/* Copia mas reciente para radio; el timestamp no forma parte del payload. */
typedef struct {
    int64_t timestamp_us;
    telemetry_data_t data;
} radio_sample_t;

static sd_logger_t s_logger;
static vertical_navigation_state_t s_vertical;
static attitude_supervision_t s_attitude_health;
static navigation_diagnostics_t s_navigation_diagnostics;
static magnetic_yaw_state_t s_magnetic_yaw;
static buzzer_t s_buzzer = {0};
static bool s_buzzer_ready;
static QueueHandle_t Queue_SDCommands;
static QueueHandle_t Queue_Calibration;
static QueueHandle_t Queue_CommandResults;
static SemaphoreHandle_t Mutex_Logging;
static TaskHandle_t s_radio_task;

// Identificadores de haradware
static i2c_master_bus_handle_t bus_handle;
/* Se fija antes de crear tareas y se conserva durante toda la ejecucion. */
static double s_baro_reference_altitude_m;

// Bus I2C1 compartido por BMP180, MPU6050 y QMC5883P (bypass).
static const i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_1,
    .scl_io_num = PIN_SCLK1_I2C,
    .sda_io_num = PIN_SDA1_I2C,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = false,
};

neo6m_t neo6m;
sd_card_t sd_card;
bmp180_t bmp180;
mpu6050_t mpu6050;
qmc5883p_t qmc5883p;
nrf24l01_t nrf24l01;
ekf_q_t ekf_q;
altitude_kalman_t altitude_kalman;

neo6m_config_t neo6mcfg = NEO6M_DEFAULT_CONFIG();
bmp180_config_t bmp180cfg = BMP180_DEFAULT_CONFIG();
mpu6050_config_t mpu6050cfg = MPU6050_DEFAULT_CONFIG();
qmc5883p_config_t qmc5883pcfg = QMC5883P_DEFAULT_CONFIG();
nrf24l01_config_t nrf24l01cfg = NRF24L01_TX_DEFAULT_CONFIG();
sd_config_t sd_config = SD_DEFAULT_CONFIG();
ekf_q_config_t ekf_qcfg = EKF_Q_DEFAULT_CONFIG();
altitude_kalman_config_t altitude_kalmancfg = ALTITUDE_KALMAN_DEFAULT_CONFIG();

// Identificadores de recursos RTOS
QueueHandle_t Queue_GPS = NULL;
QueueHandle_t Queue_data = NULL;
QueueHandle_t Queue_radio = NULL;
QueueHandle_t Queue_Attitude = NULL;
QueueHandle_t Queue_EkfLog = NULL;
QueueHandle_t Queue_EkfSerial = NULL;
QueueHandle_t Queue_Baro = NULL;
static EventGroupHandle_t EventGroup_status = NULL;

static const uint8_t s_radio_address[NRF24_MAX_ADDRESS_WIDTH] =
    NRF24_TELEMETRY_ADDRESS;
static const float s_mpu_to_body[3][3] = MPU_TO_BODY_ROTATION;
static const float s_qmc_to_mpu[3][3] = QMC_TO_MPU_ROTATION;
static const float s_qmc_hard_iron_ut[3] = QMC_HARD_IRON_OFFSET_UT;
static const float s_qmc_soft_iron[3][3] = QMC_SOFT_IRON_MATRIX;

// Prototipo de Tareas
static void vTaskReadGPS(void *pVparameters);
static void vTaskReadData(void *pVparameters);
static void vTaskWriteSD(void *pVparameters);
static void vTaskTransmitData(void *pVparameters);
static void vTaskAttitude(void *pVparameters);

// Prototipo de funciones de aplciacion
static esp_err_t init_nrf24(void);
static void init_rtos_resources(void);
static uint32_t radio_period_ms(EventBits_t state);
static telemetry_data_t prepare_radio_telemetry(const radio_sample_t *sample,
                                                bool available, EventBits_t state,
                                                int64_t now_us, uint32_t sequence);
static esp_err_t init_magnetometer(void);
static bool rotation_is_valid(const float rotation[3][3]);
static esp_err_t calibrate_accel_for_mounting(void);
static esp_err_t correct_yaw_with_magnetometer(void);
static esp_err_t apply_magnetic_yaw_correction(float heading_rad);
static esp_err_t update_imu_attitude(const float gyro[3], const float accel[3], float dt_s);
static void update_telemetry_attitude(telemetry_data_t *data, const attitude_data_t *attitude,
                                    int64_t now_us);
static void update_magnetic_yaw(void);
static esp_err_t read_mpu_body(float accel_body[3], float gyro_body[3], int64_t *timestamp_us);
static bool attitude_sample_is_still(const float accel[3], const float gyro[3]);
static esp_err_t initialize_attitude(void);
static esp_err_t initialize_relative_height(void);
static FILE *open_csv_append(const char *path, const char *header, char *buffer, size_t buffer_size);
static void process_ack_command(command_history_t *history);
static const char *command_name(uint8_t command);
static uint8_t command_state(void);
static void complete_command(const nrf24_command_packet_t *command, esp_err_t error);
static void send_command_response(const command_result_t *result, uint8_t state);
static esp_err_t recalibrate_attitude(void);
static bool enqueue_sd_record(QueueHandle_t queue, const void *record);
static void set_logging_enabled(bool enabled);
static esp_err_t start_sd_logging(void);
static esp_err_t stop_sd_logging(void);
static esp_err_t write_sd_batch(void);
static esp_err_t flush_sd_logs(void);
static esp_err_t close_log_file(FILE **file);
static esp_err_t sound_sd_closed(void);
static int32_t scale_to_i32(double value, double scale);
static int16_t scale_to_i16(double value, double scale);
static void log_nrf24_telemetry(const telemetry_data_t *sample, esp_err_t result);
static filter_storage_record_t compact_filter_record(const filter_test_record_t *record);
static int write_ekf_csv(FILE *file, const char *prefix, const filter_storage_record_t *record);
static esp_err_t write_telemetry_csv(const telemetry_log_record_t *record);
static void disable_filter_logging(uint32_t discarded_records);
static void reserve_telemetry_space(void);
static int write_navigation_csv(FILE *file, const filter_test_record_t *record);
static void print_navigation_test(const filter_test_record_t *record);
static float lowpass_update(float previous, float input, float dt_s, float tau_s);
static float vector_norm(const float vector[3]);
static float gravity_error(const float accel_body[3]);
static bool supervise_attitude(const ekf_log_record_t *record);
static bool barometer_is_stationary(int64_t now_us);
static bool update_barometer_tracking(const barometer_sample_t *sample);
static esp_err_t anchor_vertical_filter(int64_t now_us);
static void update_vertical_navigation(ekf_log_record_t *record);

// Prototipo de funciones de aplciacion
void app_main(void)
{
    // Configuracion de dispositivos y recursos antes de habilitar la radio.
    nrf24l01.config = nrf24l01cfg;
    sd_card.config = sd_config;
    neo6m.config = neo6mcfg;
    mpu6050.config = mpu6050cfg;
    /* Adquisicion a 100 Hz y banda menor que Nyquist para esta prueba. */
    mpu6050.config.smpltr_div = 9U;
    mpu6050.config.dlpf_cfg = BWA_44_BWG_42;
    bmp180.config = bmp180cfg;

    init_rtos_resources();
    xEventGroupSetBits(EventGroup_status, APP_INITIALIZING);
    ESP_ERROR_CHECK(init_nrf24());
    BaseType_t task_created = xTaskCreate(vTaskTransmitData, "nrf24_tx_task",
                                         1024 * 5, NULL, 3, &s_radio_task);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_LOGI(TAG, "Radio activa durante arranque: envios cada %lu ms; comandos pendientes hasta estar listo",
             (unsigned long)NRF24_IDLE_TX_PERIOD_MS);

    // Sensores y calibracion: la radio ya puede transmitir y recibir ACK.
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    bmp180.bus_handle = bus_handle;
    mpu6050.bus_handle = bus_handle;
    ESP_ERROR_CHECK(neo6m_init(&neo6m));
    esp_err_t sd_error = sd_init(&sd_card);
    if (sd_error != ESP_OK)
        ESP_LOGW(TAG, "SD no disponible: %s; REC_ON reintentara el montaje",
                 esp_err_to_name(sd_error));
    const buzzer_config_t buzzer_config = BUZZER_DEFAULT_CONFIG();
    esp_err_t buzzer_error = buzzer_init(&s_buzzer, &buzzer_config);
    s_buzzer_ready = buzzer_error == ESP_OK;
    if (!s_buzzer_ready)
        ESP_LOGW(TAG, "Buzzer no disponible: %s", esp_err_to_name(buzzer_error));
    ESP_ERROR_CHECK(bmp180_init(&bmp180));
    ESP_ERROR_CHECK(mpu6050_init(&mpu6050));
    ESP_ERROR_CHECK(rotation_is_valid(s_mpu_to_body) ? ESP_OK : ESP_ERR_INVALID_ARG);
    if (YAW_MAGNETOMETER_ENABLED)
    {
        const esp_err_t magnetic_error = init_magnetometer();
        if (magnetic_error != ESP_OK)
            ESP_LOGW(TAG, "QMC no disponible: %s; yaw relativo por giroscopio",
                     esp_err_to_name(magnetic_error));
    }

    // Inicio de Calibracion de el acelerometro y del giroscopio
    ESP_LOGI(TAG, "Mantener el CanSat inmovil y nivelado: calibrando MPU6050");
    ESP_ERROR_CHECK(calibrate_accel_for_mounting());
    ESP_ERROR_CHECK(mpu6050_calibrate_gyro(&mpu6050));
    ESP_LOGI(TAG, "Calibracion del MPU6050 terminada");

    // Inicializacion del EKF
    ekf_qcfg.accel_bias_process_noise_std_m_s2_sqrt_s = ATTITUDE_ACCEL_BIAS_PROCESS_NOISE_STD;
    ESP_ERROR_CHECK(ekf_q_init(&ekf_q, &ekf_qcfg));
    ESP_LOGI(TAG, "Mantener inmovil: promediando actitud inicial durante 1 s");
    ESP_ERROR_CHECK(initialize_attitude());
    ESP_LOGI(TAG, "Mantener inmovil: estableciendo altura relativa cero");
    ESP_ERROR_CHECK(initialize_relative_height());
    ESP_ERROR_CHECK(altitude_kalman_init(&altitude_kalman, &altitude_kalmancfg));
    ESP_LOGI(TAG, "Prueba vertical: sigma_baro=%.2f m, sigma_a=%.2f/%.2f m/s2; "
                 "recuperacion tras %lu rechazos coherentes; aceleracion sin reducir",
             (double)s_test_config.barometer_noise_m,
             (double)s_test_config.stationary_accel_noise_m_s2,
             (double)s_test_config.moving_accel_noise_m_s2,
             (unsigned long)s_test_config.recovery_rejections);

    xEventGroupClearBits(EventGroup_status, APP_INITIALIZING);
    ESP_LOGI(TAG, "Sensores listos. REC_ON: SD y radio a %lu ms; REC_OFF: radio a %lu ms",
             (unsigned long)NRF24_RECORDING_TX_PERIOD_MS,
             (unsigned long)NRF24_IDLE_TX_PERIOD_MS);

    // Creacion de Tareas
    task_created = xTaskCreate(vTaskReadGPS, "gps_task", 1024 * 5, NULL, 2, NULL);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    task_created = xTaskCreate(vTaskAttitude, "attitude_task", 1024 * 6, NULL, 4, NULL);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    task_created = xTaskCreate(vTaskReadData, "data_task", 1024 * 6, NULL, 3, NULL);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    task_created = xTaskCreate(vTaskWriteSD, "sd_logger_task", 1024 * 8, NULL, 3, NULL);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

}

static void vTaskAttitude(void *pVparameters)
{
    (void)pVparameters;

    uint32_t sequence = 0U;
    uint32_t dropped_records = 0U;
    uint32_t error_count = 0U;
    uint32_t recovery_samples = 0U;
    float recovery_sum[3] = {0};
    int64_t previous_timestamp_us = 0;
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_status = last_wake;
    const TickType_t period_ticks = pdMS_TO_TICKS(ATTITUDE_PERIOD_MS);
    configASSERT(period_ticks > 0);

    ESP_LOGI(TAG, "Actitud EKF: periodo=%lu ms; correccion QMC=%s cada %lld us",
             (unsigned long)ATTITUDE_PERIOD_MS, qmc5883p.initialized ? "ON" : "OFF",
             (long long)YAW_CORRECTION_PERIOD_US);

    while (1)
    {
        nrf24_command_packet_t calibration_command;
        if (xQueueReceive(Queue_Calibration, &calibration_command, 0) == pdTRUE)
        {
            const esp_err_t calibration_error = recalibrate_attitude();
            memset(&s_magnetic_yaw, 0, sizeof(s_magnetic_yaw));
            previous_timestamp_us = 0;
            recovery_samples = 0U;
            memset(recovery_sum, 0, sizeof(recovery_sum));
            last_wake = xTaskGetTickCount();
            complete_command(&calibration_command, calibration_error);
        }

        ekf_log_record_t record = {
            .sequence = sequence++,
            .accel_body_m_s2 = {NAN, NAN, NAN},
            .gyro_body_rad_s = {NAN, NAN, NAN},
        };
        record.error = read_mpu_body(record.accel_body_m_s2,
                                     record.gyro_body_rad_s,
                                     &record.attitude.timestamp_us);

        if (record.error == ESP_OK)
        {
            record.status |= EKF_LOG_MPU_OK;
            if (!ekf_q.diagnostics.initialized)
            {
                /* Una inicializacion/calibracion incompleta requiere reposo
                 * para arrancar el EKF. Lecturas fallidas y huecos de tiempo
                 * aislados conservan el estado y no entran en esta rama.
                 */
                record.status |= EKF_LOG_INITIALIZING;
                if (attitude_sample_is_still(record.accel_body_m_s2,
                                             record.gyro_body_rad_s))
                {
                    for (int axis = 0; axis < 3; axis++)
                        recovery_sum[axis] += record.accel_body_m_s2[axis];
                    if (++recovery_samples == ATTITUDE_INITIAL_SAMPLES)
                    {
                        for (int axis = 0; axis < 3; axis++)
                            recovery_sum[axis] /= (float)recovery_samples;
                        record.error = ekf_q_set_initial_attitude(&ekf_q,
                                                                  recovery_sum);
                        memset(&s_magnetic_yaw, 0, sizeof(s_magnetic_yaw));
                        recovery_samples = 0U;
                        memset(recovery_sum, 0, sizeof(recovery_sum));
                        if (record.error == ESP_OK)
                        {
                            s_attitude_health.resets++;
                            ESP_LOGI(TAG, "Actitud EKF reinicializada en reposo");
                        }
                    }
                }
                else
                {
                    recovery_samples = 0U;
                    memset(recovery_sum, 0, sizeof(recovery_sum));
                }
                previous_timestamp_us = record.attitude.timestamp_us;
            }
            else if (previous_timestamp_us != 0)
            {
                record.dt_s = (float)(record.attitude.timestamp_us - previous_timestamp_us) * 1.0e-6f;
                previous_timestamp_us = record.attitude.timestamp_us;
                if (record.dt_s < ekf_qcfg.min_dt_s || record.dt_s > ekf_qcfg.max_dt_s)
                {
                    record.status |= EKF_LOG_DT_REJECTED;
                    record.error = ESP_ERR_INVALID_STATE;
                    /* Un hueco de tiempo no borra el cuaternion. Esta muestra
                     * se omite; previous_timestamp_us ya quedo resincronizado.
                     */
                }
                else
                {
                    record.error = update_imu_attitude(record.gyro_body_rad_s,
                                                       record.accel_body_m_s2,
                                                       record.dt_s);
                    record.update_us = ekf_q.diagnostics.last_total_us;
                    if (record.error == ESP_OK)
                    {
                        for (int axis = 0; axis < 3; axis++)
                        {
                            record.innovation_m_s2[axis] =
                                ekf_q.diagnostics.innovation_m_s2[axis];
                            if (ekf_q.diagnostics.accel_axis_accepted[axis])
                                record.accepted_axes |= (uint8_t)(1U << axis);
                        }
                    }
                }
            }
            else
            {
                /* La primera lectura establece el origen de dt. */
                previous_timestamp_us = record.attitude.timestamp_us;
            }

            if (record.error == ESP_OK && ekf_q.diagnostics.initialized)
            {
                /* Solo esta tarea modifica el EKF. Un rechazo magnetico no
                 * invalida la prediccion IMU ni las correcciones de gravedad.
                 */
                const int64_t magnetic_start_us = esp_timer_get_time();
                update_magnetic_yaw();
                record.update_us += (uint32_t)(esp_timer_get_time() - magnetic_start_us);
                ekf_q_get_quaternion(&ekf_q, record.attitude.quaternion);
                ekf_q_get_euler(&ekf_q, &record.attitude.roll_rad,
                                &record.attitude.pitch_rad,
                                &record.attitude.yaw_rad);
                record.error = ekf_q_get_linear_accel_navigation(
                    &ekf_q, record.accel_body_m_s2,
                    record.attitude.linear_accel_nav_m_s2);
                record.attitude.valid = record.error == ESP_OK && supervise_attitude(&record);
                if (record.attitude.valid)
                    record.status |= EKF_LOG_ATTITUDE_VALID;
            }
        }
        else
        {
            recovery_samples = 0U;
            memset(recovery_sum, 0, sizeof(recovery_sum));
        }

        if (record.error != ESP_OK)
        {
            record.status |= EKF_LOG_ERROR;
            error_count++;
            s_attitude_health.gravity_error_m_s2 = NAN;
        }

        /* Incluso los fallos reemplazan la muestra, para no publicar una
         * estimacion antigua como si correspondiese a una lectura nueva.
         */
        update_vertical_navigation(&record);

        ekf_q_diagnostics_t diagnostics;
        ekf_q_get_diagnostics(&ekf_q, &diagnostics);
        memcpy(record.accel_bias_m_s2, diagnostics.accel_bias_m_s2,
               sizeof(record.accel_bias_m_s2));
        memcpy(record.covariance_diagonal, diagnostics.covariance_diagonal,
               sizeof(record.covariance_diagonal));
        record.accel_norm_m_s2 = sqrtf(
            record.accel_body_m_s2[0] * record.accel_body_m_s2[0] + record.accel_body_m_s2[1] * record.accel_body_m_s2[1] + record.accel_body_m_s2[2] * record.accel_body_m_s2[2]);
        record.quaternion_norm = diagnostics.quaternion_norm;
        record.dropped_records = dropped_records;
        /* Copia independiente de SD; nunca bloquear el filtro con printf. */
        const filter_test_record_t test_record = {
            .ekf = record,
            .navigation = s_navigation_diagnostics,
        };
        xQueueOverwrite(Queue_Attitude, &test_record);
        xQueueOverwrite(Queue_EkfSerial, &test_record);

        const filter_storage_record_t storage_record = compact_filter_record(&test_record);
        if (!enqueue_sd_record(Queue_EkfLog, &storage_record))
            dropped_records++;

        const TickType_t now = xTaskGetTickCount();
        if ((now - last_status) >= pdMS_TO_TICKS(SD_STATUS_PERIOD_MS))
        {
            if (qmc5883p.initialized)
            {
                const int64_t age_us = s_magnetic_yaw.last_correction_us > 0
                    ? esp_timer_get_time() - s_magnetic_yaw.last_correction_us : -1;
                ESP_LOGI(TAG, "MAG: referencia=%s campo=%.2f uT rumbo=%.2f deg "
                              "correcciones=%llu rechazos=%lu edad=%lld ms ultimo=%s",
                         age_us >= 0 && age_us <= YAW_MAGNETIC_MAX_AGE_US ? "OK" : "SIN_REFERENCIA",
                         (double)s_magnetic_yaw.field_norm_ut,
                         (double)s_magnetic_yaw.heading_rad * RAD_TO_DEG,
                         (unsigned long long)ekf_q.diagnostics.yaw_correction_count,
                         (unsigned long)s_magnetic_yaw.rejected_samples,
                         (long long)(age_us >= 0 ? age_us / 1000 : -1),
                         esp_err_to_name(s_magnetic_yaw.last_error));
            }
           /* ESP_LOGI(TAG, "EKF: seq=%lu dt=%.4f s roll=%.2f pitch=%.2f yaw=%.2f "
                          "valido=%u errores=%lu descartados_SD=%lu tiempo=%lu us",
                     (unsigned long)record.sequence, (double)record.dt_s,
                     (double)record.attitude.roll_rad * RAD_TO_DEG,
                     (double)record.attitude.pitch_rad * RAD_TO_DEG,
                     (double)record.attitude.yaw_rad * RAD_TO_DEG,
                     (unsigned)record.attitude.valid, (unsigned long)error_count,
                     (unsigned long)dropped_records, (unsigned long)record.update_us);*/
            last_status = now;
        }

        /* No recuperar plazos vencidos ejecutando iteraciones en rafaga. */
        if (xTaskDelayUntil(&last_wake, period_ticks) != pdTRUE)
        {
            last_wake = xTaskGetTickCount();
            vTaskDelay(1);
        }
    }
}

static void vTaskTransmitData(void *pVparameters)
{
    (void)pVparameters;
    command_history_t history = {0};
    uint32_t sequence = 0;
    uint32_t period_ms = 0;
    int64_t next_tx_us = 0;

    while (1)
    {
        command_result_t result;
        if (xQueueReceive(Queue_CommandResults, &result, 0) == pdTRUE)
        {
            uint8_t state = command_state();
            if (result.completed)
            {
                history.pending = false;
                history.result = result.result;
                history.state = state;
            }
            else if (history.valid
                     && history.command.sequence == result.command.sequence
                     && history.command.command == result.command.command
                     && !history.pending)
            {
                state = history.state;
            }
            send_command_response(&result, state);
            process_ack_command(&history);
        }

        const EventBits_t state = xEventGroupGetBits(EventGroup_status);
        const uint32_t requested_period_ms = radio_period_ms(state);
        int64_t now_us = esp_timer_get_time();
        if (requested_period_ms != period_ms)
        {
            period_ms = requested_period_ms;
            next_tx_us = now_us;
            ESP_LOGI(TAG, "Radio: periodo=%lu ms, estado=%s",
                     (unsigned long)period_ms,
                     (state & APP_INITIALIZING) ? "ARRANQUE"
                         : (state & SD_LOGGING_READY) ? "GRABANDO" : "ESPERA");
        }
        if (now_us >= next_tx_us)
        {
            radio_sample_t sample = {0};
            const bool available = xQueuePeek(Queue_radio, &sample, 0) == pdTRUE;
            telemetry_data_t telemetry = prepare_radio_telemetry(
                &sample, available, state, now_us, sequence++);
            const esp_err_t error = nrf24l01_transmit(
                &nrf24l01, &telemetry, sizeof(telemetry), NRF24_TX_TIMEOUT_MS);
            process_ack_command(&history);
            log_nrf24_telemetry(&telemetry, error);
            next_tx_us = now_us + (int64_t)period_ms * 1000;
            now_us = esp_timer_get_time();
            if (next_tx_us <= now_us)
                next_tx_us = now_us + (int64_t)period_ms * 1000;
        }

        /* Consumir el ACK antes de dormir. Todas las operaciones de radio
         * pertenecen a esta tarea; transmit() vuelve a despertarla al enviar.
         */
        if (nrf24l01.mode != NRF24_MODE_POWER_DOWN)
        {
            const esp_err_t error = nrf24l01_power_down(&nrf24l01);
            if (error != ESP_OK)
                ESP_LOGW(TAG, "Radio no pudo entrar en power_down: %s", esp_err_to_name(error));
        }
        const int64_t wait_us = next_tx_us - esp_timer_get_time();
        if (wait_us > 0)
        {
            const int64_t tick_us = (int64_t)portTICK_PERIOD_MS * 1000;
            const TickType_t wait_ticks = (TickType_t)((wait_us + tick_us - 1) / tick_us);
            /* REC_ON/OFF y resultados de comandos despiertan inmediatamente;
             * no consultar una cola cada 10 ms solo para esperar el plazo.
             */
            (void)ulTaskNotifyTake(pdTRUE, wait_ticks);
        }
    }
}

static void vTaskWriteSD(void *pVparameters)
{
    (void)pVparameters;
    TickType_t last_flush = xTaskGetTickCount();
    TickType_t last_status = last_flush;

    while (1)
    {
        nrf24_command_packet_t command;
        if (xQueueReceive(Queue_SDCommands, &command, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            esp_err_t error;
            if (command.command == NRF24_COMMAND_REC_ON)
            {
                error = start_sd_logging();
                last_flush = xTaskGetTickCount();
                if (error != ESP_OK)
                    ESP_LOGW(TAG, "REC_ON no inicio grabacion: %s; radio conserva periodo=%lu ms",
                             esp_err_to_name(error), (unsigned long)NRF24_IDLE_TX_PERIOD_MS);
            }
            else
            {
                error = stop_sd_logging();
                if (error == ESP_OK)
                    error = sound_sd_closed();
            }
            complete_command(&command, error);
        }

        if (s_logger.telemetry != NULL)
        {
            esp_err_t error = write_sd_batch();
            const TickType_t now = xTaskGetTickCount();
            if (error == ESP_OK && now - last_flush >= pdMS_TO_TICKS(SD_FLUSH_PERIOD_MS))
            {
                error = flush_sd_logs();
                last_flush = now;
            }
            if (error != ESP_OK)
            {
                ESP_LOGE(TAG, "Fallo de escritura SD; se cierra el registro");
                s_logger.error = error;
                set_logging_enabled(false);
                (void)close_log_file(&s_logger.telemetry);
                (void)close_log_file(&s_logger.ekf);
            }
            if (now - last_status >= pdMS_TO_TICKS(SD_STATUS_PERIOD_MS))
            {
                if (uxQueueMessagesWaiting(Queue_data) == 0)
                    reserve_telemetry_space();
                uint32_t telemetry_drops, filter_drops;
                xSemaphoreTake(Mutex_Logging, portMAX_DELAY);
                telemetry_drops = s_logger.telemetry_drops;
                filter_drops = s_logger.ekf_drops;
                xSemaphoreGive(Mutex_Logging);
                ESP_LOGI(TAG, "SD: telem=%lu filtro=%lu descartados_telem=%lu descartados_filtro=%lu filtro=%s",
                         (unsigned long)s_logger.telemetry_records, (unsigned long)s_logger.ekf_records,
                         (unsigned long)telemetry_drops, (unsigned long)filter_drops,
                         s_logger.ekf != NULL ? "ON" : "OFF");
                last_status = now;
            }
        }
    }
}

static void vTaskReadData(void *pVparameters)
{
    (void)pVparameters;

    neo6m_data_t latest_gps = {0};
    telemetry_data_t last_angles = {0};
    int32_t last_baro_altitude_cm = 0;
    uint32_t last_baro_pressure_pa = 0;
    int16_t last_baro_temperature_centi_c = 0;
    bool gps_received = false;
    uint32_t sequence = 0U;
    TickType_t last_wake = xTaskGetTickCount();

    printf("NAV_HEADER,timestamp_us,sequence,%s\n", s_test_nav_columns);
    uint32_t last_ekf_sequence = 0U;
    bool ekf_serial_received = false;

    while (1)
    {
        while (xQueueReceive(Queue_GPS, &latest_gps, 0) == pdTRUE)
            gps_received = true;

        telemetry_log_record_t record = {
            .timestamp_us = esp_timer_get_time(),
            .data.sequence = sequence++,
            .data.altitude_cm = last_baro_altitude_cm,
            .data.pressure_pa = last_baro_pressure_pa,
            .data.temperature_centi_c = last_baro_temperature_centi_c,
            .data.yaw_centi_deg = last_angles.yaw_centi_deg,
            .data.roll_centi_deg = last_angles.roll_centi_deg,
            .data.pitch_centi_deg = last_angles.pitch_centi_deg,
            .pressure_raw_pa = NAN,
            .pressure_filtered_pa = NAN,
            .pressure_filter_dt_s = NAN,
            .pressure_filter_alpha = NAN,
        };

        esp_err_t bmp_error = bmp180_read_data(&bmp180);
        if (bmp_error == ESP_OK)
        {
            record.barometer_timestamp_us = bmp180.data.timestamp_us;
            record.pressure_raw_pa = (float)bmp180.data.pressure;
            record.pressure_filtered_pa = bmp180.data.pressure_filtered_pa;
            record.pressure_filter_dt_s = bmp180.data.pressure_filter_dt_s;
            record.pressure_filter_alpha = bmp180.data.pressure_filter_alpha;
            record.barometer_valid = bmp180.data.valid;
            /* El driver calcula esta altura con la presion EMA. La presion
             * original sigue disponible y mantiene su campo de telemetria.
             */
            const double relative_height_m =
                bmp180.data.altitude - s_baro_reference_altitude_m;
            record.data.altitude_cm = scale_to_i32(relative_height_m, TELEMETRY_CENTI_SCALE);
            last_baro_altitude_cm = record.data.altitude_cm;
            record.data.pressure_pa = bmp180.data.pressure > 0.0
                                          ? (uint32_t)lround(bmp180.data.pressure)
                                          : 0U;
            record.data.temperature_centi_c = scale_to_i16(bmp180.data.temperature, TELEMETRY_CENTI_SCALE);
            last_baro_pressure_pa = record.data.pressure_pa;
            last_baro_temperature_centi_c = record.data.temperature_centi_c;
            record.data.status |= TELEMETRY_STATUS_BMP180_OK;
            if (isfinite(relative_height_m))
            {
                const barometer_sample_t baro_sample = {
                    .timestamp_us = bmp180.data.timestamp_us,
                    .height_m = (float)relative_height_m,
                };
                xQueueOverwrite(Queue_Baro, &baro_sample);
            }
        }
        else
        {
            ESP_LOGW(TAG, "BMP180: lectura fallida (%s); telemetria conserva ultima muestra, BMP_OK=0",
                     esp_err_to_name(bmp_error));
        }

        filter_test_record_t navigation_record;
        if ((xEventGroupGetBits(EventGroup_status) & COMMAND_CALIBRATING) == 0
            && xQueuePeek(Queue_Attitude, &navigation_record, 0) == pdTRUE)
        {
            update_telemetry_attitude(&record.data, &navigation_record.ekf.attitude,
                                      esp_timer_get_time());
        }
        /* Ante una muestra perdida conservar solo la ultima orientacion; el
         * bit MPU_OK queda apagado. No inventar un salto a [0,0,0].
         */
        last_angles.yaw_centi_deg = record.data.yaw_centi_deg;
        last_angles.roll_centi_deg = record.data.roll_centi_deg;
        last_angles.pitch_centi_deg = record.data.pitch_centi_deg;

        if (gps_received)
        {
            record.data.latitude_e7 = scale_to_i32(latest_gps.latitude, TELEMETRY_COORDINATE_SCALE);
            record.data.longitude_e7 = scale_to_i32(latest_gps.longitude, TELEMETRY_COORDINATE_SCALE);
            record.data.satellites = latest_gps.satellites_used;
            record.data.status |= TELEMETRY_STATUS_NEO6M_OK;

            if (neo6m_has_fix(&latest_gps))
                record.data.status |= TELEMETRY_STATUS_GPS_FIX;
        }

        if ((xEventGroupGetBits(EventGroup_status) & SD_LOGGING_READY) != 0)
        {
            record.data.status |= TELEMETRY_STATUS_SD_OK;

            if (!enqueue_sd_record(Queue_data, &record))
                ESP_LOGW(TAG, "Cola de telemetria llena; muestra descartada");
        }

        const radio_sample_t radio_sample = {
            .timestamp_us = record.timestamp_us,
            .data = record.data,
        };
        xQueueOverwrite(Queue_radio, &radio_sample);
        filter_test_record_t ekf_record;
        if (xQueuePeek(Queue_EkfSerial, &ekf_record, 0) == pdTRUE
            && (!ekf_serial_received || ekf_record.ekf.sequence != last_ekf_sequence))
        {
            print_navigation_test(&ekf_record);
            last_ekf_sequence = ekf_record.ekf.sequence;
            ekf_serial_received = true;
        }

        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(TELEMETRY_PERIOD_DEFAULT_MS));
    }
}

static void vTaskReadGPS(void *pVparameters)
{
    (void)pVparameters;

    while (1)
    {
        esp_err_t error = neo6m_update(&neo6m, GPS_UPDATE_TIMEOUT_MS);
        if (error == ESP_OK)
        {
            xQueueOverwrite(Queue_GPS, &neo6m.data);
        }
        else if (error != ESP_ERR_TIMEOUT)
        {
            ESP_LOGW(TAG, "Error actualizando GPS: %s", esp_err_to_name(error));
        }
    }
}

// Definciin de funcionres de aplicacion
static void init_rtos_resources(void)
{
    Queue_GPS = xQueueCreate(GPS_QUEUE_LENGTH, sizeof(neo6m_data_t));
    Queue_data = xQueueCreate(s_sd_policy.telemetry_queue_length, sizeof(telemetry_log_record_t));
    Queue_radio = xQueueCreate(RADIO_QUEUE_LENGTH, sizeof(radio_sample_t));
    Queue_Attitude = xQueueCreate(ATTITUDE_QUEUE_LENGTH, sizeof(filter_test_record_t));
    Queue_EkfLog = xQueueCreate(EKF_LOG_QUEUE_LENGTH, sizeof(filter_storage_record_t));
    Queue_EkfSerial = xQueueCreate(EKF_SERIAL_QUEUE_LENGTH, sizeof(filter_test_record_t));
    Queue_Baro = xQueueCreate(BARO_QUEUE_LENGTH, sizeof(barometer_sample_t));
    EventGroup_status = xEventGroupCreate();
    Queue_SDCommands = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(nrf24_command_packet_t));
    Queue_Calibration = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(nrf24_command_packet_t));
    Queue_CommandResults = xQueueCreate(COMMAND_RESULTS_LENGTH, sizeof(command_result_t));
    Mutex_Logging = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(Queue_GPS && Queue_data && Queue_radio && Queue_Attitude
                   && Queue_EkfLog && Queue_EkfSerial && Queue_Baro
                   && EventGroup_status && Queue_SDCommands && Queue_Calibration
                   && Queue_CommandResults && Mutex_Logging ? ESP_OK : ESP_ERR_NO_MEM);
}

static uint32_t radio_period_ms(EventBits_t state)
{
    return (state & SD_LOGGING_READY) != 0 && (state & APP_INITIALIZING) == 0
        ? NRF24_RECORDING_TX_PERIOD_MS : NRF24_IDLE_TX_PERIOD_MS;
}

static telemetry_data_t prepare_radio_telemetry(const radio_sample_t *sample,
                                                bool available, EventBits_t state,
                                                int64_t now_us, uint32_t sequence)
{
    telemetry_data_t telemetry = {0};
    if (available && (state & APP_INITIALIZING) == 0
        && now_us >= sample->timestamp_us
        && now_us - sample->timestamp_us <= NRF24_RADIO_SAMPLE_MAX_AGE_US)
        telemetry = sample->data;
    /* Secuencia de intentos RF independiente de las muestras adquiridas/SD.
     * A 1 Hz no se envian datos antiguos ni una cola acumulada de muestras.
     */
    telemetry.sequence = sequence;
    if ((state & SD_LOGGING_READY) != 0)
        telemetry.status |= TELEMETRY_STATUS_SD_OK;
    else
        telemetry.status &= (uint8_t)~TELEMETRY_STATUS_SD_OK;
    if ((state & (APP_INITIALIZING | COMMAND_CALIBRATING)) != 0)
    {
        telemetry.status &= (uint8_t)~TELEMETRY_STATUS_MPU6050_OK;
        telemetry.roll_centi_deg = telemetry.pitch_centi_deg = 0;
        telemetry.yaw_centi_deg = 0;
        telemetry.vertical_speed_cms = 0;
    }
    return telemetry;
}

static esp_err_t init_nrf24(void)
{
    ESP_LOGI(TAG, "NRF24 SPI inicial: host=%d modo=0 reloj=%lu Hz DMA=%d",
             (int)nrf24l01cfg.spi_host, (unsigned long)nrf24l01cfg.spi_clock_hz,
             (int)nrf24l01cfg.spi_dma_chan);
    ESP_LOGI(TAG, "NRF24 GPIO: MOSI=%d MISO=%d SCLK=%d CSN=%d CE=%d IRQ=%d (-1=NC)",
             (int)nrf24l01cfg.mosi_pin, (int)nrf24l01cfg.miso_pin,
             (int)nrf24l01cfg.sclk_pin, (int)nrf24l01cfg.csn_pin,
             (int)nrf24l01cfg.ce_pin, (int)nrf24l01cfg.irq_pin);
    ESP_LOGI(TAG, "NRF24 canal solicitado=%u (%u MHz), telemetria=%u bytes",
             (unsigned)nrf24l01cfg.channel, 2400U + nrf24l01cfg.channel,
             (unsigned)sizeof(telemetry_data_t));

    esp_err_t error = nrf24l01_init(&nrf24l01, &nrf24l01cfg);
    if (error != ESP_OK)
    {
        ESP_LOGE(TAG, "NRF24 inicializacion SPI/radio fallo: %s", esp_err_to_name(error));
        return error;
    }
    error = nrf24l01_set_tx_address(&nrf24l01, s_radio_address);
    if (error != ESP_OK)
    {
        ESP_LOGE(TAG, "NRF24 no se pudo configurar direccion TX: %s", esp_err_to_name(error));
        return error;
    }
    /* Registros leidos por SPI antes de calibrar sensores y crear tareas. */
    return nrf24l01_print_diagnostics(&nrf24l01);
}

/* Referencia unica por arranque, promediada con el CanSat quieto. El driver
 * conserva su altitud barometrica original; toda salida de la aplicacion usa
 * h_relativa = h_barometrica - referencia. No se recalcula al recuperar filtros.
 */
static esp_err_t initialize_relative_height(void)
{
    double height_sum = 0.0;
    TickType_t last_wake = xTaskGetTickCount();
    for (uint32_t sample = 0; sample < ALTITUDE_REFERENCE_SAMPLES; sample++)
    {
        const esp_err_t error = bmp180_read_data(&bmp180);
        if (error != ESP_OK)
            return error;
        if (!isfinite(bmp180.data.altitude))
            return ESP_ERR_INVALID_RESPONSE;
        height_sum += bmp180.data.altitude;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ALTITUDE_REFERENCE_PERIOD_MS));
    }
    s_baro_reference_altitude_m = height_sum / (double)ALTITUDE_REFERENCE_SAMPLES;
    ESP_LOGI(TAG, "Referencia barometrica=%.3f m; altura relativa inicial=0 m",
             s_baro_reference_altitude_m);
    return ESP_OK;
}

/* Solo vTaskAttitude modifica el filtro vertical. El barometro llega por cola;
 * predict usa cada aceleracion nueva y correct consume cada conversion una vez.
 */
static void update_telemetry_attitude(telemetry_data_t *data, const attitude_data_t *a,
                                    int64_t now_us)
{
    data->status &= (uint8_t)~TELEMETRY_STATUS_MPU6050_OK;
    const int64_t age_us = now_us - a->timestamp_us;
    if (age_us < 0 || age_us > ATTITUDE_MAX_AGE_US) return;
    if (a->valid && isfinite(a->roll_rad) && isfinite(a->pitch_rad) && isfinite(a->yaw_rad))
    {
        data->roll_centi_deg = scale_to_i16((double)a->roll_rad * RAD_TO_DEG, TELEMETRY_CENTI_SCALE);
        data->pitch_centi_deg = scale_to_i16((double)a->pitch_rad * RAD_TO_DEG, TELEMETRY_CENTI_SCALE);
        double yaw_deg = (double)a->yaw_rad * RAD_TO_DEG;
        if (yaw_deg < 0.0) yaw_deg += 360.0;
        data->yaw_centi_deg = (uint16_t)((uint32_t)lround(yaw_deg * TELEMETRY_CENTI_SCALE) % 36000U);
        data->status |= TELEMETRY_STATUS_MPU6050_OK;
    }
    /* La altura de telemetria procede exclusivamente del BMP180 filtrado.
     * El Kalman vertical sigue disponible para diagnostico y velocidad.
     */
    if (a->vertical_valid && isfinite(a->height_m) && isfinite(a->vertical_velocity_m_s))
    {
        data->vertical_speed_cms = scale_to_i16(a->vertical_velocity_m_s, TELEMETRY_CENTI_SCALE);
    }
}

static float vector_norm(const float vector[3])
{
    return sqrtf(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
}

static float lowpass_update(float previous, float input, float dt_s, float tau_s)
{
    return previous + (1.0f - expf(-dt_s / tau_s)) * (input - previous);
}

/* Residuo de gravedad sin restar el bias estimado: un bias grande no puede
 * ocultar una inclinacion incorrecta durante el diagnostico de reposo.
 */
static float gravity_error(const float accel_body[3])
{
    float linear[3];
    if (ekf_q_rotate_accel_to_navigation(&ekf_q, accel_body, linear) != ESP_OK)
        return INFINITY;
    linear[2] += ekf_qcfg.gravity_m_s2;
    return vector_norm(linear);
}

static bool barometer_is_stationary(int64_t now_us)
{
    return s_vertical.coherent_samples >= s_test_config.coherent_baro_samples
        && s_vertical.last_seen_us > 0 && now_us >= s_vertical.last_seen_us
        && now_us - s_vertical.last_seen_us <= ALTITUDE_BARO_MAX_AGE_US
        && fabsf(s_vertical.baro_velocity_m_s) <= s_test_config.stationary_baro_speed_m_s;
}

static bool supervise_attitude(const ekf_log_record_t *record)
{
    const float error = gravity_error(record->accel_body_m_s2);
    s_attitude_health.gravity_error_m_s2 = error;

    /* Diagnostico de gravedad, no condicion de caducidad ni reinicio. Una
     * aceleracion real puede rechazar correcciones sin invalidar el gyro.
     * Mismo criterio de salida que main2, comprobando valores numericos.
     */
    const attitude_data_t *a = &record->attitude;
    const float q_norm = sqrtf(a->quaternion[0] * a->quaternion[0] +
        a->quaternion[1] * a->quaternion[1] + a->quaternion[2] * a->quaternion[2] +
        a->quaternion[3] * a->quaternion[3]);
    return ekf_q.diagnostics.initialized && isfinite(q_norm) && fabsf(q_norm - 1.0f) < 0.001f &&
        isfinite(a->roll_rad) && isfinite(a->pitch_rad) && isfinite(a->yaw_rad) &&
        isfinite(vector_norm(a->linear_accel_nav_m_s2));
}

static bool update_barometer_tracking(const barometer_sample_t *sample)
{
    const float dt_s = (float)(sample->timestamp_us - s_vertical.last_seen_us) * 1.0e-6f;
    const bool contiguous = s_vertical.last_seen_us > 0 && dt_s > 0.0f &&
        sample->timestamp_us - s_vertical.last_seen_us <= ALTITUDE_BARO_MAX_AGE_US;
    const bool coherent = !contiguous ||
        fabsf(sample->height_m - s_vertical.last_height_m) <=
            s_test_config.max_baro_speed_m_s * dt_s + s_test_config.baro_jump_margin_m;
    if (!contiguous || !coherent)
    {
        s_vertical.smoothed_height_m = sample->height_m;
        s_vertical.baro_velocity_m_s = 0.0f;
        s_vertical.coherent_samples = coherent ? 1U : 0U;
        s_vertical.stationary_since_us = 0;
        s_vertical.stationary = false;
    }
    else
    {
        const float previous = s_vertical.smoothed_height_m;
        s_vertical.smoothed_height_m = lowpass_update(previous, sample->height_m,
                                                       dt_s, s_test_config.baro_height_tau_s);
        s_vertical.baro_velocity_m_s = lowpass_update(s_vertical.baro_velocity_m_s,
            (s_vertical.smoothed_height_m - previous) / dt_s,
            dt_s, s_test_config.baro_velocity_tau_s);
        if (s_vertical.coherent_samples < s_test_config.coherent_baro_samples)
            s_vertical.coherent_samples++;
    }
    s_vertical.last_seen_us = sample->timestamp_us;
    s_vertical.last_height_m = sample->height_m;
    return coherent;
}

static esp_err_t anchor_vertical_filter(int64_t now_us)
{
    /* Mantener el cero original. En movimiento no imponer velocidad cero. */
    const float velocity = s_vertical.stationary ? 0.0f : s_vertical.baro_velocity_m_s;
    const esp_err_t error = altitude_kalman_reset(&altitude_kalman,
        s_vertical.baro.height_m, velocity,
        s_test_config.barometer_noise_m * s_test_config.barometer_noise_m,
        s_vertical.stationary ? 0.01f : 25.0f);
    if (error != ESP_OK) return error;
    if (s_vertical.last_accepted_us > 0)
    {
        s_vertical.recoveries++;
        ESP_LOGW(TAG, "Kalman recuperado: h=%.2f m v=%.2f m/s, barometro coherente",
                 (double)s_vertical.baro.height_m, (double)velocity);
    }
    s_vertical.anchored = true;
    s_vertical.consecutive_rejections = 0;
    s_vertical.accepted_since_reset = 1U;
    s_vertical.last_accepted_us = s_vertical.baro.timestamp_us;
    altitude_kalman.diagnostics.last_predict_us = now_us;
    return ESP_OK;
}

static void update_vertical_navigation(ekf_log_record_t *record)
{
    attitude_data_t *attitude = &record->attitude;
    const int64_t now_us = attitude->timestamp_us;
    attitude->vertical_valid = false;
    attitude->height_m = attitude->vertical_velocity_m_s = NAN;
    s_navigation_diagnostics = (navigation_diagnostics_t){
        .baro_relative_m = NAN, .accel_up_m_s2 = NAN,
        .nis = s_vertical.last_nis, .baro_age_us = -1,
        .baro_accepted = s_vertical.last_baro_accepted,
        .gravity_error_m_s2 = s_attitude_health.gravity_error_m_s2,
        .attitude_resets = s_attitude_health.resets,
    };

    barometer_sample_t incoming;
    if (xQueueReceive(Queue_Baro, &incoming, 0) == pdTRUE &&
        incoming.timestamp_us > s_vertical.last_consumed_us)
    {
        s_vertical.baro = incoming;
        s_vertical.pending = true;
    }
    bool new_baro = false;
    if (s_vertical.pending && s_vertical.baro.timestamp_us <= now_us)
    {
        s_vertical.pending = false;
        s_vertical.last_consumed_us = s_vertical.baro.timestamp_us;
        s_vertical.last_baro_accepted = false;
        if (isfinite(s_vertical.baro.height_m) &&
            now_us - s_vertical.baro.timestamp_us <= ALTITUDE_BARO_MAX_LAG_US)
            new_baro = update_barometer_tracking(&s_vertical.baro);
    }

    const bool stationary_candidate = attitude->valid && record->accepted_axes == 7U &&
        attitude_sample_is_still(record->accel_body_m_s2, record->gyro_body_rad_s) &&
        s_attitude_health.gravity_error_m_s2 <= s_test_config.gravity_error_limit_m_s2 &&
        barometer_is_stationary(now_us);
    const bool was_stationary = s_vertical.stationary;
    if (!stationary_candidate) s_vertical.stationary_since_us = 0;
    else if (s_vertical.stationary_since_us == 0) s_vertical.stationary_since_us = now_us;
    s_vertical.stationary = s_vertical.stationary_since_us > 0 &&
        now_us - s_vertical.stationary_since_us >= s_test_config.stationary_confirmation_us;

    const int64_t filter_us = altitude_kalman.diagnostics.last_predict_us;
    const float dt_s = filter_us > 0 ? (float)(now_us - filter_us) * 1.0e-6f : 0.0f;
    if (s_vertical.anchored && (dt_s < 0.0f || dt_s > ekf_qcfg.max_dt_s))
    {
        s_vertical.anchored = false;
        s_vertical.accepted_since_reset = 0;
    }
    const float a_up = attitude->valid ? -attitude->linear_accel_nav_m_s2[2] : NAN;
    const bool inertial_valid = attitude->valid && isfinite(a_up);
    if (s_vertical.stationary && inertial_valid && dt_s > 0.0f &&
        dt_s <= ekf_qcfg.max_dt_s && fabsf(a_up) <= s_test_config.max_vertical_bias_m_s2)
        s_vertical.accel_bias_m_s2 = lowpass_update(s_vertical.accel_bias_m_s2,
                                                   a_up, dt_s, s_test_config.bias_tau_s);
    const float a_used = !s_vertical.stationary && inertial_valid
        ? a_up - s_vertical.accel_bias_m_s2 : 0.0f;
    const float noise = s_vertical.stationary ? s_test_config.stationary_accel_noise_m_s2
        : s_test_config.moving_accel_noise_m_s2 * (inertial_valid ? 1.0f : 2.0f);
    esp_err_t error = altitude_kalman_set_noise(&altitude_kalman, noise,
                                                s_test_config.barometer_noise_m);
    if (error == ESP_OK)
        error = altitude_kalman_set_nis_gate(&altitude_kalman, true,
            s_vertical.stationary ? s_test_config.stationary_nis_threshold
                                  : s_test_config.moving_nis_threshold);
    if (s_vertical.anchored && error == ESP_OK && dt_s > 0.0f)
    {
        if (s_vertical.stationary && !was_stationary)
            error = altitude_kalman_reset(&altitude_kalman,
                altitude_kalman_get_height(&altitude_kalman), 0.0f,
                altitude_kalman_get_height_variance(&altitude_kalman), 0.01f);
        else
            error = altitude_kalman_predict(&altitude_kalman, a_used, dt_s);
        altitude_kalman.diagnostics.last_predict_us = now_us;
    }
    if (error != ESP_OK) s_vertical.anchored = false;

    /* No prolongar indefinidamente una prediccion que perdio el anclaje. */
    if (s_vertical.anchored && now_us - s_vertical.last_accepted_us > ALTITUDE_BARO_MAX_AGE_US)
        s_vertical.anchored = false;
    if (new_baro && s_vertical.coherent_samples >= s_test_config.coherent_baro_samples)
    {
        if (!s_vertical.anchored)
            error = anchor_vertical_filter(now_us);
        else
        {
            error = altitude_kalman_correct_baro(&altitude_kalman, s_vertical.baro.height_m);
            s_navigation_diagnostics.nis = altitude_kalman.diagnostics.last_nis;
            s_vertical.last_nis = altitude_kalman.diagnostics.last_nis;
            if (error == ALTITUDE_KALMAN_ERR_OUTLIER)
            {
                s_vertical.consecutive_rejections++;
                if (s_vertical.consecutive_rejections >= s_test_config.recovery_rejections)
                    error = anchor_vertical_filter(now_us);
            }
            else if (error == ESP_OK)
            {
                s_vertical.consecutive_rejections = 0;
                if (s_vertical.accepted_since_reset < s_test_config.settling_corrections)
                    s_vertical.accepted_since_reset++;
                s_vertical.last_accepted_us = s_vertical.baro.timestamp_us;
            }
        }
        s_vertical.last_baro_accepted = error == ESP_OK;
        if (error != ESP_OK && error != ALTITUDE_KALMAN_ERR_OUTLIER)
            s_vertical.anchored = false;
    }
    if (s_vertical.anchored)
    {
        attitude->height_m = altitude_kalman_get_height(&altitude_kalman);
        attitude->vertical_velocity_m_s = altitude_kalman_get_velocity(&altitude_kalman);
        attitude->vertical_valid = isfinite(attitude->height_m) &&
            isfinite(attitude->vertical_velocity_m_s) &&
            s_vertical.accepted_since_reset >= s_test_config.settling_corrections &&
            now_us >= s_vertical.last_accepted_us &&
            now_us - s_vertical.last_accepted_us <= ALTITUDE_BARO_MAX_AGE_US;
    }
    s_navigation_diagnostics.baro_relative_m = s_vertical.last_seen_us > 0 ? s_vertical.last_height_m : NAN;
    s_navigation_diagnostics.baro_age_us = s_vertical.last_seen_us > 0 ? now_us - s_vertical.last_seen_us : -1;
    s_navigation_diagnostics.accel_up_m_s2 = a_up;
    s_navigation_diagnostics.accel_used_m_s2 = a_used;
    s_navigation_diagnostics.vertical_bias_m_s2 = s_vertical.accel_bias_m_s2;
    s_navigation_diagnostics.rejections = s_vertical.consecutive_rejections;
    s_navigation_diagnostics.recoveries = s_vertical.recoveries;
    s_navigation_diagnostics.stationary = s_vertical.stationary;
    s_navigation_diagnostics.baro_accepted = s_vertical.last_baro_accepted;
}

/* Mostrar la copia exacta entregada al driver de radio, despues de transmitir.
 * ESP_OK confirma el ACK hardware; cualquier otro resultado es un intento fallido.
 */
static void log_nrf24_telemetry(const telemetry_data_t *sample, esp_err_t result)
{
    ESP_LOGI(TAG,
           "NRF_TX seq=%lu resultado=%s lat=%.7f lon=%.7f sat=%u "
           "gps_datos=%u gps_fix=%u alt=%.2f m vel=%.2f m/s "
           "temp=%.2f C pres=%lu Pa yaw=%.2f roll=%.2f pitch=%.2f status=0x%02X",
           (unsigned long)sample->sequence,
           esp_err_to_name(result),
           (double)sample->latitude_e7 / TELEMETRY_COORDINATE_SCALE,
           (double)sample->longitude_e7 / TELEMETRY_COORDINATE_SCALE,
           (unsigned)sample->satellites,
           (unsigned)((sample->status & TELEMETRY_STATUS_NEO6M_OK) != 0),
           (unsigned)((sample->status & TELEMETRY_STATUS_GPS_FIX) != 0),
           (double)sample->altitude_cm / TELEMETRY_CENTI_SCALE,
           (double)sample->vertical_speed_cms / TELEMETRY_CENTI_SCALE,
           (double)sample->temperature_centi_c / TELEMETRY_CENTI_SCALE,
           (unsigned long)sample->pressure_pa,
           (double)sample->yaw_centi_deg / TELEMETRY_CENTI_SCALE,
           (double)sample->roll_centi_deg / TELEMETRY_CENTI_SCALE,
           (double)sample->pitch_centi_deg / TELEMETRY_CENTI_SCALE,
           (unsigned)sample->status);
}

static filter_storage_record_t compact_filter_record(const filter_test_record_t *sample)
{
    const ekf_log_record_t *record = &sample->ekf;
    const attitude_data_t *a = &record->attitude;
    const navigation_diagnostics_t *d = &sample->navigation;
    filter_storage_record_t compact = {
        .timestamp_us = a->timestamp_us,
        .baro_timestamp_us = d->baro_age_us >= 0 ? a->timestamp_us - d->baro_age_us : 0,
        .sequence = record->sequence, .dt_s = record->dt_s,
        .dropped_records = record->dropped_records, .recoveries = d->recoveries,
        .baro_relative_m = d->baro_relative_m, .height_m = a->height_m,
        .velocity_m_s = a->vertical_velocity_m_s, .accel_up_m_s2 = d->accel_up_m_s2,
        .accel_used_m_s2 = d->accel_used_m_s2, .bias_m_s2 = d->vertical_bias_m_s2,
        .nis = d->nis, .accepted_axes = record->accepted_axes,
        .flags = (uint8_t)((a->valid ? 1U : 0U) | (a->vertical_valid ? 2U : 0U)
            | (d->stationary ? 4U : 0U) | (d->baro_accepted ? 8U : 0U)
            | ((record->status & EKF_LOG_MPU_OK) != 0 ? 16U : 0U)
            | ((record->status & EKF_LOG_INITIALIZING) != 0 ? 32U : 0U)
            | (record->error != ESP_OK ? 64U : 0U)),
    };
    memcpy(compact.accel_body_m_s2, record->accel_body_m_s2, sizeof(compact.accel_body_m_s2));
    memcpy(compact.gyro_body_rad_s, record->gyro_body_rad_s, sizeof(compact.gyro_body_rad_s));
    memcpy(compact.quaternion, a->quaternion, sizeof(compact.quaternion));
    return compact;
}

/* Entradas a 100 Hz conservadas; Euler/normas se pueden calcular en MATLAB. */
static int write_ekf_csv(FILE *file, const char *prefix, const filter_storage_record_t *r)
{
    return fprintf(file,
        "%s%lld,%lu,%.6f,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f,"
        "%lld,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%u,%u,%lu,%lu\n",
        prefix, (long long)r->timestamp_us, (unsigned long)r->sequence, (double)r->dt_s,
        (double)r->accel_body_m_s2[0], (double)r->accel_body_m_s2[1], (double)r->accel_body_m_s2[2],
        (double)r->gyro_body_rad_s[0], (double)r->gyro_body_rad_s[1], (double)r->gyro_body_rad_s[2],
        (double)r->quaternion[0], (double)r->quaternion[1], (double)r->quaternion[2], (double)r->quaternion[3],
        (long long)r->baro_timestamp_us, (double)r->baro_relative_m,
        (double)r->height_m, (double)r->velocity_m_s, (double)r->accel_up_m_s2,
        (double)r->accel_used_m_s2, (double)r->bias_m_s2, (double)r->nis,
        (unsigned)r->accepted_axes, (unsigned)r->flags,
        (unsigned long)r->dropped_records, (unsigned long)r->recoveries);
}

static int write_navigation_csv(FILE *file, const filter_test_record_t *record)
{
    const navigation_diagnostics_t *d = &record->navigation;
    const attitude_data_t *a = &record->ekf.attitude;
    return fprintf(file, "%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.3f,%u,%lu,%lu,%u,%u,%u,%lld,%.4f,%lu\n",
        (double)d->baro_relative_m, (double)a->height_m, (double)a->vertical_velocity_m_s,
        (double)d->accel_up_m_s2, (double)d->accel_used_m_s2, (double)d->vertical_bias_m_s2,
        (double)d->nis, (unsigned)d->baro_accepted,
        (unsigned long)d->rejections, (unsigned long)d->recoveries,
        (unsigned)d->stationary, (unsigned)a->valid, (unsigned)a->vertical_valid,
        (long long)(d->baro_age_us >= 0 ? d->baro_age_us / 1000 : -1),
        (double)d->gravity_error_m_s2, (unsigned long)d->attitude_resets);
}

/* El printf se ejecuta en data_task, nunca en el ciclo del EKF de 100 Hz. */
static void print_navigation_test(const filter_test_record_t *record)
{
    /* Mantener la linea completa frente a logs de la tarea NRF24. */
    flockfile(stdout);
    if (fprintf(stdout, "NAV,%lld,%lu,", (long long)record->ekf.attitude.timestamp_us,
                (unsigned long)record->ekf.sequence) >= 0)
        (void)write_navigation_csv(stdout, record);
    funlockfile(stdout);
}

static bool rotation_is_valid(const float rotation[3][3])
{
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            float product = 0.0f;
            for (int axis = 0; axis < 3; axis++)
                product += rotation[row][axis] * rotation[col][axis];
            if (!isfinite(product) || fabsf(product - (row == col ? 1.0f : 0.0f)) > 0.001f)
                return false;
        }
    }
    const float determinant =
        rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0]);
    return fabsf(determinant - 1.0f) <= 0.001f;
}

static esp_err_t init_magnetometer(void)
{
    if (!rotation_is_valid(s_qmc_to_mpu))
        return ESP_ERR_INVALID_ARG;
    for (int row = 0; row < 3; row++)
    {
        if (!isfinite(s_qmc_hard_iron_ut[row])) return ESP_ERR_INVALID_ARG;
        for (int col = 0; col < 3; col++)
            if (!isfinite(s_qmc_soft_iron[row][col])) return ESP_ERR_INVALID_ARG;
    }
    esp_err_t error = mpu6050_set_i2c_bypass(&mpu6050, true);
    if (error != ESP_OK) return error;
    qmc5883p.bus_handle = bus_handle;
    qmc5883p.config = qmc5883pcfg;
    /* Lecturas sin espera de DRDY y con transacciones acotadas. */
    qmc5883p.config.io_timeout_ms = YAW_MAGNETOMETER_IO_TIMEOUT_MS;
    error = qmc5883p_init(&qmc5883p);
    if (error != ESP_OK) return error;

    ESP_LOGI(TAG, "Yaw QMC: declinacion=%.2f deg ruido=%.2f deg; "
                 "orientacion y calibracion en shared.h",
             (double)YAW_MAGNETIC_DECLINATION_RAD * RAD_TO_DEG,
             (double)YAW_MEASUREMENT_NOISE_STD_RAD * RAD_TO_DEG);
    for (int row = 0; row < 3; row++)
        ESP_LOGI(TAG, "Ejes fila %d: MPU->body=[%.2f %.2f %.2f] QMC->MPU=[%.2f %.2f %.2f]",
                 row, (double)s_mpu_to_body[row][0], (double)s_mpu_to_body[row][1],
                 (double)s_mpu_to_body[row][2], (double)s_qmc_to_mpu[row][0],
                 (double)s_qmc_to_mpu[row][1], (double)s_qmc_to_mpu[row][2]);
    return ESP_OK;
}

static esp_err_t calibrate_accel_for_mounting(void)
{
    /* Quieto y nivelado: fuerza especifica body=[0,0,-1] g.
     * R_mpu_body transpuesta da la referencia en los ejes fisicos del MPU.
     */
    const float reference_sensor_g[3] = {
        -s_mpu_to_body[2][0], -s_mpu_to_body[2][1], -s_mpu_to_body[2][2],
    };
    ESP_LOGI(TAG, "Calibracion accel: referencia sensor=[%.3f %.3f %.3f] g",
             (double)reference_sensor_g[0], (double)reference_sensor_g[1],
             (double)reference_sensor_g[2]);
    return mpu6050_calibrate_accel_reference(&mpu6050, reference_sensor_g);
}

static esp_err_t correct_yaw_with_magnetometer(void)
{
    esp_err_t error = qmc5883p_read_data(&qmc5883p, 0);
    if (error != ESP_OK) return error;
    const int64_t age_us = esp_timer_get_time() - qmc5883p.data.timestamp_us;
    if (!qmc5883p.data.valid || age_us < 0 || age_us > YAW_MAGNETIC_MAX_AGE_US)
        return ESP_ERR_INVALID_RESPONSE;

    const float sensor_ut[3] = {qmc5883p.data.x_ut, qmc5883p.data.y_ut, qmc5883p.data.z_ut};
    float calibrated_ut[3] = {0};
    float mpu_ut[3] = {0};
    float body_ut[3] = {0};
    for (int row = 0; row < 3; row++)
        for (int col = 0; col < 3; col++)
            calibrated_ut[row] += s_qmc_soft_iron[row][col]
                * (sensor_ut[col] - s_qmc_hard_iron_ut[col]);
    for (int row = 0; row < 3; row++)
        for (int col = 0; col < 3; col++)
            mpu_ut[row] += s_qmc_to_mpu[row][col] * calibrated_ut[col];
    for (int row = 0; row < 3; row++)
        for (int col = 0; col < 3; col++)
            body_ut[row] += s_mpu_to_body[row][col] * mpu_ut[col];

    const float norm_ut = sqrtf(body_ut[0] * body_ut[0] + body_ut[1] * body_ut[1]
                               + body_ut[2] * body_ut[2]);
    s_magnetic_yaw.field_norm_ut = norm_ut;
    if (!isfinite(norm_ut) || norm_ut < YAW_MIN_FIELD_UT || norm_ut > YAW_MAX_FIELD_UT)
        return ESP_ERR_INVALID_RESPONSE;

    float current_yaw_rad, pitch_rad;
    ekf_q_get_euler(&ekf_q, NULL, &pitch_rad, &current_yaw_rad);
    /* El heading Euler pierde definicion al apuntar el eje X verticalmente. */
    if (!isfinite(pitch_rad) || fabsf(pitch_rad) > YAW_MAX_ABS_PITCH_RAD)
        return ESP_ERR_INVALID_STATE;

    float rotation_bn[3][3];
    float navigation_ut[3] = {0};
    ekf_q_get_rotation_body_to_navigation(&ekf_q, rotation_bn);
    for (int row = 0; row < 3; row++)
        for (int col = 0; col < 3; col++)
            navigation_ut[row] += rotation_bn[row][col] * body_ut[col];
    const float horizontal_ut = hypotf(navigation_ut[0], navigation_ut[1]);
    if (!isfinite(horizontal_ut) || horizontal_ut < YAW_MIN_HORIZONTAL_FIELD_UT)
        return ESP_ERR_INVALID_RESPONSE;

    /* NED: el campo debe apuntar a declinacion D en el plano N/E.
     * Se elimina su error angular de la estimacion actual de yaw. Usar la
     * rotacion completa compensa roll/pitch, como en el proyecto CubeSat.
     */
    const float heading_rad = current_yaw_rad
        - atan2f(navigation_ut[1], navigation_ut[0]) + YAW_MAGNETIC_DECLINATION_RAD;
    s_magnetic_yaw.heading_rad = atan2f(sinf(heading_rad), cosf(heading_rad));
    return apply_magnetic_yaw_correction(s_magnetic_yaw.heading_rad);
}

static esp_err_t apply_magnetic_yaw_correction(float heading_rad)
{
    /* Restaurar la observacion magnetica usada por main2. El nucleo EKF
     * comprueba y confirma su correccion antes de actualizar x/P.
     */
    return ekf_q_correct_yaw(&ekf_q, heading_rad, YAW_MEASUREMENT_NOISE_STD_RAD);
}

static esp_err_t update_imu_attitude(const float gyro[3], const float accel[3], float dt_s)
{
    /* El nucleo predict puede modificar x/P antes de detectar un error.
     * Confirmar el ciclo completo sobre una copia evita borrar una actitud
     * valida por una lectura o un fallo numerico aislado. Misma matematica
     * e inputs que main2; workspace estatico para proteger la pila de la tarea.
     */
    static ekf_q_t candidate;
    candidate = ekf_q;
    const esp_err_t error = ekf_q_update(&candidate, gyro, accel, dt_s);
    if (error == ESP_OK) ekf_q = candidate;
    return error;
}

static void update_magnetic_yaw(void)
{
    const int64_t now_us = esp_timer_get_time();
    if (!qmc5883p.initialized || now_us < s_magnetic_yaw.next_sample_us) return;
    /* No repetir muestras ni recuperar periodos atrasados en rafaga. */
    s_magnetic_yaw.next_sample_us = now_us + YAW_CORRECTION_PERIOD_US;
    const esp_err_t error = correct_yaw_with_magnetometer();
    s_magnetic_yaw.last_error = error;
    if (error == ESP_OK)
        s_magnetic_yaw.last_correction_us = esp_timer_get_time();
    else if (error != ESP_ERR_NOT_FINISHED)
        s_magnetic_yaw.rejected_samples++;
}

static esp_err_t read_mpu_body(float accel_body[3], float gyro_body[3],
                               int64_t *timestamp_us)
{
    const int64_t start_us = esp_timer_get_time();
    esp_err_t error = mpu6050_read_accel(&mpu6050);
    if (error == ESP_OK)
        error = mpu6050_read_gyro(&mpu6050);

    *timestamp_us = start_us + (esp_timer_get_time() - start_us) / 2;
    if (error != ESP_OK)
        return error;

    const float accel_sensor[3] = {
        mpu6050.data.ax, mpu6050.data.ay, mpu6050.data.az};
    /* El driver ya resta los offsets de la calibracion. No restarlos otra vez. */
    const float gyro_sensor[3] = {
        mpu6050.data.gx * DEG_TO_RAD,
        mpu6050.data.gy * DEG_TO_RAD,
        mpu6050.data.gz * DEG_TO_RAD,
    };

    for (int row = 0; row < 3; row++)
    {
        accel_body[row] = 0.0f;
        gyro_body[row] = 0.0f;
        for (int col = 0; col < 3; col++)
        {
            accel_body[row] += s_mpu_to_body[row][col] * accel_sensor[col];
            gyro_body[row] += s_mpu_to_body[row][col] * gyro_sensor[col];
        }
        if (!isfinite(accel_body[row]) || !isfinite(gyro_body[row]))
            return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static bool attitude_sample_is_still(const float accel[3], const float gyro[3])
{
    const float accel_norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    const float gyro_norm = sqrtf(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
    return isfinite(accel_norm) && isfinite(gyro_norm) &&
        fabsf(accel_norm - ekf_qcfg.gravity_m_s2) <= s_test_config.stationary_accel_tolerance_m_s2 &&
        gyro_norm <= s_test_config.stationary_gyro_rad_s;
}

static esp_err_t initialize_attitude(void)
{
    float accel_sum[3] = {0};
    TickType_t last_wake = xTaskGetTickCount();
    for (uint32_t sample = 0; sample < ATTITUDE_INITIAL_SAMPLES; sample++)
    {
        float accel_body[3];
        float gyro_body[3];
        int64_t timestamp_us;
        esp_err_t error = read_mpu_body(accel_body, gyro_body, &timestamp_us);
        if (error != ESP_OK)
            return error;
        if (!attitude_sample_is_still(accel_body, gyro_body))
        {
            ESP_LOGE(TAG, "Movimiento durante la inicializacion de actitud");
            return ESP_ERR_INVALID_STATE;
        }
        for (int axis = 0; axis < 3; axis++)
            accel_sum[axis] += accel_body[axis];
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ATTITUDE_PERIOD_MS));
    }
    for (int axis = 0; axis < 3; axis++)
        accel_sum[axis] /= (float)ATTITUDE_INITIAL_SAMPLES;
    return ekf_q_set_initial_attitude(&ekf_q, accel_sum);
}

static FILE *open_csv_append(const char *path, const char *header,
                             char *buffer, size_t buffer_size)
{
    FILE *file = fopen(path, "a+");
    if (file == NULL)
    {
        ESP_LOGE(TAG, "No se pudo abrir %s: errno=%d (%s)",
                 path, errno, strerror(errno));
        return NULL;
    }
    if (setvbuf(file, buffer, _IOFBF, buffer_size) != 0 || fseek(file, 0, SEEK_END) != 0)
    {
        ESP_LOGE(TAG, "No se pudo preparar %s", path);
        fclose(file);
        return NULL;
    }
    const long size = ftell(file);
    if (size < 0 || (size == 0 && (fprintf(file, "%s\n", header) < 0 || fflush(file) != 0)))
    {
        ESP_LOGE(TAG, "No se pudo preparar la cabecera de %s", path);
        fclose(file);
        return NULL;
    }
    return file;
}

static int32_t scale_to_i32(double value, double scale)
{
    const double scaled = value * scale;

    if (!isfinite(scaled))
        return 0;
    if (scaled > (double)INT32_MAX)
        return INT32_MAX;
    if (scaled < (double)INT32_MIN)
        return INT32_MIN;

    return (int32_t)lround(scaled);
}

static int16_t scale_to_i16(double value, double scale)
{
    const double scaled = value * scale;

    if (!isfinite(scaled))
        return 0;
    if (scaled > (double)INT16_MAX)
        return INT16_MAX;
    if (scaled < (double)INT16_MIN)
        return INT16_MIN;

    return (int16_t)lround(scaled);
}

/* Registro SD: una sola tarea abre, escribe, sincroniza y cierra los CSV. */
static bool enqueue_sd_record(QueueHandle_t queue, const void *record)
{
    bool accepted = true;
    xSemaphoreTake(Mutex_Logging, portMAX_DELAY);
    const EventBits_t state = xEventGroupGetBits(EventGroup_status);
    if ((state & SD_LOGGING_READY) != 0)
    {
        if (queue == Queue_EkfLog)
        {
            /* Si la telemetria se atrasa, perder primero detalle del filtro.
             * El productor nunca espera espacio ni escribe directamente a SD.
             */
            accepted = (state & SD_FILTER_LOGGING_READY) != 0 &&
                uxQueueMessagesWaiting(Queue_data) < s_sd_policy.telemetry_pause_level;
            if (accepted)
            {
                filter_storage_record_t compact = *(const filter_storage_record_t *)record;
                compact.dropped_records = s_logger.ekf_drops;
                accepted = xQueueSend(queue, &compact, 0) == pdTRUE;
            }
            if (!accepted) s_logger.ekf_drops++;
        }
        else
        {
            accepted = xQueueSend(queue, record, 0) == pdTRUE;
            if (!accepted) s_logger.telemetry_drops++;
        }
    }
    xSemaphoreGive(Mutex_Logging);
    return accepted; /* Si esta detenido, no se considera una muestra perdida. */
}

static void set_logging_enabled(bool enabled)
{
    xSemaphoreTake(Mutex_Logging, portMAX_DELAY);
    if (enabled)
        xEventGroupSetBits(EventGroup_status, SD_LOGGING_READY);
    else
        xEventGroupClearBits(EventGroup_status, SD_LOGGING_READY);
    xSemaphoreGive(Mutex_Logging);
    if (s_radio_task != NULL) xTaskNotifyGive(s_radio_task);
}

static esp_err_t start_sd_logging(void)
{
    static char telemetry_buffer[SD_FILE_BUFFER_SIZE];
    static char ekf_buffer[SD_FILE_BUFFER_SIZE];
    if (s_logger.telemetry != NULL)
        return ESP_OK; /* REC_ON es idempotente. */

    if (!sd_card.mounted)
    {
        const esp_err_t error = sd_init(&sd_card);
        if (error != ESP_OK) return error;
    }

    /* No se arrastran muestras de una sesion ya cerrada o con error. */
    xQueueReset(Queue_data);
    xQueueReset(Queue_EkfLog);
    xEventGroupClearBits(EventGroup_status, SD_FILTER_LOGGING_READY);
    s_logger.telemetry = open_csv_append(s_test_telemetry_path, TELEMETRY_EMA_DATALOG_HEADER,
                                         telemetry_buffer, sizeof(telemetry_buffer));
    if (s_logger.telemetry == NULL) return ESP_FAIL;
    s_logger.ekf = open_csv_append(s_test_ekf_path, s_filter_sd_header,
                                  ekf_buffer, sizeof(ekf_buffer));
    if (s_logger.ekf == NULL)
        ESP_LOGW(TAG, "Filtro SD no disponible; se conserva el registro de telemetria");
    else
        xEventGroupSetBits(EventGroup_status, SD_FILTER_LOGGING_READY);
    reserve_telemetry_space();

    /* Confirmar escritura inicial antes de anunciar ACCEPTED. */
    const esp_err_t error = flush_sd_logs();
    if (error != ESP_OK)
    {
        (void)close_log_file(&s_logger.telemetry);
        (void)close_log_file(&s_logger.ekf);
        return error;
    }
    s_logger.error = ESP_OK;
    set_logging_enabled(true);
    ESP_LOGI(TAG, "REC_ON: telemetria=%s; filtro=%s (append)", s_test_telemetry_path,
             s_logger.ekf != NULL ? s_test_ekf_path : "OFF");
    return ESP_OK;
}

static esp_err_t write_telemetry_csv(const telemetry_log_record_t *record)
{
        const telemetry_data_t *sample = &record->data;
        if (fprintf(s_logger.telemetry,
                    "%lld,%lu,%.7f,%.7f,%.2f,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%u,"
                    "%lld,%.3f,%.3f,%.6f,%.6f,%u\n",
                    (long long)record->timestamp_us,
                    (unsigned long)sample->sequence,
                    (double)sample->latitude_e7 / TELEMETRY_COORDINATE_SCALE,
                    (double)sample->longitude_e7 / TELEMETRY_COORDINATE_SCALE,
                    (double)sample->altitude_cm / TELEMETRY_CENTI_SCALE,
                    (unsigned long)sample->pressure_pa,
                    (double)sample->vertical_speed_cms / TELEMETRY_CENTI_SCALE,
                    (double)sample->temperature_centi_c / TELEMETRY_CENTI_SCALE,
                    (double)sample->yaw_centi_deg / TELEMETRY_CENTI_SCALE,
                    (double)sample->roll_centi_deg / TELEMETRY_CENTI_SCALE,
                    (double)sample->pitch_centi_deg / TELEMETRY_CENTI_SCALE,
                    (unsigned)sample->status, (unsigned)sample->satellites,
                    (long long)record->barometer_timestamp_us,
                    (double)record->pressure_raw_pa,
                    (double)record->pressure_filtered_pa,
                    (double)record->pressure_filter_dt_s,
                    (double)record->pressure_filter_alpha,
                    (unsigned)record->barometer_valid) < 0)
            return ESP_FAIL;
        s_logger.telemetry_records++;
        return ESP_OK;
}

static void disable_filter_logging(uint32_t discarded_records)
{
    /* Limpiar su cola antes de cerrar; los productores comprueban el bit bajo
     * el mismo mutex. Un fallo opcional no desactiva SD_LOGGING_READY.
     */
    xSemaphoreTake(Mutex_Logging, portMAX_DELAY);
    xEventGroupClearBits(EventGroup_status, SD_FILTER_LOGGING_READY);
    s_logger.ekf_drops += uxQueueMessagesWaiting(Queue_EkfLog) + discarded_records;
    xQueueReset(Queue_EkfLog);
    xSemaphoreGive(Mutex_Logging);
    (void)close_log_file(&s_logger.ekf);
    ESP_LOGW(TAG, "Registro del filtro deshabilitado; telemetria continua");
}

static void reserve_telemetry_space(void)
{
    if (s_logger.ekf == NULL) return;
    uint64_t total_bytes, free_bytes;
    const esp_err_t error = esp_vfs_fat_info(sd_card.config.mount_point, &total_bytes, &free_bytes);
    if (error != ESP_OK || free_bytes <= s_sd_policy.telemetry_reserve_bytes)
    {
        if (error != ESP_OK)
            ESP_LOGW(TAG, "No se pudo consultar espacio SD: %s; se prioriza telemetria",
                     esp_err_to_name(error));
        else
            ESP_LOGW(TAG, "SD libre=%llu bytes; reserva telemetria=%llu bytes",
                     (unsigned long long)free_bytes, (unsigned long long)s_sd_policy.telemetry_reserve_bytes);
        disable_filter_logging(0U);
    }
}

static esp_err_t write_sd_batch(void)
{
    uint32_t telemetry_written = 0U, filter_written = 0U;
    const int64_t start_us = esp_timer_get_time();
    while (telemetry_written < s_sd_policy.telemetry_queue_length)
    {
        telemetry_log_record_t record;
        if (xQueueReceive(Queue_data, &record, 0) == pdTRUE)
        {
            const esp_err_t error = write_telemetry_csv(&record);
            if (error != ESP_OK) return error;
            telemetry_written++;
            continue;
        }
        if (s_logger.ekf == NULL || filter_written >= s_sd_policy.filter_batch_size ||
            esp_timer_get_time() - start_us >= s_sd_policy.filter_budget_us)
            break;
        filter_storage_record_t ekf_record;
        if (xQueueReceive(Queue_EkfLog, &ekf_record, 0) != pdTRUE) break;
        if (write_ekf_csv(s_logger.ekf, "", &ekf_record) < 0)
        {
            disable_filter_logging(1U);
            break;
        }
        s_logger.ekf_records++;
        filter_written++;
        /* Volver a comprobar telemetria despues de CADA registro del filtro. */
    }
    return ESP_OK;
}

static esp_err_t flush_sd_logs(void)
{
    if (s_logger.telemetry != NULL &&
        (fflush(s_logger.telemetry) != 0 || fsync(fileno(s_logger.telemetry)) != 0))
        return ESP_FAIL;
    if (s_logger.ekf != NULL &&
        (fflush(s_logger.ekf) != 0 || fsync(fileno(s_logger.ekf)) != 0))
    {
        disable_filter_logging(0U);
    }
    return ESP_OK;
}

static esp_err_t close_log_file(FILE **file)
{
    if (*file == NULL) return ESP_OK;
    esp_err_t error = ESP_OK;
    if (fflush(*file) != 0) error = ESP_FAIL;
    if (fsync(fileno(*file)) != 0) error = ESP_FAIL;
    if (fclose(*file) != 0) error = ESP_FAIL;
    *file = NULL;
    return error;
}

static esp_err_t stop_sd_logging(void)
{
    /* El mutex evita que un productor encole tras comprobar el bit READY.
     * Al liberarlo, ningun productor nuevo puede agregar datos al cierre.
     */
    set_logging_enabled(false);
    xEventGroupClearBits(EventGroup_status, SD_FILTER_LOGGING_READY);
    esp_err_t error = s_logger.error;
    if (s_logger.telemetry != NULL)
    {
        while (uxQueueMessagesWaiting(Queue_data) != 0
               || uxQueueMessagesWaiting(Queue_EkfLog) != 0)
        {
            error = write_sd_batch();
            if (error != ESP_OK) break;
        }
    }
    const esp_err_t telemetry_error = close_log_file(&s_logger.telemetry);
    const esp_err_t ekf_error = close_log_file(&s_logger.ekf);
    if (telemetry_error != ESP_OK || ekf_error != ESP_OK) error = ESP_FAIL;
    s_logger.error = error;
    if (error == ESP_OK)
        ESP_LOGI(TAG, "REC_OFF: archivos cerrados. Telemetria=%lu EKF=%lu descartados=%lu/%lu",
                 (unsigned long)s_logger.telemetry_records,
                 (unsigned long)s_logger.ekf_records,
                 (unsigned long)s_logger.telemetry_drops, (unsigned long)s_logger.ekf_drops);
    else
        ESP_LOGE(TAG, "REC_OFF: fallo al guardar o cerrar los archivos");
    return error;
}

static esp_err_t sound_sd_closed(void)
{
    if (!s_buzzer_ready) return ESP_ERR_INVALID_STATE;
    esp_err_t error = buzzer_set_frequency(&s_buzzer, SD_CLOSE_BUZZER_HZ);
    if (error != ESP_OK) return error;
    vTaskDelay(pdMS_TO_TICKS(SD_CLOSE_BUZZER_MS));
    error = buzzer_off(&s_buzzer);
    if (error != ESP_OK)
        ESP_LOGE(TAG, "No se pudo apagar el buzzer: %s", esp_err_to_name(error));
    return error;
}

/* Solo vTaskAttitude llama a esta funcion mientras las tareas estan activas.
 * No cambia la referencia de altura relativa fijada en el arranque.
 */
static esp_err_t recalibrate_attitude(void)
{
    float accel[3], gyro[3];
    int64_t timestamp_us;
    esp_err_t error = read_mpu_body(accel, gyro, &timestamp_us);
    if (error == ESP_OK && (!attitude_sample_is_still(accel, gyro)
                            || fabsf(accel[0]) > ATTITUDE_STILL_ACCEL_TOLERANCE * ekf_qcfg.gravity_m_s2
                            || fabsf(accel[1]) > ATTITUDE_STILL_ACCEL_TOLERANCE * ekf_qcfg.gravity_m_s2
                            || accel[2] >= 0.0f))
        error = ESP_ERR_INVALID_STATE;
    if (error != ESP_OK)
    {
        ESP_LOGW(TAG, "CALIBRAR rechazado: verificar reposo, nivel y lectura del MPU (%s)",
                 esp_err_to_name(error));
        return error;
    }

    xEventGroupSetBits(EventGroup_status, COMMAND_CALIBRATING);
    const filter_test_record_t invalid_attitude = {0};
    xQueueOverwrite(Queue_Attitude, &invalid_attitude);
    xQueueReset(Queue_EkfSerial);
    const mpu6050_calib_data previous_calibration = mpu6050.calib;
    ESP_LOGI(TAG, "CALIBRAR: mantener CanSat quieto y nivelado segun MPU_TO_BODY_ROTATION");

    if (error == ESP_OK) error = calibrate_accel_for_mounting();
    if (error == ESP_OK) error = mpu6050_calibrate_gyro(&mpu6050);
    if (error == ESP_OK) error = ekf_q_init(&ekf_q, &ekf_qcfg);
    if (error == ESP_OK) error = initialize_attitude();

    if (error != ESP_OK)
    {
        mpu6050.calib = previous_calibration;
        (void)ekf_q_init(&ekf_q, &ekf_qcfg);
        ekf_q.diagnostics.initialized = false;
        ESP_LOGE(TAG, "CALIBRAR fallo: %s; offsets anteriores restaurados",
                 esp_err_to_name(error));
    }
    const esp_err_t vertical_error = altitude_kalman_init(&altitude_kalman, &altitude_kalmancfg);
    if (error == ESP_OK) error = vertical_error;
    memset(&s_vertical, 0, sizeof(s_vertical));
    memset(&s_attitude_health, 0, sizeof(s_attitude_health));
    xQueueReset(Queue_Baro);
    xEventGroupClearBits(EventGroup_status, COMMAND_CALIBRATING);
    if (error == ESP_OK)
        ESP_LOGI(TAG, "CALIBRAR terminado: MPU, gravedad y EKF reiniciados; "
                     "el QMC retomara la referencia de yaw. Calibracion magnetica sin cambios");
    return error;
}

/* Protocolo validado en main_Tx: radio usado exclusivamente por la tarea TX. */
static const char *command_name(uint8_t command)
{
    switch (command)
    {
    case NRF24_COMMAND_REC_ON: return "REC_ON";
    case NRF24_COMMAND_REC_OFF: return "REC_OFF";
    case NRF24_COMMAND_CALIBRATE: return "CALIBRAR";
    default: return "DESCONOCIDO";
    }
}

static uint8_t command_state(void)
{
    const EventBits_t bits = xEventGroupGetBits(EventGroup_status);
    return ((bits & SD_LOGGING_READY) ? 1U : 0U)
           | ((bits & (COMMAND_CALIBRATING | APP_INITIALIZING)) ? 2U : 0U);
}

static void complete_command(const nrf24_command_packet_t *command, esp_err_t error)
{
    const uint8_t failure_result = command->command == NRF24_COMMAND_CALIBRATE
                                  && error == ESP_ERR_INVALID_STATE
        ? NRF24_COMMAND_RESULT_REJECTED_STATE : NRF24_COMMAND_RESULT_ACTUATOR_ERROR;
    const command_result_t result = {
        .command = *command,
        .result = error == ESP_OK ? NRF24_COMMAND_RESULT_ACCEPTED
                                 : failure_result,
        .completed = true,
    };
    ESP_LOGI(TAG, "%s seq=%u terminado: %s",
             command_name(command->command), (unsigned)command->sequence,
             esp_err_to_name(error));
    xQueueSend(Queue_CommandResults, &result, portMAX_DELAY);
    if (s_radio_task != NULL) xTaskNotifyGive(s_radio_task);
}

static void send_command_response(const command_result_t *result, uint8_t state)
{
    nrf24_command_ack_packet_t response = {
        .magic = NRF24_COMMAND_ACK_MAGIC,
        .sequence = result->command.sequence,
        .command = result->command.command,
        .result = result->result,
        .state = state,
    };
    response.crc16 = nrf24_command_crc16(
        &response, offsetof(nrf24_command_ack_packet_t, crc16));
    const esp_err_t error = nrf24l01_transmit(
        &nrf24l01, &response, sizeof(response), NRF24_TX_TIMEOUT_MS);
    ESP_LOGI(TAG, "Respuesta %s seq=%u resultado=%u state=%u: %s",
             command_name(response.command), (unsigned)response.sequence,
             (unsigned)response.result, (unsigned)response.state, esp_err_to_name(error));
}

static void process_ack_command(command_history_t *history)
{
    for (unsigned i = 0; i < COMMAND_RX_BATCH_SIZE; ++i)
    {
        bool available = false;
        nrf24_pipe_t pipe = NRF24_PIPE_0;
        esp_err_t error = nrf24l01_data_available(&nrf24l01, &available, &pipe);
        if (error != ESP_OK || !available) return;

        uint8_t payload[NRF24_MAX_PAYLOAD_SIZE];
        size_t length = 0;
        error = nrf24l01_receive(&nrf24l01, payload, sizeof(payload), &length, &pipe);
        if (error != ESP_OK) return;
        if (length != sizeof(nrf24_command_packet_t))
        {
            ESP_LOGW(TAG, "ACK descartado: largo=%u", (unsigned)length);
            continue;
        }
        nrf24_command_packet_t command;
        memcpy(&command, payload, sizeof(command));
        if (command.magic != NRF24_COMMAND_MAGIC
            || command.crc16 != nrf24_command_crc16(
                &command, offsetof(nrf24_command_packet_t, crc16)))
        {
            ESP_LOGW(TAG, "ACK descartado: magic o CRC invalido");
            continue;
        }

        ESP_LOGI(TAG, "Comando ACK: %s seq=%u",
                 command_name(command.command), (unsigned)command.sequence);
        command_result_t result = {.command = command, .completed = false};
        if (history->valid && command.sequence == history->command.sequence)
        {
            if (command.command != history->command.command)
                result.result = NRF24_COMMAND_RESULT_INVALID_COMMAND;
            else if (history->pending)
            {
                ESP_LOGI(TAG, "Reintento pendiente: no se repite %s",
                         command_name(command.command));
                continue; /* No confirmar antes de que termine la operacion. */
            }
            else
            {
                /* Un fallo previo se conserva: DUPLICATE no debe parecer exito. */
                result.result = history->result == NRF24_COMMAND_RESULT_ACCEPTED
                    ? NRF24_COMMAND_RESULT_DUPLICATE : history->result;
            }
        }
        else if (command.command < NRF24_COMMAND_REC_ON
                 || command.command > NRF24_COMMAND_CALIBRATE)
            result.result = NRF24_COMMAND_RESULT_INVALID_COMMAND;
        else if (history->pending)
            result.result = NRF24_COMMAND_RESULT_REJECTED_STATE;
        else
        {
            QueueHandle_t destination = command.command == NRF24_COMMAND_CALIBRATE
                ? Queue_Calibration : Queue_SDCommands;
            if (xQueueSend(destination, &command, 0) == pdTRUE)
            {
                history->command = command;
                history->valid = true;
                history->pending = true;
                continue;
            }
            result.result = NRF24_COMMAND_RESULT_REJECTED_STATE;
        }
        if (xQueueSend(Queue_CommandResults, &result, 0) != pdTRUE)
            ESP_LOGW(TAG, "Cola de respuestas llena; el receptor puede reintentar");
        else if (s_radio_task != NULL)
            xTaskNotifyGive(s_radio_task);
    }
}
