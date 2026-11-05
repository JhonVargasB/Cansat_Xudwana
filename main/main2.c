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

static const char *TAG = "main2";

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
#define MAIN2_LOG_EKF_SERIAL 0

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
} sd_logger_t;

typedef struct {
    bool anchored;
    bool pending;
    barometer_sample_t baro;
    int64_t last_consumed_us;
    int64_t last_accepted_us;
} vertical_navigation_state_t;

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
#if MAIN2_LOG_EKF_SERIAL
static void print_ekf_serial(const ekf_log_record_t *record);
#endif
static int write_ekf_csv(FILE *file, const char *prefix, const ekf_log_record_t *record);
static void update_vertical_navigation(attitude_data_t *attitude);

// Prototipo de funciones de aplciacion
void app_main(void)
{
    // Configuracion de dispositivos y recursos antes de habilitar la radio.
    nrf24l01.config = nrf24l01cfg;
    sd_card.config = sd_config;
    neo6m.config = neo6mcfg;
    mpu6050.config = mpu6050cfg;
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
                /* Tras un fallo numerico o un hueco grande no se inventa una
                 * rotacion: se esperan 100 muestras quietas para reinicializar.
                 * La recuperacion fija yaw a cero; el QMC vuelve a referenciarlo.
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
                            ESP_LOGI(TAG, "Actitud EKF reinicializada en reposo");
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
                    ekf_q.diagnostics.initialized = false;
                }
                else
                {
                    record.error = ekf_q_update(&ekf_q, record.gyro_body_rad_s,
                                                record.accel_body_m_s2,
                                                record.dt_s);
                    record.update_us = ekf_q.diagnostics.last_total_us;
                    if (record.error != ESP_OK)
                        ekf_q.diagnostics.initialized = false;
                    else
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
                record.attitude.valid = record.error == ESP_OK;
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
        }

        /* Incluso los fallos reemplazan la muestra, para no publicar una
         * estimacion antigua como si correspondiese a una lectura nueva.
         */
        update_vertical_navigation(&record.attitude);
        xQueueOverwrite(Queue_Attitude, &record.attitude);

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
        xQueueOverwrite(Queue_EkfSerial, &record);

        if (!enqueue_sd_record(Queue_EkfLog, &record))
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
            }
            else
            {
                error = stop_sd_logging();
                if (error == ESP_OK)
                    error = sound_sd_closed();
            }
            complete_command(&command, error);
        }

        if (s_logger.telemetry != NULL && s_logger.ekf != NULL)
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
        }
    }
}

static void vTaskReadData(void *pVparameters)
{
    (void)pVparameters;

    neo6m_data_t latest_gps = {0};
    bool gps_received = false;
    uint32_t sequence = 0U;
    TickType_t last_wake = xTaskGetTickCount();

#if MAIN2_LOG_EKF_SERIAL
    puts(EKF_SERIAL_HEADER);
    uint32_t last_ekf_sequence = 0U;
    bool ekf_serial_received = false;
#endif

    while (1)
    {
        while (xQueueReceive(Queue_GPS, &latest_gps, 0) == pdTRUE)
            gps_received = true;

        telemetry_log_record_t record = {
            .timestamp_us = esp_timer_get_time(),
            .data.sequence = sequence++,
            .pressure_raw_pa = NAN,
            .pressure_filtered_pa = NAN,
            .pressure_filter_dt_s = NAN,
            .pressure_filter_alpha = NAN,
        };

        const int64_t baro_start_us = esp_timer_get_time();
        esp_err_t bmp_error = bmp180_read_data(&bmp180);
        const int64_t baro_end_us = esp_timer_get_time();
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
            record.data.pressure_pa = bmp180.data.pressure > 0.0
                                          ? (uint32_t)lround(bmp180.data.pressure)
                                          : 0U;
            record.data.temperature_centi_c = scale_to_i16(bmp180.data.temperature, TELEMETRY_CENTI_SCALE);
            record.data.status |= TELEMETRY_STATUS_BMP180_OK;
            if (isfinite(relative_height_m))
            {
                const barometer_sample_t baro_sample = {
                    .timestamp_us = baro_start_us + (baro_end_us - baro_start_us) / 2,
                    .height_m = (float)relative_height_m,
                };
                xQueueOverwrite(Queue_Baro, &baro_sample);
            }
        }

        attitude_data_t attitude;
        if ((xEventGroupGetBits(EventGroup_status) & COMMAND_CALIBRATING) == 0
            && xQueuePeek(Queue_Attitude, &attitude, 0) == pdTRUE && attitude.valid)
        {
            const int64_t age_us = esp_timer_get_time() - attitude.timestamp_us;
            if (age_us >= 0 && age_us <= ATTITUDE_MAX_AGE_US)
            {
                record.data.roll_centi_deg = scale_to_i16(
                    (double)attitude.roll_rad * RAD_TO_DEG, TELEMETRY_CENTI_SCALE);
                record.data.pitch_centi_deg = scale_to_i16(
                    (double)attitude.pitch_rad * RAD_TO_DEG, TELEMETRY_CENTI_SCALE);
                /* El EKF da [-pi,pi]; la telemetria usa yaw sin signo [0,360). */
                double yaw_deg = (double)attitude.yaw_rad * RAD_TO_DEG;
                if (yaw_deg < 0.0)
                    yaw_deg += 360.0;
                record.data.yaw_centi_deg = (uint16_t)((uint32_t)lround(yaw_deg * TELEMETRY_CENTI_SCALE) % 36000U);
                record.data.status |= TELEMETRY_STATUS_MPU6050_OK;
                if (attitude.vertical_valid)
                {
                    record.data.altitude_cm = scale_to_i32(
                        attitude.height_m, TELEMETRY_CENTI_SCALE);
                    record.data.vertical_speed_cms = scale_to_i16(
                        attitude.vertical_velocity_m_s, TELEMETRY_CENTI_SCALE);
                }
                /* Hasta tener una estimacion vertical valida se conserva la
                 * altura barometrica relativa y la velocidad permanece en cero.
                 */
            }
        }

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
#if MAIN2_LOG_EKF_SERIAL
        ekf_log_record_t ekf_record;
        if (xQueuePeek(Queue_EkfSerial, &ekf_record, 0) == pdTRUE
            && (!ekf_serial_received || ekf_record.sequence != last_ekf_sequence))
        {
            print_ekf_serial(&ekf_record);
            last_ekf_sequence = ekf_record.sequence;
            ekf_serial_received = true;
        }
#endif

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
    Queue_data = xQueueCreate(TELEMETRY_QUEUE_LENGTH, sizeof(telemetry_log_record_t));
    Queue_radio = xQueueCreate(RADIO_QUEUE_LENGTH, sizeof(radio_sample_t));
    Queue_Attitude = xQueueCreate(ATTITUDE_QUEUE_LENGTH, sizeof(attitude_data_t));
    Queue_EkfLog = xQueueCreate(EKF_LOG_QUEUE_LENGTH, sizeof(ekf_log_record_t));
    Queue_EkfSerial = xQueueCreate(EKF_SERIAL_QUEUE_LENGTH, sizeof(ekf_log_record_t));
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
static void update_vertical_navigation(attitude_data_t *attitude)
{
    const int64_t now_us = attitude->timestamp_us;
    attitude->vertical_valid = false;

    barometer_sample_t incoming;
    if (xQueueReceive(Queue_Baro, &incoming, 0) == pdTRUE
        && incoming.timestamp_us > s_vertical.last_consumed_us)
    {
        s_vertical.baro = incoming;
        s_vertical.pending = true;
    }
    if (!attitude->valid)
        return;

    const float acceleration_up = -attitude->linear_accel_nav_m_s2[2];
    if (!isfinite(acceleration_up))
        return;

    int64_t filter_us = altitude_kalman.diagnostics.last_predict_us;
    if (s_vertical.anchored && (now_us <= filter_us
                     || (float)(now_us - filter_us) * 1.0e-6f > ekf_qcfg.max_dt_s))
    {
        /* No integrar un hueco grande con una sola muestra. Esperar un nuevo
         * anclaje barometrico; la velocidad se reinicia con la configuracion.
         */
        s_vertical.anchored = false;
    }

    if (s_vertical.pending && s_vertical.baro.timestamp_us <= now_us)
    {
        s_vertical.pending = false;
        s_vertical.last_consumed_us = s_vertical.baro.timestamp_us;
        if (isfinite(s_vertical.baro.height_m)
            && now_us - s_vertical.baro.timestamp_us <= ALTITUDE_BARO_MAX_LAG_US)
        {
            esp_err_t error;
            if (!s_vertical.anchored)
            {
                /* El barometro ya llega relativo al arranque. Al recuperar el
                 * filtro se conserva ese origen, incluso si estamos en vuelo.
                 */
                error = altitude_kalman_reset(
                    &altitude_kalman, s_vertical.baro.height_m,
                    altitude_kalmancfg.initial_velocity_mps,
                    altitude_kalmancfg.initial_height_variance,
                    altitude_kalmancfg.initial_velocity_variance);
                if (error != ESP_OK)
                    return;
                s_vertical.anchored = true;
                filter_us = s_vertical.baro.timestamp_us;
                s_vertical.last_accepted_us = s_vertical.baro.timestamp_us;
                altitude_kalman.diagnostics.last_predict_us = filter_us;
            }
            else
            {
                /* Si la medicion cae en este intervalo, dividir la prediccion
                 * y corregir en su timestamp. Un retraso menor de 100 ms se
                 * aproxima al estado actual; no se rebobina ni duplica P.
                 */
                if (s_vertical.baro.timestamp_us > filter_us)
                {
                    error = altitude_kalman_predict(
                        &altitude_kalman, acceleration_up,
                        (float)(s_vertical.baro.timestamp_us - filter_us) * 1.0e-6f);
                    if (error != ESP_OK)
                    {
                        s_vertical.anchored = false;
                        return;
                    }
                    filter_us = s_vertical.baro.timestamp_us;
                    altitude_kalman.diagnostics.last_predict_us = filter_us;
                }
                error = altitude_kalman_correct_baro(&altitude_kalman, s_vertical.baro.height_m);
                if (error == ESP_OK)
                    s_vertical.last_accepted_us = s_vertical.baro.timestamp_us;
                else if (error != ALTITUDE_KALMAN_ERR_OUTLIER)
                {
                    s_vertical.anchored = false;
                    return;
                }
                /* Un outlier no reinicia el filtro ni deshabilita su compuerta. */
            }
        }
    }

    if (!s_vertical.anchored)
        return;
    if (now_us > filter_us)
    {
        const esp_err_t error = altitude_kalman_predict(
            &altitude_kalman, acceleration_up, (float)(now_us - filter_us) * 1.0e-6f);
        if (error != ESP_OK)
        {
            s_vertical.anchored = false;
            return;
        }
        altitude_kalman.diagnostics.last_predict_us = now_us;
    }

    attitude->height_m = altitude_kalman_get_height(&altitude_kalman);
    attitude->vertical_velocity_m_s = altitude_kalman_get_velocity(&altitude_kalman);
    attitude->vertical_valid = isfinite(attitude->height_m)
                              && isfinite(attitude->vertical_velocity_m_s)
                              && now_us - s_vertical.last_accepted_us <= ALTITUDE_BARO_MAX_AGE_US;
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

/* Registro completo en SD; la consola publica un resumen independiente. */
static int write_ekf_csv(FILE *file, const char *prefix, const ekf_log_record_t *record)
{
    const attitude_data_t *attitude = &record->attitude;
    return fprintf(
        file,
        "%s%lld,%lu,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.7f,%.7f,%.7f,%.7f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
        "%.6f,%.7f,%u,%u,%ld,%lu,%lu\n",
        prefix, (long long)attitude->timestamp_us,
        (unsigned long)record->sequence, (double)record->dt_s,
        (double)record->accel_body_m_s2[0],
        (double)record->accel_body_m_s2[1],
        (double)record->accel_body_m_s2[2],
        (double)record->gyro_body_rad_s[0],
        (double)record->gyro_body_rad_s[1],
        (double)record->gyro_body_rad_s[2],
        (double)attitude->quaternion[0], (double)attitude->quaternion[1],
        (double)attitude->quaternion[2], (double)attitude->quaternion[3],
        (double)attitude->roll_rad, (double)attitude->pitch_rad,
        (double)attitude->yaw_rad,
        (double)record->accel_bias_m_s2[0],
        (double)record->accel_bias_m_s2[1],
        (double)record->accel_bias_m_s2[2],
        (double)record->innovation_m_s2[0],
        (double)record->innovation_m_s2[1],
        (double)record->innovation_m_s2[2],
        (double)attitude->linear_accel_nav_m_s2[0],
        (double)attitude->linear_accel_nav_m_s2[1],
        (double)attitude->linear_accel_nav_m_s2[2],
        (double)record->covariance_diagonal[0],
        (double)record->covariance_diagonal[1],
        (double)record->covariance_diagonal[2],
        (double)record->covariance_diagonal[3],
        (double)record->covariance_diagonal[4],
        (double)record->covariance_diagonal[5],
        (double)record->covariance_diagonal[6],
        (double)record->accel_norm_m_s2,
        (double)record->quaternion_norm,
        (unsigned)record->accepted_axes, (unsigned)record->status,
        (long)record->error, (unsigned long)record->update_us,
        (unsigned long)record->dropped_records);
}

#if MAIN2_LOG_EKF_SERIAL
static void print_ekf_serial(const ekf_log_record_t *record)
{
    const attitude_data_t *attitude = &record->attitude;
    printf(EKF_SERIAL_PREFIX "%lld,%lu,%.2f,%.2f,%.2f,%.4f,%u,%.3f,%.3f,%u\n",
           (long long)attitude->timestamp_us,
           (unsigned long)record->sequence,
           (double)attitude->roll_rad * RAD_TO_DEG,
           (double)attitude->pitch_rad * RAD_TO_DEG,
           (double)attitude->yaw_rad * RAD_TO_DEG,
           (double)attitude->linear_accel_nav_m_s2[2],
           (unsigned)attitude->valid,
           (double)attitude->height_m,
           (double)attitude->vertical_velocity_m_s,
           (unsigned)attitude->vertical_valid);
}
#endif

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
    return ekf_q_correct_yaw(&ekf_q, s_magnetic_yaw.heading_rad,
                              YAW_MEASUREMENT_NOISE_STD_RAD);
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
    return fabsf(accel_norm - ekf_qcfg.gravity_m_s2) <= ATTITUDE_STILL_ACCEL_TOLERANCE * ekf_qcfg.gravity_m_s2 && gyro_norm <= ATTITUDE_STILL_GYRO_RAD_S;
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
    if ((xEventGroupGetBits(EventGroup_status) & SD_LOGGING_READY) != 0)
        accepted = xQueueSend(queue, record, 0) == pdTRUE;
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
    if (s_logger.telemetry != NULL && s_logger.ekf != NULL)
        return ESP_OK; /* REC_ON es idempotente. */

    if (!sd_card.mounted)
    {
        const esp_err_t error = sd_init(&sd_card);
        if (error != ESP_OK) return error;
    }

    /* No se arrastran muestras de una sesion ya cerrada o con error. */
    xQueueReset(Queue_data);
    xQueueReset(Queue_EkfLog);
    s_logger.telemetry = open_csv_append(TELEMETRY_EMA_DATALOG_PATH, TELEMETRY_EMA_DATALOG_HEADER,
                                         telemetry_buffer, sizeof(telemetry_buffer));
    if (s_logger.telemetry == NULL) return ESP_FAIL;
    s_logger.ekf = open_csv_append(EKF_DATALOG_PATH, EKF_DATALOG_HEADER,
                                  ekf_buffer, sizeof(ekf_buffer));
    if (s_logger.ekf == NULL)
    {
        (void)close_log_file(&s_logger.telemetry);
        return ESP_FAIL;
    }

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
    ESP_LOGI(TAG, "REC_ON: grabando %s y %s (append)", TELEMETRY_EMA_DATALOG_PATH, EKF_DATALOG_PATH);
    return ESP_OK;
}

static esp_err_t write_sd_batch(void)
{
    telemetry_log_record_t record;
    if (xQueueReceive(Queue_data, &record, 0) == pdTRUE)
    {
        const telemetry_data_t *sample = &record.data;
        if (fprintf(s_logger.telemetry,
                    "%lld,%lu,%.7f,%.7f,%.2f,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%u,"
                    "%lld,%.3f,%.3f,%.6f,%.6f,%u\n",
                    (long long)record.timestamp_us,
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
                    (long long)record.barometer_timestamp_us,
                    (double)record.pressure_raw_pa,
                    (double)record.pressure_filtered_pa,
                    (double)record.pressure_filter_dt_s,
                    (double)record.pressure_filter_alpha,
                    (unsigned)record.barometer_valid) < 0)
            return ESP_FAIL;
        s_logger.telemetry_records++;
    }

    ekf_log_record_t ekf_record;
    for (uint32_t i = 0; i < SD_EKF_BATCH_SIZE; ++i)
    {
        if (xQueueReceive(Queue_EkfLog, &ekf_record, 0) != pdTRUE) break;
        if (write_ekf_csv(s_logger.ekf, "", &ekf_record) < 0) return ESP_FAIL;
        s_logger.ekf_records++;
    }
    return ESP_OK;
}

static esp_err_t flush_sd_logs(void)
{
    esp_err_t error = ESP_OK;
    FILE *files[] = {s_logger.telemetry, s_logger.ekf};
    for (size_t i = 0; i < sizeof(files) / sizeof(files[0]); ++i)
    {
        if (files[i] == NULL) continue;
        if (fflush(files[i]) != 0) error = ESP_FAIL;
        if (fsync(fileno(files[i])) != 0) error = ESP_FAIL;
    }
    return error;
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
    esp_err_t error = s_logger.error;
    if (s_logger.telemetry != NULL && s_logger.ekf != NULL)
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
        ESP_LOGI(TAG, "REC_OFF: archivos sincronizados y cerrados. Telemetria=%lu EKF=%lu",
                 (unsigned long)s_logger.telemetry_records,
                 (unsigned long)s_logger.ekf_records);
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
    const attitude_data_t invalid_attitude = {0};
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
