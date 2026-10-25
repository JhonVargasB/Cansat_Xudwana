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
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"

// Librerias propias

#include "shared.h"

#include "nrf24_example_protocol.h"
#include "nrf24l01.h"
#include "sd_lector.h"
#include "neo6m.h"
#include "bmp180.h"
#include "mpu6050.h"
#include "ekq_q.h"
#include "altitude_kalman.h"

_Static_assert(EKF_LOG_STATE_DIM == EKF_Q_STATE_DIM,
               "El registro EKF debe coincidir con la dimension del filtro");

static const char *TAG = "main";

// Identificadores de haradware
static i2c_master_bus_handle_t bus_handle;
/* Se fija antes de crear tareas y se conserva durante toda la ejecucion. */
static double s_baro_reference_altitude_m;

// Bus I2C1 compartido por el BMP180 y el MPU6050.
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
nrf24l01_t nrf24l01;
ekf_q_t ekf_q;
altitude_kalman_t altitude_kalman;

neo6m_config_t neo6mcfg = NEO6M_DEFAULT_CONFIG();
bmp180_config_t bmp180cfg = BMP180_DEFAULT_CONFIG();
mpu6050_config_t mpu6050cfg = MPU6050_DEFAULT_CONFIG();
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

static volatile uint32_t s_telemetry_period_ms =
    TELEMETRY_PERIOD_DEFAULT_MS;

static const uint8_t s_radio_address[NRF24_MAX_ADDRESS_WIDTH] =
    NRF24_TELEMETRY_ADDRESS;
static const float s_mpu_to_body[3][3] = MPU_TO_BODY_ROTATION;

// Prototipo de Tareas
static void vTaskReadGPS(void *pVparameters);
static void vTaskReadData(void *pVparameters);
static void vTaskWriteSD(void *pVparameters);
static void vTaskTransmitData(void *pVparameters);
static void vTaskAttitude(void *pVparameters);

// Prototipo de funciones de aplciacion
static esp_err_t read_mpu_body(float accel_body[3], float gyro_body[3], int64_t *timestamp_us);
static bool attitude_sample_is_still(const float accel[3], const float gyro[3]);
static esp_err_t initialize_attitude(void);
static esp_err_t initialize_relative_height(void);
static FILE *open_csv_append(const char *path, const char *header, char *buffer, size_t buffer_size);
static void process_ack_command(uint16_t *last_command_id);
static int32_t scale_to_i32(double value, double scale);
static int16_t scale_to_i16(double value, double scale);
static void print_telemetry_serial(const telemetry_log_record_t *record);
static void print_ekf_serial(const ekf_log_record_t *record);
static int write_ekf_csv(FILE *file, const char *prefix, const ekf_log_record_t *record);
static void update_vertical_navigation(attitude_data_t *attitude);

// Prototipo de funciones de aplciacion
void app_main(void)
{
    // Creacion del Bus i2c
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    // Inicializaciond e sipsotibvos
    bmp180.bus_handle = bus_handle;
    mpu6050.bus_handle = bus_handle;

    nrf24l01.config = nrf24l01cfg;
    sd_card.config = sd_config;
    neo6m.config = neo6mcfg;
    mpu6050.config = mpu6050cfg;
    bmp180.config = bmp180cfg;

    ESP_ERROR_CHECK(neo6m_init(&neo6m));
    ESP_ERROR_CHECK(nrf24l01_init(&nrf24l01, &nrf24l01cfg));
    ESP_ERROR_CHECK(nrf24l01_set_tx_address(&nrf24l01,
                                            s_radio_address));
    /* Diagnostico de radio antes de calibrar sensores y crear las tareas. */
    ESP_ERROR_CHECK(nrf24l01_print_diagnostics(&nrf24l01));
    ESP_ERROR_CHECK(sd_init(&sd_card));
    ESP_ERROR_CHECK(bmp180_init(&bmp180));
    ESP_ERROR_CHECK(mpu6050_init(&mpu6050));

    // Inicio de Calibracion de el acelerometro y del giroscopio
    ESP_LOGI(TAG, "Mantener el CanSat inmovil y nivelado: calibrando MPU6050");
    ESP_ERROR_CHECK(mpu6050_calibrate_accel(&mpu6050));
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

    // Creacion de recursos RTOS
    Queue_GPS = xQueueCreate(GPS_QUEUE_LENGTH, sizeof(neo6m_data_t));
    Queue_data = xQueueCreate(TELEMETRY_QUEUE_LENGTH, sizeof(telemetry_log_record_t));
    Queue_radio = xQueueCreate(RADIO_QUEUE_LENGTH, sizeof(telemetry_data_t));
    Queue_Attitude = xQueueCreate(ATTITUDE_QUEUE_LENGTH, sizeof(attitude_data_t));
    Queue_EkfLog = xQueueCreate(EKF_LOG_QUEUE_LENGTH, sizeof(ekf_log_record_t));
    Queue_EkfSerial = xQueueCreate(EKF_SERIAL_QUEUE_LENGTH, sizeof(ekf_log_record_t));
    ESP_ERROR_CHECK(Queue_EkfSerial != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    Queue_Baro = xQueueCreate(BARO_QUEUE_LENGTH, sizeof(barometer_sample_t));
    ESP_ERROR_CHECK(Queue_Baro != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    // Registro de STATUS
    EventGroup_status = xEventGroupCreate();

    /*
    if ((Queue_GPS == NULL) || (Queue_data == NULL) || (Queue_radio == NULL) || (Queue_Attitude == NULL) || (Queue_EkfLog == NULL) || (EventGroup_status == NULL))
    {
        ESP_LOGE(TAG, "No se pudieron crear las colas");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }*/

    // Creacion de Tareas
    BaseType_t task_created = xTaskCreate(vTaskReadGPS, "gps_task", 1024 * 5, NULL, 2, NULL);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    task_created = xTaskCreate(vTaskAttitude, "attitude_task", 1024 * 6, NULL, 4, NULL);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    task_created = xTaskCreate(vTaskReadData, "data_task", 1024 * 6, NULL, 3, NULL);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    task_created = xTaskCreate(vTaskWriteSD, "sd_logger_task", 1024 * 8, NULL, 3, NULL);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    task_created = xTaskCreate(vTaskTransmitData, "nrf24_tx_task",
                               1024 * 5, NULL, 3, NULL);
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

    ESP_LOGI(TAG, "Actitud EKF: periodo=%lu ms, ejes body=[X,-Y,-Z], yaw relativo",
             (unsigned long)ATTITUDE_PERIOD_MS);

    while (1)
    {
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
                 * La recuperacion vuelve a fijar yaw relativo a cero.
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

        if ((xEventGroupGetBits(EventGroup_status) & SD_LOGGING_READY) != 0 && xQueueSend(Queue_EkfLog, &record, 0) != pdTRUE)
            dropped_records++;

        const TickType_t now = xTaskGetTickCount();
        if ((now - last_status) >= pdMS_TO_TICKS(SD_STATUS_PERIOD_MS))
        {
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

    telemetry_data_t telemetry;
    uint16_t last_command_id = 0U;

    while (1)
    {
        if (xQueueReceive(Queue_radio, &telemetry, portMAX_DELAY) != pdTRUE)
            continue;

        esp_err_t error = nrf24l01_transmit(&nrf24l01, &telemetry, sizeof(telemetry), NRF24_TX_TIMEOUT_MS);

        if (error == ESP_OK)
        {
            // ESP_LOGI(TAG, "NRF24 TX: seq=%lu, status=0x%02X",
            //          (unsigned long)telemetry.sequence, (unsigned)telemetry.status);
            process_ack_command(&last_command_id);
        }
        else
        {
            //ESP_LOGW(TAG, "NRF24 TX fallo: seq=%lu, %s", (unsigned long)telemetry.sequence, esp_err_to_name(error));
        }
    }
}

static void vTaskWriteSD(void *pVparameters)
{
    (void)pVparameters;

    if (!sd_card.mounted)
    {
        ESP_LOGW(TAG, "SD no montada; se elimina la tarea de almacenamiento");
        vTaskDelete(NULL);
        return;
    }

    static char telemetry_buffer[SD_FILE_BUFFER_SIZE];
    static char ekf_buffer[SD_FILE_BUFFER_SIZE];
    FILE *telemetry_file = open_csv_append(DATALOG_PATH,
                                           TELEMETRY_DATALOG_HEADER,
                                           telemetry_buffer,
                                           sizeof(telemetry_buffer));
    if (telemetry_file == NULL)
    {
        vTaskDelete(NULL);
        return;
    }

    FILE *ekf_file = open_csv_append(EKF_DATALOG_PATH, EKF_DATALOG_HEADER,
                                     ekf_buffer, sizeof(ekf_buffer));
    if (ekf_file == NULL)
    {
        fclose(telemetry_file);
        vTaskDelete(NULL);
        return;
    }

    xEventGroupSetBits(EventGroup_status, SD_LOGGING_READY);
    ESP_LOGI(TAG, "SD logger activo: %s y %s", DATALOG_PATH, EKF_DATALOG_PATH);

    TickType_t last_flush = xTaskGetTickCount();
    TickType_t last_status = last_flush;
    uint32_t telemetry_records = 0U;
    uint32_t ekf_records = 0U;
    telemetry_log_record_t telemetry_record;
    ekf_log_record_t ekf_record;

    while (1)
    {
        bool wrote_data = false;
        if (xQueueReceive(Queue_data, &telemetry_record, 0) == pdTRUE)
        {
            const telemetry_data_t *sample = &telemetry_record.data;
            const int written = fprintf(
                telemetry_file,
                "%lld,%lu,%.7f,%.7f,%.2f,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%u\n",
                (long long)telemetry_record.timestamp_us,
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
                (unsigned)sample->status,
                (unsigned)sample->satellites);

            if (written < 0)
            {
                ESP_LOGE(TAG, "Error escribiendo %s: errno=%d (%s)",
                         DATALOG_PATH, errno, strerror(errno));
                goto storage_error;
            }
            telemetry_records++;
            wrote_data = true;
        }

        /* Lotes limitados: atender ambas colas incluso si el EKF produce
         * muestras mientras se escribe. La tarea de actitud nunca espera SD.
         */
        for (uint32_t sample = 0; sample < SD_EKF_BATCH_SIZE; sample++)
        {
            if (xQueueReceive(Queue_EkfLog, &ekf_record, 0) != pdTRUE)
                break;

            const int written = write_ekf_csv(ekf_file, "", &ekf_record);

            if (written < 0)
            {
                ESP_LOGE(TAG, "Error escribiendo %s: errno=%d (%s)",
                         EKF_DATALOG_PATH, errno, strerror(errno));
                goto storage_error;
            }
            ekf_records++;
            wrote_data = true;
        }

        TickType_t now = xTaskGetTickCount();
        if ((now - last_flush) >= pdMS_TO_TICKS(SD_FLUSH_PERIOD_MS))
        {
            const bool telemetry_ok = (fflush(telemetry_file) == 0) && (fsync(fileno(telemetry_file)) == 0);
            const bool ekf_ok = (fflush(ekf_file) == 0) && (fsync(fileno(ekf_file)) == 0);
            last_flush = now;

            if (!telemetry_ok || !ekf_ok)
            {
                ESP_LOGE(TAG, "Fallo sincronizando los archivos SD");
                goto storage_error;
            }
        }

        if ((now - last_status) >= pdMS_TO_TICKS(SD_STATUS_PERIOD_MS))
        {
            /*
            ESP_LOGI(TAG,
                     "SD OK: telemetria=%lu (%ld bytes), EKF=%lu (%ld bytes)",
                     (unsigned long)telemetry_records,
                     ftell(telemetry_file), (unsigned long)ekf_records,
                     ftell(ekf_file));*/
            last_status = now;
        }

        if (!wrote_data)
            vTaskDelay(1);
    }

storage_error:
    xEventGroupClearBits(EventGroup_status, SD_LOGGING_READY);
    fclose(telemetry_file);
    fclose(ekf_file);
    ESP_LOGE(TAG, "Registro SD detenido; MPU/EKF y radio continuan activos");
    vTaskDelete(NULL);
}

static void vTaskReadData(void *pVparameters)
{
    (void)pVparameters;

    neo6m_data_t latest_gps = {0};
    bool gps_received = false;
    uint32_t sequence = 0U;
    TickType_t last_wake = xTaskGetTickCount();

    puts(TELEMETRY_SERIAL_HEADER);
    puts(EKF_SERIAL_HEADER);
    uint32_t last_ekf_sequence = 0U;
    bool ekf_serial_received = false;

    while (1)
    {
        while (xQueueReceive(Queue_GPS, &latest_gps, 0) == pdTRUE)
            gps_received = true;

        telemetry_log_record_t record = {
            .timestamp_us = esp_timer_get_time(),
            .data.sequence = sequence++,
        };

        const int64_t baro_start_us = esp_timer_get_time();
        esp_err_t bmp_error = bmp180_read_data(&bmp180);
        const int64_t baro_end_us = esp_timer_get_time();
        if (bmp_error == ESP_OK)
        {
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
        if (xQueuePeek(Queue_Attitude, &attitude, 0) == pdTRUE && attitude.valid)
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

            if (xQueueSend(Queue_data, &record, 0) != pdTRUE)
                ESP_LOGW(TAG, "Cola de telemetria llena; muestra descartada");
        }

        xQueueOverwrite(Queue_radio, &record.data);
        //print_telemetry_serial(&record);
        ekf_log_record_t ekf_record;
        if (xQueuePeek(Queue_EkfSerial, &ekf_record, 0) == pdTRUE
            && (!ekf_serial_received || ekf_record.sequence != last_ekf_sequence))
        {
            print_ekf_serial(&ekf_record);
            last_ekf_sequence = ekf_record.sequence;
            ekf_serial_received = true;
        }

        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(s_telemetry_period_ms));
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
    static bool anchored = false;
    static bool pending = false;
    static barometer_sample_t baro;
    static int64_t last_consumed_us = 0;
    static int64_t last_accepted_us = 0;
    const int64_t now_us = attitude->timestamp_us;
    attitude->vertical_valid = false;

    barometer_sample_t incoming;
    if (xQueueReceive(Queue_Baro, &incoming, 0) == pdTRUE
        && incoming.timestamp_us > last_consumed_us)
    {
        baro = incoming;
        pending = true;
    }
    if (!attitude->valid)
        return;

    const float acceleration_up = -attitude->linear_accel_nav_m_s2[2];
    if (!isfinite(acceleration_up))
        return;

    int64_t filter_us = altitude_kalman.diagnostics.last_predict_us;
    if (anchored && (now_us <= filter_us
                     || (float)(now_us - filter_us) * 1.0e-6f > ekf_qcfg.max_dt_s))
    {
        /* No integrar un hueco grande con una sola muestra. Esperar un nuevo
         * anclaje barometrico; la velocidad se reinicia con la configuracion.
         */
        anchored = false;
    }

    if (pending && baro.timestamp_us <= now_us)
    {
        pending = false;
        last_consumed_us = baro.timestamp_us;
        if (isfinite(baro.height_m)
            && now_us - baro.timestamp_us <= ALTITUDE_BARO_MAX_LAG_US)
        {
            esp_err_t error;
            if (!anchored)
            {
                /* El barometro ya llega relativo al arranque. Al recuperar el
                 * filtro se conserva ese origen, incluso si estamos en vuelo.
                 */
                error = altitude_kalman_reset(
                    &altitude_kalman, baro.height_m,
                    altitude_kalmancfg.initial_velocity_mps,
                    altitude_kalmancfg.initial_height_variance,
                    altitude_kalmancfg.initial_velocity_variance);
                if (error != ESP_OK)
                    return;
                anchored = true;
                filter_us = baro.timestamp_us;
                last_accepted_us = baro.timestamp_us;
                altitude_kalman.diagnostics.last_predict_us = filter_us;
            }
            else
            {
                /* Si la medicion cae en este intervalo, dividir la prediccion
                 * y corregir en su timestamp. Un retraso menor de 100 ms se
                 * aproxima al estado actual; no se rebobina ni duplica P.
                 */
                if (baro.timestamp_us > filter_us)
                {
                    error = altitude_kalman_predict(
                        &altitude_kalman, acceleration_up,
                        (float)(baro.timestamp_us - filter_us) * 1.0e-6f);
                    if (error != ESP_OK)
                    {
                        anchored = false;
                        return;
                    }
                    filter_us = baro.timestamp_us;
                    altitude_kalman.diagnostics.last_predict_us = filter_us;
                }
                error = altitude_kalman_correct_baro(&altitude_kalman, baro.height_m);
                if (error == ESP_OK)
                    last_accepted_us = baro.timestamp_us;
                else if (error != ALTITUDE_KALMAN_ERR_OUTLIER)
                {
                    anchored = false;
                    return;
                }
                /* Un outlier no reinicia el filtro ni deshabilita su compuerta. */
            }
        }
    }

    if (!anchored)
        return;
    if (now_us > filter_us)
    {
        const esp_err_t error = altitude_kalman_predict(
            &altitude_kalman, acceleration_up, (float)(now_us - filter_us) * 1.0e-6f);
        if (error != ESP_OK)
        {
            anchored = false;
            return;
        }
        altitude_kalman.diagnostics.last_predict_us = now_us;
    }

    attitude->height_m = altitude_kalman_get_height(&altitude_kalman);
    attitude->vertical_velocity_m_s = altitude_kalman_get_velocity(&altitude_kalman);
    attitude->vertical_valid = isfinite(attitude->height_m)
                              && isfinite(attitude->vertical_velocity_m_s)
                              && now_us - last_accepted_us <= ALTITUDE_BARO_MAX_AGE_US;
}

/* Una sola linea CSV por muestra, sin prefijo de ESP_LOG ni colores.
 * MATLAB puede seleccionar las filas que comienzan por TELEMETRY_SERIAL_PREFIX
 * y quitar ese primer campo; el resto coincide con telem.csv.
 * Se publica aunque falle el enlace de radio o el registro en SD.
 */
static void print_telemetry_serial(const telemetry_log_record_t *record)
{
    const telemetry_data_t *sample = &record->data;
    printf(TELEMETRY_SERIAL_PREFIX
           "%lld,%lu,%.7f,%.7f,%.2f,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%u\n",
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
           (unsigned)sample->status, (unsigned)sample->satellites);
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

static void process_ack_command(uint16_t *last_command_id)
{
    bool available = false;
    nrf24_pipe_t pipe = NRF24_PIPE_0;
    esp_err_t error = nrf24l01_data_available(
        &nrf24l01, &available, &pipe);

    if ((error != ESP_OK) || !available)
        return;

    nrf24_command_packet_t command = {0};
    size_t payload_length = 0U;
    error = nrf24l01_receive(&nrf24l01, &command, sizeof(command),
                             &payload_length, &pipe);
    if (error != ESP_OK)
    {
        ESP_LOGW(TAG, "No se pudo leer comando ACK: %s",
                 esp_err_to_name(error));
        return;
    }

    if (payload_length != sizeof(command))
    {
        ESP_LOGW(TAG, "Comando ACK invalido: %u bytes",
                 (unsigned)payload_length);
        return;
    }

    if (command.command_id == *last_command_id)
        return;

    *last_command_id = command.command_id;

    switch ((nrf24_command_code_t)command.command)
    {
    case NRF24_COMMAND_PING:
        ESP_LOGI(TAG, "PING recibido por ACK, id=%u", command.command_id);
        break;

    case NRF24_COMMAND_SET_TELEMETRY_PERIOD:
        if ((command.argument < TELEMETRY_PERIOD_MIN_MS) || (command.argument > TELEMETRY_PERIOD_MAX_MS))
        {
            ESP_LOGW(TAG, "Periodo rechazado: %lu ms",
                     (unsigned long)command.argument);
            break;
        }

        s_telemetry_period_ms = command.argument;
        ESP_LOGI(TAG, "Periodo de telemetria actualizado: %lu ms",
                 (unsigned long)s_telemetry_period_ms);
        break;

    default:
        ESP_LOGW(TAG, "Comando no soportado: %u",
                 (unsigned)command.command);
        break;
    }
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
