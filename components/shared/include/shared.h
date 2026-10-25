#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/uart.h"

// Pines para la memoria SD en SPI2
/*
#define PIN_SD_MOSI GPIO_NUM_12
#define PIN_SD_MISO GPIO_NUM_10
#define PIN_SD_CS GPIO_NUM_13
#define PIN_SD_SCK GPIO_NUM_11
*/

#define PIN_SD_MOSI GPIO_NUM_11
#define PIN_SD_MISO GPIO_NUM_9
#define PIN_SD_CS GPIO_NUM_12
#define PIN_SD_SCK GPIO_NUM_10

#define HOST_SD_SPI2 SPI2_HOST

/* nRF24L01 transmisor: SPI3 de la placa CanSat. */
/*
#define PIN_NRF24_TX_MOSI GPIO_NUM_4
#define PIN_NRF24_TX_MISO GPIO_NUM_5
#define PIN_NRF24_TX_SCLK GPIO_NUM_6
#define PIN_NRF24_TX_CSN GPIO_NUM_15
#define PIN_NRF24_TX_CE GPIO_NUM_7
#define PIN_NRF24_TX_IRQ GPIO_NUM_NC

*/

#define PIN_NRF24_TX_MOSI GPIO_NUM_16
#define PIN_NRF24_TX_MISO GPIO_NUM_15
#define PIN_NRF24_TX_SCLK GPIO_NUM_6
#define PIN_NRF24_TX_CSN GPIO_NUM_4
#define PIN_NRF24_TX_CE GPIO_NUM_7
#define PIN_NRF24_TX_IRQ GPIO_NUM_NC


#define HOST_NRF24_TX_SPI3 SPI3_HOST

/* nRF24L01 receptor: mismo cableado en una segunda placa ESP32-S3. */
#define PIN_NRF24_RX_MOSI GPIO_NUM_9
#define PIN_NRF24_RX_MISO GPIO_NUM_43
#define PIN_NRF24_RX_SCLK GPIO_NUM_5
#define PIN_NRF24_RX_CSN GPIO_NUM_7
#define PIN_NRF24_RX_CE GPIO_NUM_4
#define PIN_NRF24_RX_IRQ GPIO_NUM_NC
#define HOST_NRF24_RX_SPI3 SPI3_HOST

// pines para el BUS I2C 1
#define PIN_SDA1_I2C GPIO_NUM_14
#define PIN_SCLK1_I2C GPIO_NUM_21
#define I2C_1 I2C_NUM_1

// pines UART0 para el GPS NEO6M
/*
#define PIN_GPS_UART_TX GPIO_NUM_17
#define PIN_GPS_UART_RX GPIO_NUM_18

 */

 #define PIN_GPS_UART_TX GPIO_NUM_48
#define PIN_GPS_UART_RX GPIO_NUM_47
#define UART_GPS UART_NUM_1

/* Buzzer pasivo: salida PWM. GPIO8 libre en el cableado actual. */
#define PIN_BUZZER GPIO_NUM_38

/* Capacidad de las colas de la aplicacion. */
#define GPS_QUEUE_LENGTH UINT32_C(1)
#define TELEMETRY_QUEUE_LENGTH UINT32_C(16)
#define RADIO_QUEUE_LENGTH UINT32_C(1)
#define ATTITUDE_QUEUE_LENGTH UINT32_C(1)
#define EKF_LOG_QUEUE_LENGTH UINT32_C(64)
#define EKF_SERIAL_QUEUE_LENGTH UINT32_C(1)
#define BARO_QUEUE_LENGTH UINT32_C(1)

/* Periodos, tiempos de espera e inicializacion de actitud. */
#define ATTITUDE_PERIOD_MS UINT32_C(10)
#define ATTITUDE_INITIAL_SAMPLES UINT32_C(100)
#define ATTITUDE_MAX_AGE_US INT64_C(200000)
#define ATTITUDE_STILL_GYRO_RAD_S 0.08726646f
#define ATTITUDE_STILL_ACCEL_TOLERANCE 0.15f
#define ATTITUDE_ACCEL_BIAS_PROCESS_NOISE_STD 0.0005f
#define ALTITUDE_BARO_MAX_LAG_US INT64_C(100000)
#define ALTITUDE_BARO_MAX_AGE_US INT64_C(1000000)
#define ALTITUDE_REFERENCE_SAMPLES UINT32_C(20)
#define ALTITUDE_REFERENCE_PERIOD_MS UINT32_C(50)
#define TELEMETRY_PERIOD_DEFAULT_MS UINT32_C(100)
#define TELEMETRY_PERIOD_MIN_MS UINT32_C(100)
#define TELEMETRY_PERIOD_MAX_MS UINT32_C(10000)
#define NRF24_EXAMPLE_TELEMETRY_PERIOD_DEFAULT_MS UINT32_C(1000)
#define NRF24_TX_TIMEOUT_MS UINT32_C(100)
#define GPS_UPDATE_TIMEOUT_MS UINT32_C(1000)

/* Escritura y estado del registro SD. */
#define SD_FLUSH_PERIOD_MS UINT32_C(1000)
#define SD_STATUS_PERIOD_MS UINT32_C(5000)
#define SD_EKF_BATCH_SIZE UINT32_C(16)
#define SD_FILE_BUFFER_SIZE UINT32_C(4096)
#define SD_LOGGING_READY (UINT32_C(1) << 0)

/* Inicializadores de la direccion de radio y de la orientacion del MPU. */
#define NRF24_TELEMETRY_ADDRESS {0x01, 0xC2, 0xC2, 0xC2, 0xC2}

/*
 * Montaje asumido: X del MPU hacia delante, Y hacia la izquierda, Z arriba.
 * Rotacion propia de 180 grados en X: body FRD = [X_mpu,-Y_mpu,-Z_mpu].
 * Aplicar la MISMA rotacion a accel y gyro. Ajustar aqui si cambia el montaje.
 * Con la calibracion actual (+g en Z del MPU), body recibe [0,0,-g].
 */
#define MPU_TO_BODY_ROTATION \
    {                        \
        {1.0f, 0.0f, 0.0f}, \
        {0.0f, -1.0f, 0.0f}, \
        {0.0f, 0.0f, -1.0f}, \
    }

/* Escalas utilizadas en la trama de telemetria. */
#define TELEMETRY_COORDINATE_SCALE 10000000.0
#define TELEMETRY_CENTI_SCALE 100.0
#define RAD_TO_DEG (57.29577951308232)
#define DEG_TO_RAD (0.017453292519943295f)

/* Bits de telemetry_data_t.status. */
#define TELEMETRY_STATUS_GPS_FIX (UINT8_C(1) << 0)
#define TELEMETRY_STATUS_BMP180_OK (UINT8_C(1) << 1)
#define TELEMETRY_STATUS_MPU6050_OK (UINT8_C(1) << 2)
#define TELEMETRY_STATUS_NEO6M_OK (UINT8_C(1) << 3)
#define TELEMETRY_STATUS_SD_OK (UINT8_C(1) << 4)
#define TELEMETRY_STATUS_LOW_BATTERY (UINT8_C(1) << 5)
#define TELEMETRY_STATUS_DESCENDING (UINT8_C(1) << 6)
#define TELEMETRY_STATUS_PARACHUTE (UINT8_C(1) << 7)

/* Bits de ekf_log_record_t.status; accepted_axes usa bits 0:X, 1:Y, 2:Z. */
#define EKF_LOG_MPU_OK (UINT8_C(1) << 0)
#define EKF_LOG_ATTITUDE_VALID (UINT8_C(1) << 1)
#define EKF_LOG_DT_REJECTED (UINT8_C(1) << 2)
#define EKF_LOG_ERROR (UINT8_C(1) << 3)
#define EKF_LOG_INITIALIZING (UINT8_C(1) << 4)

/* Dimension del registro; mantiene shared independiente del componente EKF. */
#define EKF_LOG_STATE_DIM 7

/*
 * Trama comun de telemetria (32 bytes, limite del payload nRF24L01+).
 *
 * latitude_e7/longitude_e7 : grados * 1e7
 * altitude_cm              : altura relativa al arranque, metros * 100
 * vertical_speed_cms       : m/s * 100
 * temperature_centi_c      : grados Celsius * 100
 * *_centi_deg              : grados * 100
 */
typedef struct
{
    uint32_t sequence;

    int32_t latitude_e7;
    int32_t longitude_e7;
    int32_t altitude_cm;
    uint32_t pressure_pa;

    int16_t vertical_speed_cms;
    int16_t temperature_centi_c;

    uint16_t yaw_centi_deg;
    int16_t roll_centi_deg;
    int16_t pitch_centi_deg;

    uint8_t status;
    uint8_t satellites;
} telemetry_data_t;

_Static_assert(sizeof(telemetry_data_t) == 32U, "telemetry_data_t debe ocupar exactamente 32 bytes");

/** Registro de una muestra de telemetría para almacenamiento continuo. */
typedef struct
{
    int64_t timestamp_us;
    telemetry_data_t data;
} telemetry_log_record_t;

/* Altura relativa al arranque; timestamp aproximado de la conversion. */
typedef struct
{
    int64_t timestamp_us;
    float height_m;
} barometer_sample_t;

/* Ultima estimacion publicada por la tarea de actitud. Ejes de navegacion NED. */
typedef struct
{
    int64_t timestamp_us;
    float quaternion[4];
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float linear_accel_nav_m_s2[3];
    bool valid;
    /* Altura relativa al arranque y velocidad vertical positiva hacia arriba. */
    float height_m;
    float vertical_velocity_m_s;
    bool vertical_valid;
} attitude_data_t;

/* Registro de una iteracion del filtro para analisis en SD. */
typedef struct
{
    attitude_data_t attitude;
    uint32_t sequence;
    float dt_s;
    float accel_body_m_s2[3];
    float gyro_body_rad_s[3];
    float accel_bias_m_s2[3];
    float innovation_m_s2[3];
    float covariance_diagonal[EKF_LOG_STATE_DIM];
    float accel_norm_m_s2;
    float quaternion_norm;
    uint32_t update_us;
    uint32_t dropped_records;
    esp_err_t error;
    uint8_t accepted_axes;
    uint8_t status;
} ekf_log_record_t;

/* Rutas y cabeceras de los archivos CSV. */
#define DATALOG_PATH "/sdcard/telem.csv"

#define DATALOG_HEADER                                             \
    "sequence,latitude_deg,longitude_deg,altitude_m,pressure_pa,"  \
    "vertical_speed_m_s,temperature_c,yaw_deg,roll_deg,pitch_deg," \
    "status,satellites"

#define TELEMETRY_DATALOG_HEADER "timestamp_us," DATALOG_HEADER
#define TELEMETRY_SERIAL_PREFIX "TELEM,"
#define TELEMETRY_SERIAL_HEADER "TELEM_HEADER," TELEMETRY_DATALOG_HEADER

#define EKF_DATALOG_PATH "/sdcard/ekf.csv"
#define EKF_DATALOG_HEADER                                                   \
    "timestamp_us,sequence,dt_s,ax_body_m_s2,ay_body_m_s2,az_body_m_s2,"       \
    "gx_body_rad_s,gy_body_rad_s,gz_body_rad_s,qw,qx,qy,qz,"                  \
    "roll_rad,pitch_rad,yaw_rad,bax_m_s2,bay_m_s2,baz_m_s2,"                  \
    "innovation_x_m_s2,innovation_y_m_s2,innovation_z_m_s2,"                 \
    "linear_an_m_s2,linear_ae_m_s2,linear_ad_m_s2,"                           \
    "p_qw,p_qx,p_qy,p_qz,p_bax,p_bay,p_baz,accel_norm_m_s2,quaternion_norm," \
    "accepted_axes,status,error,update_us,dropped_records"
#define EKF_SERIAL_PREFIX "EKF,"
#define EKF_SERIAL_HEADER \
    "EKF_HEADER,timestamp_us,sequence,roll_deg,pitch_deg,yaw_deg,linear_ad_m_s2,valid," \
    "height_m,vertical_velocity_m_s,vertical_valid"

#define NAVIGATION_DATALOG_PATH "/sdcard/nav.csv"

#define NAVIGATION_DATALOG_HEADER                                         \
    "timestamp_us,sequence,temperature_c,pressure_pa,estimated_height_m," \
    "yaw_rad,pitch_rad,roll_rad,vertical_velocity_m_s,accel_n_m_s2,"      \
    "accel_e_m_s2,accel_d_m_s2"
