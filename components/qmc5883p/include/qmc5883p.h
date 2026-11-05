#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "qmc5883p_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define QMC5883P_ERR_OVERFLOW ESP_ERR_INVALID_RESPONSE

typedef struct
{
    uint8_t device_address;
    uint32_t scl_speed_hz;
    uint32_t scl_wait_us;
    uint32_t io_timeout_ms;
    qmc5883p_odr_t output_data_rate;
    qmc5883p_range_t range;
    qmc5883p_osr1_t oversampling_1;
    qmc5883p_osr2_t oversampling_2;
} qmc5883p_config_t;

typedef struct
{
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;
    float x_ut;
    float y_ut;
    float z_ut;
    float norm_ut;
    /* Tiempo aproximado de lectura, no timestamp interno del sensor. */
    int64_t timestamp_us;
    uint8_t status;
    bool overflow;
    bool valid;
} qmc5883p_data_t;

typedef struct
{
    /* La aplicacion crea y conserva el bus; habilita bypass si hace falta. */
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t i2c_dev;
    i2c_device_config_t dev_cfg;
    qmc5883p_config_t config;
    qmc5883p_data_t data;
    float sensitivity_lsb_per_gauss;
    uint8_t chip_id;
    bool initialized;
} qmc5883p_t;

#define QMC5883P_DEFAULT_CONFIG()                       \
    {                                                  \
        .device_address = QMC5883P_I2C_ADDRESS,         \
        .scl_speed_hz = UINT32_C(100000),              \
        .scl_wait_us = 0,                              \
        .io_timeout_ms = UINT32_C(50),                 \
        .output_data_rate = QMC5883P_ODR_100_HZ,        \
        .range = QMC5883P_RANGE_8_G,                    \
        .oversampling_1 = QMC5883P_OSR1_8,              \
        .oversampling_2 = QMC5883P_OSR2_8,              \
    }

/* Objeto inicialmente a cero, salvo bus_handle y config.
 * Verifica CHIP_ID, reinicia, configura modo normal y comprueba CONTROL_1/2.
 * No modifica el MPU ni crea/destruye el bus. No cambiar config tras init.
 */
esp_err_t qmc5883p_init(qmc5883p_t *device);

/* Suspende las mediciones y retira el dispositivo del bus compartido. */
esp_err_t qmc5883p_deinit(qmc5883p_t *device);

/* Lee STATUS una vez por sondeo (DRDY/OVFL se borran al leerlo) y luego
 * X/Y/Z en una unica transferencia, little-endian con signo.
 * timeout_ms=0: un solo sondeo; sin dato devuelve ESP_ERR_NOT_FINISHED.
 * timeout_ms>0: espera DRDY, cediendo CPU; sin dato devuelve ESP_ERR_TIMEOUT.
 * El timeout es para esperar datos; cada transaccion usa io_timeout_ms.
 * QMC5883P_ERR_OVERFLOW indica saturacion y descarta esa muestra.
 * Solo ESP_OK publica valores nuevos y data.valid=true. En otro caso se
 * conservan valores previos con valid=false. Controlar desde una sola tarea
 * o serializar externamente. No aplica calibracion ni rotacion al marco body.
 */
esp_err_t qmc5883p_read_data(qmc5883p_t *device, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
