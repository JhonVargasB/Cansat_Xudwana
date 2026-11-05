#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#include "bmp180_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /* Hardware configuration. The application owns and initializes the bus. */
    uint8_t device_address;
    uint32_t scl_speed_hz;
    uint32_t scl_wait_us;

    /* Sensor and altitude configuration. */
    bmp180_oversampling_t oversampling;
    double sea_level_pressure_pa;
    /* EMA de presion: fc >= 0, 0 desactiva el suavizado. */
    float pressure_filter_cutoff_hz;
} bmp180_config_t;

typedef struct
{
    float pressure_pa;
    int64_t timestamp_us;
    bool initialized;
} bmp180_pressure_filter_t;

typedef struct
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t i2c_dev;
    i2c_device_config_t dev_cfg;

    bmp180_calib_data_t calib;
    bmp180_uncomp_data_t uncomp;
    bmp180_config_t config;
    bmp180_data_t data;
    bmp180_pressure_filter_t pressure_filter;

    bool initialized;
} bmp180_t;

#define BMP180_DEFAULT_CONFIG()                                  \
    {                                                            \
        .device_address = BMP180_I2C_ADDRESS,                    \
        .scl_speed_hz = UINT32_C(100000),                        \
        .scl_wait_us = 0,                                        \
        .oversampling = BMP180_OVERSAMPLING_STANDARD,            \
        .sea_level_pressure_pa = BMP180_DEFAULT_SEA_LEVEL_PRESSURE_PA, \
        .pressure_filter_cutoff_hz = BMP180_DEFAULT_PRESSURE_FILTER_CUTOFF_HZ, \
    }

/**
 * @brief Adds the BMP180 to an existing I2C master bus and reads calibration.
 *
 * Set device->bus_handle and device->config before calling this function.
 */
esp_err_t bmp180_init(bmp180_t *device);

/** Remove the BMP180 device from its I2C bus. */
esp_err_t bmp180_deinit(bmp180_t *device);

/**
 * @brief Perform temperature and pressure conversions and compensate them.
 *
 * Results:
 *   temperature: degrees Celsius
 *   pressure: pascals compensados originales, sin EMA (se conservan).
 *   pressure_filtered_pa: EMA float con dt real entre lecturas validas.
 *   altitude: metros calculados con pressure_filtered_pa y sea_level_pressure_pa.
 *   timestamp_us, pressure_filter_dt_s/alpha: para analisis de la adquisicion.
 * Solo una conversion nueva y valida actualiza el EMA. La primera se copia
 * directamente (dt=0, alpha=1). Un fallo conserva el estado y marca valid=false;
 * la siguiente lectura usa el tiempo transcurrido desde la ultima valida.
 * El EMA es causal O(1), no anade esperas a las conversiones existentes.
 * Serializar externamente si varias tareas acceden a una misma instancia.
 */
esp_err_t bmp180_read_data(bmp180_t *device);

#ifdef __cplusplus
}
#endif

