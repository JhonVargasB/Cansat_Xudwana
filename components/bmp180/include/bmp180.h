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
} bmp180_config_t;

typedef struct
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t i2c_dev;
    i2c_device_config_t dev_cfg;

    bmp180_calib_data_t calib;
    bmp180_uncomp_data_t uncomp;
    bmp180_config_t config;
    bmp180_data_t data;

    bool initialized;
} bmp180_t;

#define BMP180_DEFAULT_CONFIG()                                  \
    {                                                            \
        .device_address = BMP180_I2C_ADDRESS,                    \
        .scl_speed_hz = UINT32_C(100000),                        \
        .scl_wait_us = 0,                                        \
        .oversampling = BMP180_OVERSAMPLING_STANDARD,            \
        .sea_level_pressure_pa = BMP180_DEFAULT_SEA_LEVEL_PRESSURE_PA, \
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
 *   pressure: pascals
 *   altitude: meters relative to config.sea_level_pressure_pa
 */
esp_err_t bmp180_read_data(bmp180_t *device);

#ifdef __cplusplus
}
#endif

