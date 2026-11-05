#pragma once

#include <stdbool.h>

#include "esp_log.h"
#include "driver/i2c_master.h"
#include "mpu6050_defs.h"


typedef struct
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t i2c_dev;
    i2c_device_config_t dev_cfg;
    mpu6050_calib_data calib;
    mpu6050_config_t config;
    mpu6050_scale scale;
    mpu6050_data_t data;
    bool initialized;
} mpu6050_t;

#define MPU6050_DEFAULT_CONFIG()                 \
    {                                            \
        .device_address = MPU6050_I2C_ADDR_PRIM, \
        .scl_speed_hz = UINT32_C(100000),        \
        .scl_wait_us = 0,                        \
        .smpltr_div = UINT8_C(0),                \
        .dlpf_cfg = BWA_94_BWG_98,               \
        .gfs_sel = GFS_SEL_250,                  \
        .afs_sel = AFS_SEL_8G,                   \
        .clk_sel = PLL_X_GYRO_REF_CLK,           \
    }

esp_err_t mpu6050_init(mpu6050_t *mpu6050);
esp_err_t mpu6050_deinit(mpu6050_t *mpu6050);
/* Habilita acceso directo del ESP32 al bus auxiliar del MPU.
 * Al habilitar, desactiva I2C_MST_EN y activa I2C_BYPASS_EN.
 * Al deshabilitar, solo quita el bypass; no reactiva el maestro auxiliar.
 * Conserva los otros bits y comprueba los registros despues de escribir.
 * Llamar con el dispositivo inicializado y sin accesos concurrentes al MPU.
 */
esp_err_t mpu6050_set_i2c_bypass(mpu6050_t *mpu6050, bool enabled);
esp_err_t selftest_xg(mpu6050_t *mpu6050);
esp_err_t selftest_yg(mpu6050_t *mpu6050);
esp_err_t selftest_zg(mpu6050_t *mpu6050);
esp_err_t mpu6050_calibrate_accel(mpu6050_t *mpu6050);

/* Calibracion en reposo con la fuerza especifica esperada en ejes sensor,
 * expresada en g y de norma 1. Permite montar el MPU en otra orientacion.
 * La API anterior conserva la referencia [0,0,+1]. No estima escala ni
 * desalineacion: solo offsets para esta pose conocida.
 */
esp_err_t mpu6050_calibrate_accel_reference(mpu6050_t *mpu6050,
                                            const float reference_sensor_g[3]);
esp_err_t mpu6050_read_accel(mpu6050_t *mpu6050);
esp_err_t mpu6050_read_gyro(mpu6050_t *mpu6050);
esp_err_t mpu6050_calibrate_gyro(mpu6050_t *mpu6050);
