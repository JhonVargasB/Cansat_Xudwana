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
esp_err_t selftest_xg(mpu6050_t *mpu6050);
esp_err_t selftest_yg(mpu6050_t *mpu6050);
esp_err_t selftest_zg(mpu6050_t *mpu6050);
esp_err_t mpu6050_calibrate_accel(mpu6050_t *mpu6050);
esp_err_t mpu6050_read_accel(mpu6050_t *mpu6050);
esp_err_t mpu6050_read_gyro(mpu6050_t *mpu6050);
esp_err_t mpu6050_calibrate_gyro(mpu6050_t *mpu6050);
