#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "mpu6050.h"

#include "esp_timer.h"
#include "shared.h"

#define BUFFER_SAMPLES 10

typedef struct
{
    uint32_t id;
    int64_t time_us;
    float ax;
    float ay;
    float az;
} sample_t;

static const char *TAG = "main";

i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_1,
    .sda_io_num = PIN_SDA1_I2C,
    .scl_io_num = PIN_SCLK1_I2C,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = false,
};

i2c_master_bus_handle_t bus_handle;
mpu6050_t mpu6050 = {
    .config = MPU6050_DEFAULT_CONFIG(),
};

void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando aplicación MPU6050");
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    mpu6050.bus_handle = bus_handle;
    ESP_ERROR_CHECK(mpu6050_init(&mpu6050));
    // ESP_ERROR_CHECK(selftest_xg(&mpu6050));
    // ESP_ERROR_CHECK(selftest_yg(&mpu6050));
    // ESP_ERROR_CHECK(selftest_zg(&mpu6050));
    ESP_ERROR_CHECK(mpu6050_calibrate_accel(&mpu6050));
    ESP_ERROR_CHECK(mpu6050_calibrate_gyro(&mpu6050));
    ESP_LOGI(TAG, "MPU6050 listo para uso");
    sample_t buffer[BUFFER_SAMPLES];

    uint32_t sample_id = 0;
    int idx = 0;

    while (1)
    {
        esp_err_t err = mpu6050_read_accel(&mpu6050);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error leyendo acelerometro: %s",
                     esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        buffer[idx].id = sample_id++;
        buffer[idx].time_us = esp_timer_get_time();

        buffer[idx].ax = mpu6050.data.ax;
        buffer[idx].ay = mpu6050.data.ay;
        buffer[idx].az = mpu6050.data.az;

        idx++;

        if (idx >= BUFFER_SAMPLES)
        {
            for (int i = 0; i < BUFFER_SAMPLES; i++)
            {
                printf("%lu,%lld,%.6f,%.6f,%.6f\n",
                       (unsigned long)buffer[i].id,
                       (long long)buffer[i].time_us,
                       buffer[i].ax,
                       buffer[i].ay,
                       buffer[i].az);
            }

            idx = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(25));
    }
}
