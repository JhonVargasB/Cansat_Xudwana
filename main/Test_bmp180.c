#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"

#include "bmp180.h"
#include "shared.h"

static const char *TAG = "bmp180_test";

#define BMP180_SAMPLE_PERIOD_MS UINT32_C(100)

void app_main(void)
{
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_1,
        .sda_io_num = PIN_SDA1_I2C,
        .scl_io_num = PIN_SCLK1_I2C,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t i2c_bus = NULL;
    esp_err_t err = i2c_new_master_bus(&bus_config, &i2c_bus);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo iniciar I2C: %s", esp_err_to_name(err));
        return;
    }

    bmp180_t bmp180 = {
        .bus_handle = i2c_bus,
        .config = BMP180_DEFAULT_CONFIG(),
    };

    err = bmp180_init(&bmp180);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo iniciar BMP180: %s", esp_err_to_name(err));
        i2c_del_master_bus(i2c_bus);
        return;
    }

    ESP_LOGI(TAG,
             "BMP180 listo: direccion=0x%02X SDA=GPIO%d SCL=GPIO%d",
             bmp180.config.device_address,
             (int)PIN_SDA1_I2C,
             (int)PIN_SCLK1_I2C);

    TickType_t last_wake_time = xTaskGetTickCount();

    while (true)
    {
        err = bmp180_read_data(&bmp180);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG,
                     "Temperatura=%.2f C, Presion=%.2f Pa, Altitud=%.2f m",
                     bmp180.data.temperature,
                     bmp180.data.pressure,
                     bmp180.data.altitude);
        }
        else
        {
            ESP_LOGE(TAG, "Error de lectura: %s", esp_err_to_name(err));
        }

        xTaskDelayUntil(&last_wake_time,
                        pdMS_TO_TICKS(BMP180_SAMPLE_PERIOD_MS));
    }
}
