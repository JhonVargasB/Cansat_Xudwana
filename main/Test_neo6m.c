#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "neo6m.h"

static const char *TAG = "test_neo6m";

void app_main(void)
{
    neo6m_t gps = {
        .config = NEO6M_DEFAULT_CONFIG(),
    };

    esp_err_t err = neo6m_init(&gps);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo iniciar NEO-6M: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "NEO-6M listo: UART%d TX=GPIO%d RX=GPIO%d, %lu bps",
             (int)gps.config.uart_num,
             (int)gps.config.tx_pin,
             (int)gps.config.rx_pin,
             (unsigned long)gps.config.baud_rate);

    while (true) {
        err = neo6m_update(&gps, 1000);
        if (err == ESP_OK) {
            neo6m_print_data(&gps);
        } else if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "Esperando una trama GPS valida");
        } else {
            ESP_LOGE(TAG, "Error de lectura: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
