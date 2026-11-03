#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "buzzer.h"
#include "shared.h"

#define BUZZER_TEST_TONE_MS UINT32_C(500)
#define BUZZER_TEST_PAUSE_MS UINT32_C(500)
#define BUZZER_TEST_CYCLE_PAUSE_MS UINT32_C(2000)

static const char *TAG = "test_buzzer";
static buzzer_t s_buzzer = {0};

/* Prototipos de tareas. */
static void vTaskTestBuzzer(void *argument);

/* Prototipos de funciones auxiliares. */
static esp_err_t init_buzzer(void);

void app_main(void)
{
    ESP_ERROR_CHECK(init_buzzer());

    ESP_LOGI(TAG, "Buzzer listo: GPIO%d", (int)s_buzzer.config.pin);
    ESP_ERROR_CHECK(xTaskCreate(vTaskTestBuzzer, "test_buzzer", 3072,
                               NULL, 2, NULL) == pdPASS
                        ? ESP_OK : ESP_ERR_NO_MEM);
}

/* Tareas. */
static void vTaskTestBuzzer(void *argument)
{
    (void)argument;
    const uint32_t frequencies_hz[] = {250, 500, 1024};

    while (true)
    {
        for (size_t i = 0; i < sizeof(frequencies_hz) / sizeof(frequencies_hz[0]); ++i)
        {
            ESP_LOGI(TAG, "Tono: %lu Hz", (unsigned long)frequencies_hz[i]);
            ESP_ERROR_CHECK(buzzer_set_frequency(&s_buzzer, frequencies_hz[i]));
            vTaskDelay(pdMS_TO_TICKS(BUZZER_TEST_TONE_MS));

            ESP_ERROR_CHECK(buzzer_off(&s_buzzer));
            ESP_LOGI(TAG, "Buzzer apagado");
            vTaskDelay(pdMS_TO_TICKS(BUZZER_TEST_PAUSE_MS));
        }

        vTaskDelay(pdMS_TO_TICKS(BUZZER_TEST_CYCLE_PAUSE_MS));
    }
}

/* Definiciones de funciones auxiliares. */
static esp_err_t init_buzzer(void)
{
    const buzzer_config_t config = BUZZER_DEFAULT_CONFIG();
    return buzzer_init(&s_buzzer, &config);
}
