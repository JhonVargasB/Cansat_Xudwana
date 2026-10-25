#include "esp_err.h"
#include "esp_log.h"

#include "shared.h"
#include "sd_lector.h"

static const char *TAG = "test_storage";

static sd_card_t sd = {0};

static esp_err_t sd_prepare_csv(void)
{
    char first_line[256] = {0};
    esp_err_t ret = sd_read_file(DATALOG_PATH, first_line, sizeof(first_line));

    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Archivo existente: %s", DATALOG_PATH);
        ESP_LOGI(TAG, "Primera linea: %s", first_line);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Creando archivo CSV: %s", DATALOG_PATH);
    return sd_write_file(DATALOG_PATH, DATALOG_HEADER "\n");
}

void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando prueba de SD");

    sd.config = SD_DEFAULT_CONFIG();

    esp_err_t ret = sd_init(&sd);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo inicializar SD: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "SD montada en %s", sd.config.mount_point);

    ret = sd_prepare_csv();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo preparar CSV: %s", esp_err_to_name(ret));
        sd_deinit(&sd);
        return;
    }

    ret = sd_write_file(DATALOG_PATH,
                        "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n");
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo escribir fila de prueba: %s", esp_err_to_name(ret));
        sd_deinit(&sd);
        return;
    }

    ESP_LOGI(TAG, "Fila de prueba escrita en %s", DATALOG_PATH);

    char buffer[256] = {0};
    ret = sd_read_file(DATALOG_PATH, buffer, sizeof(buffer));
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Lectura OK: %s", buffer);
    }
    else
    {
        ESP_LOGE(TAG, "No se pudo leer CSV: %s", esp_err_to_name(ret));
    }

    ret = sd_deinit(&sd);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo desmontar SD: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Prueba de SD finalizada");
}
