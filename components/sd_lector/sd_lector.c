
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include "sd_lector.h"

static const char *TAG = "sd_card";

esp_err_t sd_write_file(const char *path, const char *data)
{
    if (path == NULL || data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    FILE *f = fopen(path, "a");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "No se pudo abrir %s: errno=%d (%s)", path, errno, strerror(errno));
        return ESP_FAIL;
    }

    if (fputs(data, f) == EOF)
    {
        ESP_LOGE(TAG, "No se pudo escribir %s: errno=%d (%s)", path, errno, strerror(errno));
        fclose(f);
        return ESP_FAIL;
    }

    if (fclose(f) != 0)
    {
        ESP_LOGE(TAG, "No se pudo cerrar %s: errno=%d (%s)", path, errno, strerror(errno));
        return ESP_FAIL;
    }

    return ESP_OK;
}


esp_err_t sd_deinit(sd_card_t *sd)
{
    if (sd == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (sd->mounted && sd->card != NULL)
    {
        esp_vfs_fat_sdcard_unmount(sd->config.mount_point, sd->card);
        sd->card = NULL;
        sd->mounted = false;
    }

    if (sd->bus_initialized)
    {
        esp_err_t ret = spi_bus_free(sd->host.slot);
        if (ret != ESP_OK)
        {
            return ret;
        }

        sd->bus_initialized = false;
    }

    return ESP_OK;
}


esp_err_t sd_read_file(const char *path, char *buffer, size_t buffer_size)
{
    if (path == NULL || buffer == NULL || buffer_size == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    FILE *f = fopen(path, "r");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "No se pudo abrir %s: errno=%d (%s)", path, errno, strerror(errno));
        return ESP_FAIL;
    }

    if (fgets(buffer, buffer_size, f) == NULL)
    {
        fclose(f);
        return ESP_FAIL;
    }

    fclose(f);

    char *pos = strchr(buffer, '\n');
    if (pos != NULL)
    {
        *pos = '\0';
    }

    return ESP_OK;
}



esp_err_t sd_init(sd_card_t *sd)
{
    if (sd == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (sd->config.mount_point == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    if (sd->config.max_freq_khz < 400 ||
        sd->config.max_freq_khz > 20000)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (sd->mounted || sd->bus_initialized)
    {
        esp_err_t deinit_ret = sd_deinit(sd);
        if (deinit_ret != ESP_OK)
        {
            ESP_LOGE(TAG, "No se pudo reiniciar SD: %s", esp_err_to_name(deinit_ret));
            return deinit_ret;
        }
    }

    sd->card = NULL;
    sd->mounted = false;
    sd->bus_initialized = false;
    esp_err_t ret;

    sd->host = (sdmmc_host_t)SDSPI_HOST_DEFAULT();
    sd->host.slot = SPI2_HOST;
    sd->host.max_freq_khz = sd->config.max_freq_khz;

    ESP_LOGI(TAG, "SD SPI: MOSI=%d MISO=%d SCK=%d CS=%d, max=%lu kHz",
             sd->config.mosi_pin,
             sd->config.miso_pin,
             sd->config.sclk_pin,
             sd->config.cs_pin,
             (unsigned long)sd->config.max_freq_khz);

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = sd->config.mosi_pin,
        .miso_io_num = sd->config.miso_pin,
        .sclk_io_num = sd->config.sclk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    gpio_set_direction(sd->config.cs_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(sd->config.cs_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(20));

    ret = spi_bus_initialize(sd->host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "spi_bus_initialize fallo: %s", esp_err_to_name(ret));
        return ret;
    }

    sd->bus_initialized = true;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = sd->config.cs_pin;
    slot_config.host_id = sd->host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = sd->config.format_if_mount_failed,
        .max_files = sd->config.max_files,
        .allocation_unit_size = sd->config.allocation_unit_size,
    };

    ret = esp_vfs_fat_sdspi_mount(
        sd->config.mount_point,
        &sd->host,
        &slot_config,
        &mount_config,
        &sd->card);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_vfs_fat_sdspi_mount fallo: %s", esp_err_to_name(ret));
        spi_bus_free(sd->host.slot);
        sd->bus_initialized = false;
        sd->card = NULL;
        return ret;
    }

    sd->mounted = true;

    return ESP_OK;
}
