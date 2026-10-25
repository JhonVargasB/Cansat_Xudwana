#pragma once
#include <stdbool.h>
#include <stdio.h>

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "esp_err.h"
#include "esp_log.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"

#include "shared.h"
typedef struct
{
    gpio_num_t mosi_pin;
    gpio_num_t miso_pin;
    gpio_num_t sclk_pin;
    gpio_num_t cs_pin;

    const char *mount_point;
    int max_files;
    uint32_t max_freq_khz;
    size_t allocation_unit_size;
    bool format_if_mount_failed;

} sd_config_t;

typedef struct
{
    sd_config_t config;

    sdmmc_card_t *card;
    sdmmc_host_t host;

    bool mounted;
    bool bus_initialized;

} sd_card_t;

#define SD_DEFAULT_CONFIG()                  \
    ((sd_config_t){                          \
        .mosi_pin = (gpio_num_t)PIN_SD_MOSI,     \
        .miso_pin = (gpio_num_t)PIN_SD_MISO,     \
        .sclk_pin = (gpio_num_t)PIN_SD_SCK,      \
        .cs_pin = (gpio_num_t)PIN_SD_CS,         \
        .mount_point = "/sdcard",            \
        .max_files = 5,                      \
        .max_freq_khz = 4*1000,                \
        .allocation_unit_size = 16 * 1024,   \
        .format_if_mount_failed = false      \
    })

esp_err_t sd_init(sd_card_t *sd);
esp_err_t sd_read_file(const char *path, char *buffer, size_t buffer_size);
esp_err_t sd_deinit(sd_card_t *sd);
esp_err_t sd_write_file(const char *path, const char *data);
