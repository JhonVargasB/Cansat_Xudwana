#include <limits.h>
#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "qmc5883p.h"

static const char *TAG = "qmc5883p";

static const float s_sensitivity_lsb_per_gauss[] = {1000.0f, 2500.0f, 3750.0f, 15000.0f};
static const unsigned s_output_rates_hz[] = {10, 50, 100, 200};
static const unsigned s_ranges_gauss[] = {30, 12, 8, 2};

/* Prototipos de funciones auxiliares. */
static bool config_is_valid(const qmc5883p_config_t *config);
static uint8_t make_control_1(const qmc5883p_config_t *config);
static esp_err_t read_registers(qmc5883p_t *device, uint8_t reg, uint8_t *data, size_t size);
static esp_err_t write_u8(qmc5883p_t *device, uint8_t reg, uint8_t value);
static esp_err_t reset_device(qmc5883p_t *device);
static int16_t decode_i16_le(const uint8_t *bytes);

esp_err_t qmc5883p_init(qmc5883p_t *device)
{
    if (device == NULL || device->bus_handle == NULL || !config_is_valid(&device->config))
        return ESP_ERR_INVALID_ARG;
    if (device->initialized || device->i2c_dev != NULL)
        return ESP_ERR_INVALID_STATE;

    memset(&device->data, 0, sizeof(device->data));
    device->chip_id = 0;
    device->dev_cfg = (i2c_device_config_t){
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device->config.device_address,
        .scl_speed_hz = device->config.scl_speed_hz,
        .scl_wait_us = device->config.scl_wait_us,
    };
    esp_err_t error = i2c_master_bus_add_device(device->bus_handle,
                                               &device->dev_cfg, &device->i2c_dev);
    if (error != ESP_OK) return error;

    bool identified = false;
    error = read_registers(device, QMC5883P_REG_CHIP_ID, &device->chip_id, 1);
    if (error != ESP_OK) goto fail;
    if (device->chip_id != QMC5883P_CHIP_ID)
    {
        ESP_LOGE(TAG, "CHIP_ID inesperado: 0x%02X (esperado 0x80)", device->chip_id);
        error = ESP_ERR_NOT_FOUND;
        goto fail;
    }
    identified = true;

    error = reset_device(device);
    if (error != ESP_OK) goto fail;

    /* Pasar por suspend antes de configurar el modo normal (QST, seccion 9.2.3). */
    error = write_u8(device, QMC5883P_REG_CONTROL_1, QMC5883P_MODE_SUSPEND);
    if (error != ESP_OK) goto fail;
    error = write_u8(device, QMC5883P_REG_AXIS_SIGN, QMC5883P_AXIS_SIGN_DEFAULT);
    if (error != ESP_OK) goto fail;

    /* SET/RESET MODE=00: set y reset activados; RNG ocupa bits 3:2. */
    const uint8_t control_2 = (uint8_t)((unsigned)device->config.range << 2);
    const uint8_t control_1 = make_control_1(&device->config);
    error = write_u8(device, QMC5883P_REG_CONTROL_2, control_2);
    if (error != ESP_OK) goto fail;
    vTaskDelay(1);
    uint8_t control_2_before_mode;
    error = read_registers(device, QMC5883P_REG_CONTROL_2, &control_2_before_mode, 1);
    if (error != ESP_OK) goto fail;
    ESP_LOGI(TAG, "CTRL2 en suspend: escrito=0x%02X leido=0x%02X",
             control_2, control_2_before_mode);
    error = write_u8(device, QMC5883P_REG_CONTROL_1, control_1);
    if (error != ESP_OK) goto fail;
    vTaskDelay(1);

    uint8_t readback[2];
    /* Direccionar cada control explicitamente. No asumir que una lectura
     * de dos bytes desde CTRL1 devuelve CTRL1 y CTRL2 consecutivamente.
     */
    error = read_registers(device, QMC5883P_REG_CONTROL_1, &readback[0], 1);
    if (error != ESP_OK) goto fail;
    error = read_registers(device, QMC5883P_REG_CONTROL_2, &readback[1], 1);
    if (error != ESP_OK) goto fail;
    ESP_LOGI(TAG, "Tras activar modo normal: CTRL1=0x%02X CTRL2=0x%02X",
             readback[0], readback[1]);
    if ((readback[1] & QMC5883P_CONTROL_2_DEFINED_MASK) != control_2)
    {
        /* Un unico reintento con el sensor ya activo; el rango se acepta
         * solamente si la nueva lectura lo confirma.
         */
        ESP_LOGW(TAG, "Reaplicando CTRL2 con el modo normal activo");
        error = write_u8(device, QMC5883P_REG_CONTROL_2, control_2);
        if (error != ESP_OK) goto fail;
        vTaskDelay(1);
        error = read_registers(device, QMC5883P_REG_CONTROL_2, &readback[1], 1);
        if (error != ESP_OK) goto fail;
        error = read_registers(device, QMC5883P_REG_CONTROL_1, &readback[0], 1);
        if (error != ESP_OK) goto fail;
    }
    if (readback[0] != control_1
        || (readback[1] & QMC5883P_CONTROL_2_DEFINED_MASK) != control_2)
    {
        ESP_LOGE(TAG, "Configuracion difiere: CTRL1=0x%02X (esperado 0x%02X), "
                     "CTRL2=0x%02X (esperado 0x%02X)",
                 readback[0], control_1, readback[1], control_2);
        error = ESP_ERR_INVALID_RESPONSE;
        goto fail;
    }

    device->sensitivity_lsb_per_gauss = s_sensitivity_lsb_per_gauss[device->config.range];
    device->initialized = true;
    ESP_LOGI(TAG, "Listo: addr=0x%02X ID=0x%02X I2C=%lu Hz ODR=%u Hz rango=+-%u G",
             device->config.device_address, device->chip_id,
             (unsigned long)device->config.scl_speed_hz,
             s_output_rates_hz[device->config.output_data_rate],
             s_ranges_gauss[device->config.range]);
    ESP_LOGI(TAG, "CTRL1=0x%02X CTRL2=0x%02X signos=0x%02X sensibilidad=%.0f LSB/G",
             readback[0], readback[1], QMC5883P_AXIS_SIGN_DEFAULT,
             (double)device->sensitivity_lsb_per_gauss);
    return ESP_OK;

fail:
    if (identified)
        (void)write_u8(device, QMC5883P_REG_CONTROL_1, QMC5883P_MODE_SUSPEND);
    if (i2c_master_bus_rm_device(device->i2c_dev) == ESP_OK)
        device->i2c_dev = NULL;
    else
        ESP_LOGE(TAG, "No se pudo retirar el dispositivo del bus tras el fallo");
    return error;
}

esp_err_t qmc5883p_deinit(qmc5883p_t *device)
{
    if (device == NULL) return ESP_ERR_INVALID_ARG;
    if (device->i2c_dev == NULL) return ESP_ERR_INVALID_STATE;

    if (device->initialized)
    {
        const uint8_t suspend = make_control_1(&device->config)
            & (uint8_t)~QMC5883P_CONTROL_1_MODE_MASK;
        const esp_err_t error = write_u8(device, QMC5883P_REG_CONTROL_1, suspend);
        if (error != ESP_OK) return error;
    }
    const esp_err_t error = i2c_master_bus_rm_device(device->i2c_dev);
    if (error == ESP_OK)
    {
        device->i2c_dev = NULL;
        device->initialized = false;
        device->data.valid = false;
    }
    return error;
}

esp_err_t qmc5883p_read_data(qmc5883p_t *device, uint32_t timeout_ms)
{
    if (device == NULL) return ESP_ERR_INVALID_ARG;
    if (!device->initialized || device->i2c_dev == NULL) return ESP_ERR_INVALID_STATE;

    device->data.valid = false;
    device->data.overflow = false;
    const int64_t deadline_us = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (true)
    {
        uint8_t status;
        esp_err_t error = read_registers(device, QMC5883P_REG_STATUS, &status, 1);
        if (error != ESP_OK) return error;
        device->data.status = status;
        if ((status & QMC5883P_STATUS_OVFL) != 0)
        {
            device->data.overflow = true;
            return QMC5883P_ERR_OVERFLOW;
        }
        if ((status & QMC5883P_STATUS_DRDY) != 0)
        {
            uint8_t bytes[6];
            const int64_t start_us = esp_timer_get_time();
            error = read_registers(device, QMC5883P_REG_X_LSB, bytes, sizeof(bytes));
            const int64_t end_us = esp_timer_get_time();
            if (error != ESP_OK) return error;

            qmc5883p_data_t sample = {
                .raw_x = decode_i16_le(&bytes[0]),
                .raw_y = decode_i16_le(&bytes[2]),
                .raw_z = decode_i16_le(&bytes[4]),
                .timestamp_us = start_us + (end_us - start_us) / 2,
                .status = status,
                .valid = true,
            };
            /* 1 G = 100 microteslas. */
            const float ut_per_lsb = 100.0f / device->sensitivity_lsb_per_gauss;
            sample.x_ut = (float)sample.raw_x * ut_per_lsb;
            sample.y_ut = (float)sample.raw_y * ut_per_lsb;
            sample.z_ut = (float)sample.raw_z * ut_per_lsb;
            sample.norm_ut = sqrtf(sample.x_ut * sample.x_ut
                                   + sample.y_ut * sample.y_ut
                                   + sample.z_ut * sample.z_ut);
            device->data = sample;
            return ESP_OK;
        }
        if (timeout_ms == 0) return ESP_ERR_NOT_FINISHED;
        if (esp_timer_get_time() >= deadline_us) return ESP_ERR_TIMEOUT;
        vTaskDelay(1);
    }
}

/* Definiciones de funciones auxiliares. */
static bool config_is_valid(const qmc5883p_config_t *config)
{
    return config != NULL && config->device_address == QMC5883P_I2C_ADDRESS
        && config->scl_speed_hz > 0 && config->scl_speed_hz <= QMC5883P_MAX_I2C_CLOCK_HZ
        && config->io_timeout_ms > 0 && config->io_timeout_ms <= INT_MAX
        && (unsigned)config->output_data_rate <= QMC5883P_ODR_200_HZ
        && (unsigned)config->range <= QMC5883P_RANGE_2_G
        && (unsigned)config->oversampling_1 <= QMC5883P_OSR1_1
        && (unsigned)config->oversampling_2 <= QMC5883P_OSR2_8;
}

static uint8_t make_control_1(const qmc5883p_config_t *config)
{
    return (uint8_t)(((unsigned)config->oversampling_2 << 6)
                   | ((unsigned)config->oversampling_1 << 4)
                   | ((unsigned)config->output_data_rate << 2)
                   | QMC5883P_MODE_NORMAL);
}

static esp_err_t read_registers(qmc5883p_t *device, uint8_t reg, uint8_t *data, size_t size)
{
    return i2c_master_transmit_receive(device->i2c_dev, &reg, 1, data, size,
                                       (int)device->config.io_timeout_ms);
}

static esp_err_t write_u8(qmc5883p_t *device, uint8_t reg, uint8_t value)
{
    const uint8_t bytes[] = {reg, value};
    return i2c_master_transmit(device->i2c_dev, bytes, sizeof(bytes),
                              (int)device->config.io_timeout_ms);
}

static int16_t decode_i16_le(const uint8_t *bytes)
{
    const uint16_t value = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
    return value <= INT16_MAX ? (int16_t)value : (int16_t)((int32_t)value - 65536);
}

static esp_err_t reset_device(qmc5883p_t *device)
{
    /* QST define SOFT_RST=1 como reset y 0 como funcionamiento normal;
     * no especifica auto-clear para este bit (si para SELF_TEST).
     * Dar un pulso y desactivarlo explicitamente, como el driver IIO propuesto
     * y probado en GY-271. Los 10 ms superan los 250 us del POR sin busy wait.
     */
    esp_err_t error = write_u8(device, QMC5883P_REG_CONTROL_2,
                                QMC5883P_CONTROL_2_SOFT_RESET);
    if (error != ESP_OK) return error;
    TickType_t delay_ticks = pdMS_TO_TICKS(10);
    if (delay_ticks == 0) delay_ticks = 1;
    vTaskDelay(delay_ticks);

    error = write_u8(device, QMC5883P_REG_CONTROL_2, 0);
    if (error != ESP_OK) return error;
    vTaskDelay(delay_ticks);

    uint8_t control_1, control_2;
    error = read_registers(device, QMC5883P_REG_CHIP_ID, &device->chip_id, 1);
    if (error != ESP_OK) return error;
    error = read_registers(device, QMC5883P_REG_CONTROL_1, &control_1, 1);
    if (error != ESP_OK) return error;
    error = read_registers(device, QMC5883P_REG_CONTROL_2, &control_2, 1);
    if (error != ESP_OK) return error;
    ESP_LOGI(TAG, "Reset 0x80 -> 0x00: ID=0x%02X CTRL1=0x%02X CTRL2=0x%02X",
             device->chip_id, control_1, control_2);
    if (device->chip_id != QMC5883P_CHIP_ID
        || (control_1 & QMC5883P_CONTROL_1_MODE_MASK) != QMC5883P_MODE_SUSPEND
        || (control_2 & QMC5883P_CONTROL_2_DEFINED_MASK) != 0)
    {
        ESP_LOGE(TAG, "El sensor no confirma suspend con reset desactivado");
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}
