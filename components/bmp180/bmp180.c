#include "bmp180.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BMP180_I2C_TIMEOUT_MS 50

static const char *TAG = "bmp180";

/* Prototipos de funciones auxiliares del filtro. */
static esp_err_t bmp180_filter_pressure(bmp180_pressure_filter_t *state,
                                        float pressure_pa, int64_t timestamp_us,
                                        float cutoff_hz, float *dt_s, float *alpha);

static uint16_t bmp180_u16_be(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static int16_t bmp180_s16_be(const uint8_t *data)
{
    return (int16_t)bmp180_u16_be(data);
}

static void bmp180_delay_ms(uint32_t delay_ms)
{
    /*
     * Adding one tick prevents a sub-tick conversion delay from becoming zero
     * when the FreeRTOS tick period is greater than one millisecond.
     */
    vTaskDelay(pdMS_TO_TICKS(delay_ms) + 1U);
}

static esp_err_t bmp180_read(const bmp180_t *device, uint8_t reg,
                             void *data, size_t data_size)
{
    if ((device == NULL) || (device->i2c_dev == NULL)
        || (data == NULL) || (data_size == 0U))
    {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_transmit_receive(device->i2c_dev,
                                       &reg,
                                       sizeof(reg),
                                       data,
                                       data_size,
                                       BMP180_I2C_TIMEOUT_MS);
}

static esp_err_t bmp180_write_u8(const bmp180_t *device, uint8_t reg,
                                 uint8_t value)
{
    if ((device == NULL) || (device->i2c_dev == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t transaction[] = {reg, value};
    return i2c_master_transmit(device->i2c_dev,
                               transaction,
                               sizeof(transaction),
                               BMP180_I2C_TIMEOUT_MS);
}

static esp_err_t bmp180_device_create(bmp180_t *device)
{
    device->dev_cfg = (i2c_device_config_t){
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device->config.device_address,
        .scl_speed_hz = device->config.scl_speed_hz,
        .scl_wait_us = device->config.scl_wait_us,
    };

    esp_err_t err = i2c_master_bus_add_device(device->bus_handle,
                                               &device->dev_cfg,
                                               &device->i2c_dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo agregar el dispositivo I2C 0x%02X: %s",
                 device->config.device_address, esp_err_to_name(err));
    }

    return err;
}

static bool bmp180_calibration_is_valid(const uint8_t *raw)
{
    for (size_t offset = 0; offset < BMP180_CALIBRATION_DATA_SIZE;
         offset += 2U)
    {
        uint16_t word = bmp180_u16_be(&raw[offset]);
        if ((word == 0U) || (word == UINT16_MAX))
        {
            return false;
        }
    }

    return true;
}

static esp_err_t bmp180_read_calibration(bmp180_t *device)
{
    uint8_t raw[BMP180_CALIBRATION_DATA_SIZE] = {0};
    esp_err_t err = bmp180_read(device,
                                BMP180_REG_CALIB_START,
                                raw,
                                sizeof(raw));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo leer la calibracion: %s",
                 esp_err_to_name(err));
        return err;
    }

    if (!bmp180_calibration_is_valid(raw))
    {
        ESP_LOGE(TAG, "Los coeficientes de calibracion no son validos");
        return ESP_ERR_INVALID_RESPONSE;
    }

    device->calib = (bmp180_calib_data_t){
        .ac1 = bmp180_s16_be(&raw[0]),
        .ac2 = bmp180_s16_be(&raw[2]),
        .ac3 = bmp180_s16_be(&raw[4]),
        .ac4 = bmp180_u16_be(&raw[6]),
        .ac5 = bmp180_u16_be(&raw[8]),
        .ac6 = bmp180_u16_be(&raw[10]),
        .b1 = bmp180_s16_be(&raw[12]),
        .b2 = bmp180_s16_be(&raw[14]),
        .mb = bmp180_s16_be(&raw[16]),
        .mc = bmp180_s16_be(&raw[18]),
        .md = bmp180_s16_be(&raw[20]),
    };

    return ESP_OK;
}

static esp_err_t bmp180_wait_conversion(bmp180_t *device,
                                        uint32_t conversion_time_ms)
{
    bmp180_delay_ms(conversion_time_ms);

    uint8_t control = 0;
    esp_err_t err = bmp180_read(device,
                                BMP180_REG_CONTROL,
                                &control,
                                sizeof(control));
    if (err != ESP_OK)
    {
        return err;
    }

    return ((control & BMP180_CONTROL_SCO_MASK) == 0U)
               ? ESP_OK
               : ESP_ERR_TIMEOUT;
}

static esp_err_t bmp180_read_uncompensated_temperature(
    bmp180_t *device, uint32_t *temperature)
{
    esp_err_t err = bmp180_write_u8(device,
                                    BMP180_REG_CONTROL,
                                    BMP180_TEMPERATURE_COMMAND);
    if (err != ESP_OK)
    {
        return err;
    }

    err = bmp180_wait_conversion(device, BMP180_TEMPERATURE_TIME_MS);
    if (err != ESP_OK)
    {
        return err;
    }

    uint8_t raw[2] = {0};
    err = bmp180_read(device, BMP180_REG_DATA_MSB, raw, sizeof(raw));
    if (err == ESP_OK)
    {
        *temperature = bmp180_u16_be(raw);
    }

    return err;
}

static uint32_t bmp180_pressure_conversion_time_ms(
    bmp180_oversampling_t oversampling)
{
    static const uint8_t conversion_times[] = {
        BMP180_PRESSURE_TIME_ULP_MS,
        BMP180_PRESSURE_TIME_STD_MS,
        BMP180_PRESSURE_TIME_HR_MS,
        BMP180_PRESSURE_TIME_UHR_MS,
    };

    return conversion_times[(uint8_t)oversampling];
}

static esp_err_t bmp180_read_uncompensated_pressure(
    bmp180_t *device, uint32_t *pressure)
{
    const uint8_t oss = (uint8_t)device->config.oversampling;
    const uint8_t command = (uint8_t)(BMP180_PRESSURE_COMMAND
                                     | (oss << BMP180_CONTROL_OSS_POS));

    esp_err_t err = bmp180_write_u8(device, BMP180_REG_CONTROL, command);
    if (err != ESP_OK)
    {
        return err;
    }

    err = bmp180_wait_conversion(
        device, bmp180_pressure_conversion_time_ms(device->config.oversampling));
    if (err != ESP_OK)
    {
        return err;
    }

    uint8_t raw[3] = {0};
    err = bmp180_read(device, BMP180_REG_DATA_MSB, raw, sizeof(raw));
    if (err == ESP_OK)
    {
        uint32_t adc = ((uint32_t)raw[0] << 16)
                       | ((uint32_t)raw[1] << 8)
                       | raw[2];
        *pressure = adc >> (8U - oss);
    }

    return err;
}

static esp_err_t bmp180_compensate_temperature(
    const bmp180_t *device, int32_t *b5, double *temperature_c)
{
    const int64_t x1 =
        (((int64_t)device->uncomp.temperature - device->calib.ac6)
         * device->calib.ac5)
        >> 15;
    const int64_t denominator = x1 + device->calib.md;

    if (denominator == 0)
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const int64_t x2 = ((int64_t)device->calib.mc * 2048) / denominator;
    const int64_t compensated_b5 = x1 + x2;

    if ((compensated_b5 < INT32_MIN) || (compensated_b5 > INT32_MAX))
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    *b5 = (int32_t)compensated_b5;
    *temperature_c = (double)(((int64_t)*b5 + 8) >> 4) / 10.0;
    return ESP_OK;
}

static esp_err_t bmp180_compensate_pressure(
    const bmp180_t *device, int32_t b5, double *pressure_pa)
{
    const uint8_t oss = (uint8_t)device->config.oversampling;
    const int64_t b6 = (int64_t)b5 - 4000;
    int64_t x1 = ((int64_t)device->calib.b2 * ((b6 * b6) >> 12)) >> 11;
    int64_t x2 = ((int64_t)device->calib.ac2 * b6) >> 11;
    int64_t x3 = x1 + x2;
    const int64_t b3 =
        (((((int64_t)device->calib.ac1 * 4) + x3)
          * ((int64_t)1 << oss))
         + 2)
        >> 2;

    x1 = ((int64_t)device->calib.ac3 * b6) >> 13;
    x2 = ((int64_t)device->calib.b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = (x1 + x2 + 2) >> 2;

    const int64_t b4_base = x3 + 32768;
    if (b4_base <= 0)
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const uint64_t b4 =
        ((uint64_t)device->calib.ac4 * (uint64_t)b4_base) >> 15;
    const int64_t pressure_delta =
        (int64_t)device->uncomp.pressure - b3;

    if ((b4 == 0U) || (pressure_delta < 0))
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const uint64_t b7 =
        (uint64_t)pressure_delta * (uint64_t)(50000U >> oss);
    int64_t pressure;

    if (b7 < UINT32_C(0x80000000))
    {
        pressure = (int64_t)((b7 * 2U) / b4);
    }
    else
    {
        pressure = (int64_t)((b7 / b4) * 2U);
    }

    x1 = (pressure >> 8) * (pressure >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * pressure) >> 16;
    pressure += (x1 + x2 + 3791) >> 4;

    if (pressure <= 0)
    {
        return ESP_ERR_INVALID_RESPONSE;
    }

    *pressure_pa = (double)pressure;
    return ESP_OK;
}

static double bmp180_calculate_altitude(double pressure_pa,
                                        double sea_level_pressure_pa)
{
    return 44330.0
           * (1.0 - pow(pressure_pa / sea_level_pressure_pa,
                        1.0 / 5.255));
}

esp_err_t bmp180_init(bmp180_t *device)
{
    if ((device == NULL) || (device->bus_handle == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (device->initialized || (device->i2c_dev != NULL))
    {
        return ESP_ERR_INVALID_STATE;
    }

    if ((device->config.device_address != BMP180_I2C_ADDRESS)
        || (device->config.scl_speed_hz == 0U)
        || (device->config.scl_speed_hz > BMP180_MAX_I2C_CLOCK_HZ)
        || ((uint8_t)device->config.oversampling
            > BMP180_OVERSAMPLING_ULTRA_HIGH_RESOLUTION)
        || !isfinite(device->config.sea_level_pressure_pa)
        || (device->config.sea_level_pressure_pa <= 0.0)
        || !isfinite(device->config.pressure_filter_cutoff_hz)
        || (device->config.pressure_filter_cutoff_hz < 0.0f))
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = bmp180_device_create(device);
    if (err != ESP_OK)
    {
        return err;
    }

    /* The datasheet requires 10 ms after power-up before the first command. */
    bmp180_delay_ms(BMP180_STARTUP_TIME_MS);

    err = bmp180_write_u8(device,
                          BMP180_REG_SOFT_RESET,
                          BMP180_SOFT_RESET_COMMAND);
    if (err == ESP_OK)
    {
        bmp180_delay_ms(BMP180_STARTUP_TIME_MS);
    }

    uint8_t chip_id = 0;
    if (err == ESP_OK)
    {
        err = bmp180_read(device,
                          BMP180_REG_CHIP_ID,
                          &chip_id,
                          sizeof(chip_id));
    }

    if ((err == ESP_OK) && (chip_id != BMP180_CHIP_ID_VALUE))
    {
        ESP_LOGE(TAG, "Chip ID inesperado: 0x%02X", chip_id);
        err = ESP_ERR_NOT_FOUND;
    }

    if (err == ESP_OK)
    {
        err = bmp180_read_calibration(device);
    }

    if (err != ESP_OK)
    {
        esp_err_t remove_err = i2c_master_bus_rm_device(device->i2c_dev);
        if (remove_err != ESP_OK)
        {
            ESP_LOGW(TAG, "No se pudo retirar el dispositivo I2C: %s",
                     esp_err_to_name(remove_err));
        }
        device->i2c_dev = NULL;
        return err;
    }

    device->pressure_filter = (bmp180_pressure_filter_t){0};
    device->data = (bmp180_data_t){0};
    device->initialized = true;
    ESP_LOGI(TAG, "BMP180 inicializado en 0x%02X, OSS=%u EMA fc=%.2f Hz",
             device->config.device_address,
             (unsigned)device->config.oversampling,
             (double)device->config.pressure_filter_cutoff_hz);
    return ESP_OK;
}

esp_err_t bmp180_deinit(bmp180_t *device)
{
    if (device == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!device->initialized || (device->i2c_dev == NULL))
    {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = i2c_master_bus_rm_device(device->i2c_dev);
    if (err == ESP_OK)
    {
        device->i2c_dev = NULL;
        device->initialized = false;
        device->data.valid = false;
        device->pressure_filter = (bmp180_pressure_filter_t){0};
    }

    return err;
}

esp_err_t bmp180_read_data(bmp180_t *device)
{
    if (device == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    device->data.valid = false;

    if (!device->initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (((uint8_t)device->config.oversampling
         > BMP180_OVERSAMPLING_ULTRA_HIGH_RESOLUTION)
        || !isfinite(device->config.sea_level_pressure_pa)
        || (device->config.sea_level_pressure_pa <= 0.0)
        || !isfinite(device->config.pressure_filter_cutoff_hz)
        || (device->config.pressure_filter_cutoff_hz < 0.0f))
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t uncompensated_temperature = 0;
    uint32_t uncompensated_pressure = 0;

    esp_err_t err = bmp180_read_uncompensated_temperature(
        device, &uncompensated_temperature);
    if (err != ESP_OK)
    {
        return err;
    }

    err = bmp180_read_uncompensated_pressure(
        device, &uncompensated_pressure);
    if (err != ESP_OK)
    {
        return err;
    }
    const int64_t timestamp_us = esp_timer_get_time();

    device->uncomp.temperature = uncompensated_temperature;
    device->uncomp.pressure = uncompensated_pressure;

    int32_t b5 = 0;
    double temperature_c = 0.0;
    double pressure_pa = 0.0;

    err = bmp180_compensate_temperature(device, &b5, &temperature_c);
    if (err != ESP_OK)
    {
        return err;
    }

    err = bmp180_compensate_pressure(device, b5, &pressure_pa);
    if (err != ESP_OK)
    {
        return err;
    }

    /* Solo confirmar estado y muestra si conversion, compensacion y filtro
     * completos son validos. Nunca filtrar de nuevo device->data.pressure.
     */
    bmp180_pressure_filter_t filter = device->pressure_filter;
    float dt_s, alpha;
    err = bmp180_filter_pressure(&filter, (float)pressure_pa, timestamp_us,
                                  device->config.pressure_filter_cutoff_hz,
                                  &dt_s, &alpha);
    if (err != ESP_OK) return err;
    const double altitude = bmp180_calculate_altitude(
        (double)filter.pressure_pa, device->config.sea_level_pressure_pa);
    if (!isfinite(temperature_c) || !isfinite(altitude))
        return ESP_ERR_INVALID_RESPONSE;
    device->pressure_filter = filter;
    device->data = (bmp180_data_t){
        .temperature = temperature_c,
        .pressure = pressure_pa,
        .altitude = altitude,
        .pressure_filtered_pa = filter.pressure_pa,
        .pressure_filter_dt_s = dt_s,
        .pressure_filter_alpha = alpha,
        .timestamp_us = timestamp_us,
        .valid = true,
    };

    return ESP_OK;
}

static esp_err_t bmp180_filter_pressure(bmp180_pressure_filter_t *state,
                                        float pressure_pa, int64_t timestamp_us,
                                        float cutoff_hz, float *dt_s, float *alpha)
{
    if (!isfinite(pressure_pa) || pressure_pa <= 0.0f || timestamp_us < 0
        || !isfinite(cutoff_hz) || cutoff_hz < 0.0f)
        return ESP_ERR_INVALID_ARG;
    if (state->initialized && timestamp_us <= state->timestamp_us)
        return ESP_ERR_INVALID_STATE; /* No procesar dos veces el mismo instante. */

    *dt_s = state->initialized
        ? (float)(timestamp_us - state->timestamp_us) * 1.0e-6f : 0.0f;
    *alpha = 1.0f;
    if (!state->initialized || cutoff_hz == 0.0f)
        state->pressure_pa = pressure_pa;
    else
    {
        *alpha = 1.0f - expf(-6.2831853071795864769f * cutoff_hz * *dt_s);
        state->pressure_pa += *alpha * (pressure_pa - state->pressure_pa);
    }
    state->timestamp_us = timestamp_us;
    state->initialized = true;
    return ESP_OK;
}
