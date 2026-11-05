/* Pruebas del codigo de produccion con I2C y reloj simulados. Incluir el .c
 * permite comprobar tambien el EMA privado, sin exponer una API de pruebas.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../../components/bmp180/bmp180.c"

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); exit(1); \
} } while (0)

static int64_t mock_time_us = 1000000;
static uint32_t mock_up = 23843;
static uint8_t mock_oss;
static bool fail_next_read;
static unsigned delay_calls, temperature_commands, pressure_commands;
/* Coeficientes y UT/UP del ejemplo de compensacion de Bosch (OSS=0). */
static const int32_t calibration_words[11] = {
    408, -72, -14383, 32741, 32757, 23153, 6190, 4, -32768, -8711, 2868
};

int64_t esp_timer_get_time(void) { return mock_time_us; }
void vTaskDelay(uint32_t ticks) { (void)ticks; ++delay_calls; }
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus,
                                    const i2c_device_config_t *config,
                                    i2c_master_dev_handle_t *device)
{
    (void)config;
    *device = bus;
    return ESP_OK;
}
esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t device)
{
    (void)device;
    return ESP_OK;
}
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t device,
                              const uint8_t *data, size_t size, int timeout_ms)
{
    (void)device; (void)timeout_ms;
    CHECK(size == 2);
    if (data[0] == BMP180_REG_CONTROL)
    {
        if (data[1] == BMP180_TEMPERATURE_COMMAND) ++temperature_commands;
        else { ++pressure_commands; mock_oss = data[1] >> 6; }
    }
    return ESP_OK;
}
esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t device,
                                      const uint8_t *tx, size_t tx_size,
                                      void *rx, size_t size, int timeout_ms)
{
    (void)device; (void)timeout_ms;
    CHECK(tx_size == 1);
    if (fail_next_read) { fail_next_read = false; return ESP_ERR_TIMEOUT; }
    uint8_t *bytes = rx;
    memset(bytes, 0, size);
    if (*tx == BMP180_REG_CHIP_ID) bytes[0] = BMP180_CHIP_ID_VALUE;
    else if (*tx == BMP180_REG_CALIB_START)
    {
        CHECK(size == 22);
        for (unsigned i = 0; i < 11; ++i)
        {
            bytes[2 * i] = (uint16_t)calibration_words[i] >> 8;
            bytes[2 * i + 1] = (uint16_t)calibration_words[i] & 255;
        }
    }
    else if (*tx == BMP180_REG_DATA_MSB && size == 2)
    { bytes[0] = 27898 >> 8; bytes[1] = 27898 & 255; }
    else if (*tx == BMP180_REG_DATA_MSB && size == 3)
    {
        uint32_t adc = mock_up << (8 - mock_oss);
        bytes[0] = adc >> 16; bytes[1] = (adc >> 8) & 255; bytes[2] = adc & 255;
    }
    else CHECK(*tx == BMP180_REG_CONTROL);
    return ESP_OK;
}

static void test_ema_signals(void)
{
    bmp180_pressure_filter_t filter = {0};
    float dt, alpha;
    CHECK(bmp180_filter_pressure(&filter, 75000, 0, 3, &dt, &alpha) == ESP_OK);
    CHECK(filter.pressure_pa == 75000 && dt == 0 && alpha == 1);
    int64_t t = 0;
    unsigned delays_before = delay_calls;
    for (int i = 0; i < 1000; ++i)
    {
        t += 80000 + (i % 3) * 20000;
        CHECK(bmp180_filter_pressure(&filter, 75000, t, 3, &dt, &alpha) == ESP_OK);
        CHECK(filter.pressure_pa == 75000);
    }
    CHECK(delay_calls == delays_before);
    printf("PASS first sample, constant input, variable dt, no added waits\n");

    filter = (bmp180_pressure_filter_t){0};
    CHECK(bmp180_filter_pressure(&filter, 75000, 0, 3, &dt, &alpha) == ESP_OK);
    CHECK(bmp180_filter_pressure(&filter, 74900, 100000, 3, &dt, &alpha) == ESP_OK);
    CHECK(fabsf(alpha - 0.8481642f) < 0.000002f);
    CHECK(fabsf(dt - 0.1f) < 0.000001f);
    float first = (75000 - filter.pressure_pa) / 100;
    CHECK(first > 0.84f && first < 0.86f);
    CHECK(bmp180_filter_pressure(&filter, 74900, 200000, 3, &dt, &alpha) == ESP_OK);
    float second = (75000 - filter.pressure_pa) / 100;
    CHECK(second > 0.97f && second < 1.0f);
    printf("Step at 10 Hz: %.2f%% after 100 ms, %.2f%% after 200 ms\n", first * 100, second * 100);

    filter = (bmp180_pressure_filter_t){0};
    CHECK(bmp180_filter_pressure(&filter, 75000, 0, 3, &dt, &alpha) == ESP_OK);
    double raw_energy = 0, filtered_energy = 0;
    for (int i = 1; i <= 500; ++i)
    {
        float noise = i % 2 ? 10.0f : -10.0f; /* Fluctuacion a Nyquist (5 Hz). */
        CHECK(bmp180_filter_pressure(&filter, 75000 + noise, i * INT64_C(100000), 3, &dt, &alpha) == ESP_OK);
        if (i > 20)
        {
            raw_energy += noise * noise;
            double error = filter.pressure_pa - 75000;
            filtered_energy += error * error;
        }
    }
    double ratio = sqrt(filtered_energy / raw_energy);
    CHECK(ratio > 0.73 && ratio < 0.75);
    printf("PASS rapid fluctuation: RMS output/input=%.4f\n", ratio);

    bmp180_pressure_filter_t saved = filter;
    CHECK(bmp180_filter_pressure(&filter, 75100, filter.timestamp_us, 3, &dt, &alpha) == ESP_ERR_INVALID_STATE);
    CHECK(memcmp(&saved, &filter, sizeof(filter)) == 0);
    CHECK(bmp180_filter_pressure(&filter, NAN, filter.timestamp_us + 100000, 3, &dt, &alpha) == ESP_ERR_INVALID_ARG);
    CHECK(memcmp(&saved, &filter, sizeof(filter)) == 0);
    CHECK(bmp180_filter_pressure(&filter, 74900, filter.timestamp_us + 50000, 3, &dt, &alpha) == ESP_OK);
    CHECK(fabsf(alpha - 0.6103389f) < 0.000002f);
    CHECK(bmp180_filter_pressure(&filter, 74800, filter.timestamp_us + 200000, 3, &dt, &alpha) == ESP_OK);
    CHECK(fabsf(alpha - 0.9769459f) < 0.000002f);
    printf("PASS duplicate/invalid sample rejection and real elapsed time\n");
}

static void test_driver_acquisition(void)
{
    bmp180_t sensor = {.bus_handle = (void *)1, .config = BMP180_DEFAULT_CONFIG()};
    sensor.config.oversampling = BMP180_OVERSAMPLING_ULTRA_LOW_POWER;
    CHECK(bmp180_init(&sensor) == ESP_OK);
    unsigned delays_before = delay_calls;
    CHECK(bmp180_read_data(&sensor) == ESP_OK);
    CHECK(sensor.data.valid && sensor.data.temperature == 15.0);
    CHECK(sensor.data.pressure == 69964.0);
    CHECK(sensor.data.pressure_filtered_pa == (float)sensor.data.pressure);
    CHECK(sensor.data.pressure_filter_dt_s == 0 && sensor.data.pressure_filter_alpha == 1);
    CHECK(delay_calls - delays_before == 2);

    mock_up = 24000; mock_time_us += 100000;
    delays_before = delay_calls;
    CHECK(bmp180_read_data(&sensor) == ESP_OK);
    CHECK(sensor.data.pressure_filtered_pa != (float)sensor.data.pressure);
    CHECK(fabs(sensor.data.altitude - bmp180_calculate_altitude(
        sensor.data.pressure_filtered_pa, sensor.config.sea_level_pressure_pa)) < 1.0e-9);
    CHECK(delay_calls - delays_before == 2);
    double original = sensor.data.pressure;
    bmp180_pressure_filter_t saved = sensor.pressure_filter;
    mock_time_us += 100000; fail_next_read = true;
    CHECK(bmp180_read_data(&sensor) == ESP_ERR_TIMEOUT);
    CHECK(!sensor.data.valid && sensor.data.pressure == original);
    CHECK(memcmp(&saved, &sensor.pressure_filter, sizeof(saved)) == 0);
    mock_time_us += 100000;
    CHECK(bmp180_read_data(&sensor) == ESP_OK);
    CHECK(fabsf(sensor.data.pressure_filter_dt_s - 0.2f) < 0.000001f);

    sensor.config.pressure_filter_cutoff_hz = 0;
    mock_time_us += 100000; delays_before = delay_calls;
    CHECK(bmp180_read_data(&sensor) == ESP_OK);
    CHECK(sensor.data.pressure == original && sensor.data.pressure_filtered_pa == original);
    CHECK(delay_calls - delays_before == 2);
    CHECK(bmp180_deinit(&sensor) == ESP_OK);
    CHECK(!sensor.pressure_filter.initialized);
    sensor.config.pressure_filter_cutoff_hz = 3;
    CHECK(bmp180_init(&sensor) == ESP_OK);
    mock_time_us += 100000;
    CHECK(bmp180_read_data(&sensor) == ESP_OK);
    CHECK(sensor.data.pressure_filtered_pa == sensor.data.pressure);
    printf("PASS actual driver: raw preserved, filtered altitude, I2C failure, bypass and reinit\n");
    printf("Acquisition: %u temperature commands, %u pressure commands; two existing waits per successful read\n",
           temperature_commands, pressure_commands);
}

int main(void)
{
    test_ema_signals();
    test_driver_acquisition();
    puts("ALL PASS BMP180 EMA (production C, mocked sensor and clock)");
    return 0;
}
