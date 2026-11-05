#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "shared.h"
#include "mpu6050.h"

#define QMC5883P_ADDRESS UINT8_C(0x2C)
#define QMC5883P_REG_CHIP_ID UINT8_C(0x00)
#define QMC5883P_CHIP_ID UINT8_C(0x80)

static const char *TAG = "test_i2c";
static i2c_master_bus_handle_t s_i2c_bus;

/* Prototipos de tareas. */
static void vTaskScanI2C(void *argument);

/* Prototipos de funciones auxiliares. */
static esp_err_t init_i2c(void);
static esp_err_t scan_i2c(const char *label, bool detected[128]);
static esp_err_t identify_magnetometer(void);

void app_main(void)
{
    ESP_ERROR_CHECK(init_i2c());
    ESP_ERROR_CHECK(xTaskCreate(vTaskScanI2C, "i2c_scanner", 4096,
                               NULL, 2, NULL) == pdPASS
                        ? ESP_OK : ESP_ERR_NO_MEM);
}

/* Tareas. */
static void vTaskScanI2C(void *argument)
{
    (void)argument;
    bool without_bypass[128] = {0};
    bool with_bypass[128] = {0};
    mpu6050_t mpu = {
        .bus_handle = s_i2c_bus,
        .config = MPU6050_DEFAULT_CONFIG(),
    };

    ESP_LOGI(TAG, "Prueba I2C: SDA=GPIO%d SCL=GPIO%d",
             (int)PIN_SDA1_I2C, (int)PIN_SCLK1_I2C);
    ESP_ERROR_CHECK(mpu6050_init(&mpu));
    ESP_ERROR_CHECK(mpu6050_set_i2c_bypass(&mpu, false));
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_ERROR_CHECK(scan_i2c("SIN BYPASS", without_bypass));

    ESP_ERROR_CHECK(mpu6050_set_i2c_bypass(&mpu, true));
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_ERROR_CHECK(scan_i2c("CON BYPASS", with_bypass));

    unsigned added = 0;
    for (unsigned address = 0x08; address <= 0x77; ++address)
    {
        if (with_bypass[address] && !without_bypass[address])
        {
            ESP_LOGI(TAG, "Nueva direccion con bypass: 0x%02X (posible sensor auxiliar)",
                     address);
            ++added;
        }
    }
    if (added == 0)
        ESP_LOGW(TAG, "No aparecieron direcciones nuevas al activar bypass");

    if (with_bypass[QMC5883P_ADDRESS])
    {
        const esp_err_t error = identify_magnetometer();
        if (error != ESP_OK)
            ESP_LOGE(TAG, "No se pudo identificar el magnetometro: %s",
                     esp_err_to_name(error));
    }
    else
    {
        ESP_LOGW(TAG, "No responde el candidato QMC5883P en 0x%02X", QMC5883P_ADDRESS);
    }

    ESP_ERROR_CHECK(mpu6050_set_i2c_bypass(&mpu, false));
    ESP_ERROR_CHECK(mpu6050_deinit(&mpu));
    ESP_ERROR_CHECK(i2c_del_master_bus(s_i2c_bus));
    ESP_LOGI(TAG, "Prueba terminada; bypass desactivado");
    vTaskDelete(NULL);
}

/* Definiciones de funciones auxiliares. */
static esp_err_t identify_magnetometer(void)
{
    const i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = QMC5883P_ADDRESS,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t magnetometer = NULL;
    esp_err_t error = i2c_master_bus_add_device(s_i2c_bus, &config, &magnetometer);
    if (error != ESP_OK)
        return error;

    /* Leer identificacion sin cambiar los registros de configuracion del chip. */
    const uint8_t reg = QMC5883P_REG_CHIP_ID;
    uint8_t chip_id = 0;
    error = i2c_master_transmit_receive(magnetometer, &reg, 1, &chip_id, 1, 50);
    if (error == ESP_OK)
    {
        ESP_LOGI(TAG, "Identificacion: direccion=0x%02X registro=0x%02X CHIP_ID=0x%02X",
                 QMC5883P_ADDRESS, QMC5883P_REG_CHIP_ID, chip_id);
        if (chip_id == QMC5883P_CHIP_ID)
            ESP_LOGI(TAG, "Magnetometro compatible con QMC5883P: direccion e ID correctos");
        else
            ESP_LOGW(TAG, "Modelo sin confirmar: se esperaba CHIP_ID=0x%02X para QMC5883P",
                     QMC5883P_CHIP_ID);
    }

    const esp_err_t remove_error = i2c_master_bus_rm_device(magnetometer);
    return error != ESP_OK ? error : remove_error;
}

static esp_err_t scan_i2c(const char *label, bool detected[128])
{
    unsigned found = 0;
    ESP_LOGI(TAG, "========== %s ==========", label);

    /* Direcciones de 7 bits, excluyendo las reservadas. */
    for (uint16_t address = 0x08; address <= 0x77; ++address)
    {
        const esp_err_t error = i2c_master_probe(s_i2c_bus, address, 50);
        detected[address] = error == ESP_OK;
        if (error == ESP_OK)
        {
            ESP_LOGI(TAG, "Dispositivo encontrado: 0x%02X", (unsigned)address);
            ++found;
        }
        else if (error != ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Escaneo interrumpido en 0x%02X: %s",
                     (unsigned)address, esp_err_to_name(error));
            return error;
        }
        vTaskDelay(1);
    }

    ESP_LOGI(TAG, "%s: %u dispositivos encontrados", label, found);
    return ESP_OK;
}

static esp_err_t init_i2c(void)
{
    const i2c_master_bus_config_t config = {
        .i2c_port = I2C_1,
        .sda_io_num = PIN_SDA1_I2C,
        .scl_io_num = PIN_SCLK1_I2C,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&config, &s_i2c_bus);
}
