#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mpu6050.h"
#include "qmc5883p.h"
#include "shared.h"

#define QMC_TEST_PERIOD_MS UINT32_C(100)
#define QMC_TEST_DATA_TIMEOUT_MS UINT32_C(100)

static const char *TAG = "test_qmc";
static i2c_master_bus_handle_t s_i2c_bus;
static mpu6050_t s_mpu;
static qmc5883p_t s_qmc;

/* Prototipos de tareas. */
static void vTaskReadQMC(void *argument);

/* Prototipos de funciones auxiliares. */
static esp_err_t init_i2c(void);
static esp_err_t init_sensors(void);
static void diagnose_qmc_configuration(void);
static void cleanup_sensors(void);

void app_main(void)
{
    ESP_ERROR_CHECK(init_i2c());
    const esp_err_t error = init_sensors();
    if (error != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudo iniciar el QMC5883P: %s", esp_err_to_name(error));
        if (error == ESP_ERR_INVALID_RESPONSE && s_qmc.i2c_dev == NULL)
            diagnose_qmc_configuration();
        cleanup_sensors();
        return;
    }

    ESP_LOGI(TAG, "GY-87: SDA=GPIO%d SCL=GPIO%d, bypass MPU=ON, QMC=0x%02X",
             (int)PIN_SDA1_I2C, (int)PIN_SCLK1_I2C, s_qmc.config.device_address);
    if (xTaskCreate(vTaskReadQMC, "qmc_reader", 4096, NULL, 2, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "No se pudo crear la tarea de lectura");
        cleanup_sensors();
    }
}

/* Tareas. */
static void vTaskReadQMC(void *argument)
{
    (void)argument;
    uint32_t sequence = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (true)
    {
        const esp_err_t error = qmc5883p_read_data(&s_qmc, QMC_TEST_DATA_TIMEOUT_MS);
        if (error == ESP_OK)
        {
            ESP_LOGI(TAG,
                     "QMC seq=%lu raw=[%d,%d,%d] B=[%.3f,%.3f,%.3f] uT norma=%.3f uT",
                     (unsigned long)sequence++,
                     (int)s_qmc.data.raw_x, (int)s_qmc.data.raw_y, (int)s_qmc.data.raw_z,
                     (double)s_qmc.data.x_ut, (double)s_qmc.data.y_ut,
                     (double)s_qmc.data.z_ut, (double)s_qmc.data.norm_ut);
        }
        else if (error == QMC5883P_ERR_OVERFLOW && s_qmc.data.overflow)
        {
            ESP_LOGW(TAG, "Saturacion magnetica: muestra descartada, STATUS=0x%02X",
                     s_qmc.data.status);
        }
        else
        {
            ESP_LOGW(TAG, "Error de lectura: %s", esp_err_to_name(error));
        }

        if (xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(QMC_TEST_PERIOD_MS)) != pdTRUE)
        {
            last_wake = xTaskGetTickCount();
            vTaskDelay(1);
        }
    }
}

/* Definiciones de funciones auxiliares. */
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

static esp_err_t init_sensors(void)
{
    s_mpu.bus_handle = s_i2c_bus;
    s_mpu.config = (mpu6050_config_t)MPU6050_DEFAULT_CONFIG();
    esp_err_t error = mpu6050_init(&s_mpu);
    if (error != ESP_OK) return error;
    error = mpu6050_set_i2c_bypass(&s_mpu, true);
    if (error != ESP_OK) return error;
    vTaskDelay(pdMS_TO_TICKS(10));

    s_qmc.bus_handle = s_i2c_bus;
    s_qmc.config = (qmc5883p_config_t)QMC5883P_DEFAULT_CONFIG();
    return qmc5883p_init(&s_qmc);
}

static void diagnose_qmc_configuration(void)
{
    /* Prueba de registros solamente: no asigna sensibilidad ni publica uT. */
    const i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = QMC5883P_I2C_ADDRESS,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t handle = NULL;
    esp_err_t error = i2c_master_bus_add_device(s_i2c_bus, &config, &handle);
    if (error != ESP_OK) return;

    uint8_t id = 0, saved[2] = {0};
    uint8_t reg = QMC5883P_REG_CHIP_ID;
    error = i2c_master_transmit_receive(handle, &reg, 1, &id, 1, 50);
    if (error != ESP_OK || id != QMC5883P_CHIP_ID) goto finish;
    reg = QMC5883P_REG_CONTROL_1;
    error = i2c_master_transmit_receive(handle, &reg, 1, &saved[0], 1, 50);
    if (error != ESP_OK) goto finish;
    reg = QMC5883P_REG_CONTROL_2;
    error = i2c_master_transmit_receive(handle, &reg, 1, &saved[1], 1, 50);
    if (error != ESP_OK) goto finish;
    ESP_LOGI(TAG, "DIAG inicial: ID=0x%02X CTRL1=0x%02X CTRL2=0x%02X",
             id, saved[0], saved[1]);
    /* Comparar el transporte sin alterar los controles. Una lectura con STOP
     * entre direccion y datos permite distinguir un problema de acceso de uno
     * de configuracion. El driver conserva el repeated START de QST.
     */
    for (unsigned i = 0; i < 2; ++i)
    {
        uint8_t with_stop = 0;
        reg = i == 0 ? QMC5883P_REG_CONTROL_1 : QMC5883P_REG_CONTROL_2;
        error = i2c_master_transmit(handle, &reg, 1, 50);
        if (error != ESP_OK) goto finish;
        error = i2c_master_receive(handle, &with_stop, 1, 50);
        if (error != ESP_OK) goto finish;
        ESP_LOGI(TAG, "DIAG acceso 0x%02X: repeated_START=0x%02X STOP=0x%02X",
                 reg, saved[i], with_stop);
    }
    if ((saved[1] & UINT8_C(0xC0)) != 0)
    {
        ESP_LOGW(TAG, "DIAG detenido: reset o self-test siguen activos");
        goto finish;
    }

    uint8_t write_bytes[2] = {QMC5883P_REG_CONTROL_1,
                              saved[0] & (uint8_t)~QMC5883P_CONTROL_1_MODE_MASK};
    error = i2c_master_transmit(handle, write_bytes, sizeof(write_bytes), 50);
    if (error != ESP_OK) goto restore;
    vTaskDelay(1);
    for (unsigned range = 0; range < 4; ++range)
    {
        const uint8_t requested = (saved[1] & UINT8_C(0xF3)) | (uint8_t)(range << 2);
        write_bytes[0] = QMC5883P_REG_CONTROL_2;
        write_bytes[1] = requested;
        error = i2c_master_transmit(handle, write_bytes, sizeof(write_bytes), 50);
        if (error != ESP_OK) break;
        vTaskDelay(1);
        uint8_t actual = 0;
        uint8_t with_stop = 0;
        reg = QMC5883P_REG_CONTROL_2;
        error = i2c_master_transmit_receive(handle, &reg, 1, &actual, 1, 50);
        if (error != ESP_OK) break;
        error = i2c_master_transmit(handle, &reg, 1, 50);
        if (error != ESP_OK) break;
        error = i2c_master_receive(handle, &with_stop, 1, 50);
        if (error != ESP_OK) break;
        ESP_LOGI(TAG, "DIAG rango: CTRL2 escrito=0x%02X leido=0x%02X STOP=0x%02X %s",
                 requested, actual, with_stop, actual == requested ? "OK" : "DIFIERE");
    }

restore:
    write_bytes[0] = QMC5883P_REG_CONTROL_2;
    write_bytes[1] = saved[1];
    if (i2c_master_transmit(handle, write_bytes, sizeof(write_bytes), 50) != ESP_OK)
        ESP_LOGW(TAG, "DIAG: no se pudo restaurar CTRL2");
    write_bytes[0] = QMC5883P_REG_CONTROL_1;
    write_bytes[1] = saved[0];
    if (i2c_master_transmit(handle, write_bytes, sizeof(write_bytes), 50) != ESP_OK)
        ESP_LOGW(TAG, "DIAG: no se pudo restaurar CTRL1");
finish:
    if (error != ESP_OK)
        ESP_LOGW(TAG, "DIAG: error I2C %s", esp_err_to_name(error));
    if (i2c_master_bus_rm_device(handle) != ESP_OK)
        ESP_LOGW(TAG, "DIAG: no se pudo retirar el dispositivo");
}

static void cleanup_sensors(void)
{
    if (s_qmc.i2c_dev != NULL)
    {
        const esp_err_t error = qmc5883p_deinit(&s_qmc);
        if (error != ESP_OK)
        {
            ESP_LOGE(TAG, "No se pudo retirar QMC del bus: %s", esp_err_to_name(error));
            return; /* Conservar el bypass y bus si aun existe el dispositivo. */
        }
    }
    if (s_mpu.initialized)
    {
        esp_err_t error = mpu6050_set_i2c_bypass(&s_mpu, false);
        if (error != ESP_OK)
            ESP_LOGW(TAG, "No se pudo desactivar bypass: %s", esp_err_to_name(error));
        error = mpu6050_deinit(&s_mpu);
        if (error != ESP_OK)
        {
            ESP_LOGE(TAG, "No se pudo retirar MPU del bus: %s", esp_err_to_name(error));
            return;
        }
    }
    if (s_i2c_bus != NULL)
    {
        const esp_err_t error = i2c_del_master_bus(s_i2c_bus);
        if (error == ESP_OK) s_i2c_bus = NULL;
        else ESP_LOGE(TAG, "No se pudo cerrar I2C: %s", esp_err_to_name(error));
    }
}
