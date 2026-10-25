#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "lorae220_900.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

const static char *TAG = "lora";

// Funciones para construir los registros
static uint8_t lora_build_reg0(lora_t *lora);
static uint8_t lora_build_reg1(lora_t *lora);
static uint8_t lora_build_reg3(lora_t *lora);

static esp_err_t lora_hw_init(lora_t *lora)
{
    if (lora == NULL)
        return ESP_ERR_INVALID_ARG;
    uart_config_t uart_config = {
        .baud_rate = lora->config.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(lora->config.uart_port_num, 1024, 1024, 0, NULL, 0);

    if (ret != ESP_OK)
        return ret;

    ret = uart_param_config(lora->config.uart_port_num, &uart_config);

    if (ret != ESP_OK)
        return ret;

    ret = uart_set_pin(
        lora->config.uart_port_num,
        lora->config.tx_pin, lora->config.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    if (ret != ESP_OK)
        return ret;
    if ((lora->config.m0_pin != GPIO_NUM_NC) && (lora->config.m1_pin != GPIO_NUM_NC))
    {
        gpio_config_t o_Config = {
            .pin_bit_mask = (1ULL << lora->config.m0_pin) | (1ULL << lora->config.m1_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE};
        ret = gpio_config(&o_Config);
        if (ret != ESP_OK)
            return ret;
        ret = gpio_set_level(lora->config.m0_pin, 0);
        if (ret != ESP_OK)
            return ret;
        ret = gpio_set_level(lora->config.m1_pin, 0);
        if (ret != ESP_OK)
            return ret;
    }
    if (lora->config.aux_pin != GPIO_NUM_NC)
    {
        gpio_config_t i_Config = {
            .pin_bit_mask = (1ULL << lora->config.aux_pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE

        };

        ret = gpio_config(&i_Config);
        if (ret != ESP_OK)
            return ret;
    }

    uart_flush_input(lora->config.uart_port_num);
    return ESP_OK;
}

static esp_err_t lora_e220_uart_write(lora_t *lora, const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0 || lora == NULL)
        return ESP_ERR_INVALID_ARG;

    int written = uart_write_bytes(lora->config.uart_port_num,
                                   (const char *)data,
                                   length);

    if (written != length)
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t lora_e220_uart_read(lora_t *lora,
                                     uint8_t *data,
                                     uint16_t length,
                                     uint32_t timeout_ms)
{
    if (data == NULL || length == 0 || lora == NULL)
        return ESP_ERR_INVALID_ARG;

    int read = uart_read_bytes(
        lora->config.uart_port_num,
        data,
        length,
        pdMS_TO_TICKS(timeout_ms));

    if (read != length)
        return ESP_ERR_TIMEOUT;

    return ESP_OK;
}

static esp_err_t lora_e220_write_registers(lora_t *lora)
{
    if (lora == NULL)
        return ESP_ERR_INVALID_ARG;

    uint8_t buffer[11];
    uint8_t response[11];

    buffer[0] = COMMAND_SET;
    buffer[1] = LORA_ADDH;
    buffer[2] = 0x08;
    buffer[3] = (uint8_t)(lora->config.address >> 8);
    buffer[4] = (uint8_t)(lora->config.address & 0xFF);
    buffer[5] = lora_build_reg0(lora);
    buffer[6] = lora_build_reg1(lora);
    buffer[7] = lora->config.channel;
    buffer[8] = lora_build_reg3(lora);
    buffer[9] = (uint8_t)(lora->config.crypt_key >> 8);
    buffer[10] = (uint8_t)(lora->config.crypt_key & 0xFF);

    ESP_LOGI(TAG, "TX:");
    for (int i = 0; i < sizeof(buffer); i++)
        ESP_LOGI(TAG, "[%02d] = 0x%02X", i, buffer[i]);

    esp_err_t ret = lora_e220_uart_write(lora, buffer, sizeof(buffer));
    if (ret != ESP_OK)
        return ret;

    ret = lora_e220_uart_read(lora, response, sizeof(response), 1000);
    if (ret != ESP_OK)
        return ret;

    ESP_LOGI(TAG, "RX:");
    for (int i = 0; i < sizeof(response); i++)
        ESP_LOGI(TAG, "[%02d] = 0x%02X", i, response[i]);

    if (response[0] != COMMAND_RESPONSE)
        return ESP_FAIL;

    for (int i = 1; i < sizeof(buffer); i++)
    {
        if (response[i] != buffer[i])
            return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t lora_e220_set_mode(lora_t *lora, lora_e220_mode_t mode)
{
    if (lora == NULL)
        return ESP_ERR_INVALID_ARG;

    if ((lora->config.m0_pin == GPIO_NUM_NC) ||
        (lora->config.m1_pin == GPIO_NUM_NC))
        return ESP_ERR_INVALID_STATE;

    int m0_level;
    int m1_level;

    switch (mode)
    {
    case LORA_MODE_NORMAL:
        m1_level = 0;
        m0_level = 0;
        break;

    case LORA_MODE_WOR_TX:
        m1_level = 0;
        m0_level = 1;
        break;

    case LORA_MODE_WOR_RX:
        m1_level = 1;
        m0_level = 0;
        break;

    case LORA_MODE_CONFIG:
        m1_level = 1;
        m0_level = 1;
        break;

    default:
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = gpio_set_level(lora->config.m1_pin, m1_level);
    if (ret != ESP_OK)
        return ret;

    ret = gpio_set_level(lora->config.m0_pin, m0_level);
    if (ret != ESP_OK)
        return ret;

    lora->config.mode = mode;

    vTaskDelay(pdMS_TO_TICKS(2));

    ESP_LOGI(TAG,
             "Mode=%d M0(GPIO%d)=%d M1(GPIO%d)=%d",
             (int)mode,
             (int)lora->config.m0_pin,
             gpio_get_level(lora->config.m0_pin),
             (int)lora->config.m1_pin,
             gpio_get_level(lora->config.m1_pin));

    return ESP_OK;
}

static esp_err_t lora_e220_wait_aux(lora_t *lora, uint32_t timeout_ms)
{
    if (lora == NULL)
        return ESP_ERR_INVALID_ARG;

    if (lora->config.aux_pin == GPIO_NUM_NC)
    {
        vTaskDelay(pdMS_TO_TICKS(50));
        return ESP_OK;
    }

    TickType_t start_time = xTaskGetTickCount();

    while (gpio_get_level(lora->config.aux_pin) == 0)
    {
        if ((xTaskGetTickCount() - start_time) > pdMS_TO_TICKS(timeout_ms))
            return ESP_ERR_TIMEOUT;

        /* Con CONFIG_FREERTOS_HZ=100, pdMS_TO_TICKS(1) es cero y no bloquea.
           Un tick real permite ejecutar IDLE y alimentar el watchdog. */
        vTaskDelay(1);
    }

    return ESP_OK;
}

static esp_err_t lora_e220_wait_transmission_complete(lora_t *lora,
                                                       uint32_t timeout_ms)
{
    if (lora == NULL)
        return ESP_ERR_INVALID_ARG;

    esp_err_t ret = uart_wait_tx_done(lora->config.uart_port_num,
                                      pdMS_TO_TICKS(timeout_ms));
    if (ret != ESP_OK)
        return ret;

    if (lora->config.aux_pin != GPIO_NUM_NC)
    {
        /*
         * uart_write_bytes() termina antes de que todos los bytes salgan por
         * TXD. Incluso despues de uart_wait_tx_done(), el E220 necesita unos
         * milisegundos para reflejar el estado ocupado en AUX. Con un tick de
         * 10 ms se evita leer el nivel alto anterior a la transmision.
         */
        vTaskDelay(1);
    }

    return lora_e220_wait_aux(lora, timeout_ms);
}

esp_err_t lora_e220_900_init(lora_t *lora)
{
    if (lora == NULL)
        return ESP_ERR_INVALID_ARG;

    const bool m0_available = lora->config.m0_pin != GPIO_NUM_NC;
    const bool m1_available = lora->config.m1_pin != GPIO_NUM_NC;

    /* M0 y M1 deben estar ambos conectados o ambos controlados externamente. */
    if (m0_available != m1_available)
    {
        ESP_LOGE(TAG, "M0 and M1 must both be configured or both be GPIO_NUM_NC");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = lora_hw_init(lora);
    if (ret != ESP_OK)
        return ret;

    ret = lora_e220_wait_aux(lora, 1000);
    if (ret != ESP_OK)
        return ret;

    /*
     * Sin GPIO para M0/M1, el modulo debe estar preconfigurado y tener ambos
     * pines conectados fisicamente a GND (modo normal).
     */
    if (!m0_available)
    {
        lora->config.mode = LORA_MODE_NORMAL;
        ESP_LOGI(TAG, "E220 preconfigured; M0/M1 fixed in normal mode");
        return ESP_OK;
    }

    ret = lora_e220_set_mode(lora, LORA_MODE_CONFIG);
    if (ret != ESP_OK)
        return ret;

    ret = lora_e220_wait_aux(lora, 1000);
    if (ret != ESP_OK)
        return ret;

    ret = lora_e220_write_registers(lora);
    if (ret != ESP_OK)
        return ret;

    ret = lora_e220_wait_aux(lora, 1000);
    if (ret != ESP_OK)
        return ret;

    ret = lora_e220_set_mode(lora, LORA_MODE_NORMAL);
    if (ret != ESP_OK)
        return ret;

    return lora_e220_wait_aux(lora, 1000);
}

// Enviar datos en modo normal
esp_err_t lora_e220_send(lora_t *lora, const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0 || lora == NULL)
        return ESP_ERR_INVALID_ARG;

    esp_err_t ret = lora_e220_wait_aux(lora, 1000);
    if (ret != ESP_OK)
        return ret;

    ret = lora_e220_uart_write(lora, data, length);
    if (ret != ESP_OK)
        return ret;

    return lora_e220_wait_transmission_complete(lora, 1000);
}

esp_err_t lora_e220_send_fixed(
    lora_t *lora,
    uint16_t address,
    uint8_t channel,
    const uint8_t *data,
    uint16_t length)
{
    if (data == NULL || length == 0 || lora == NULL)
        return ESP_ERR_INVALID_ARG;

    uint8_t tx_buffer[3 + length];

    // Destino
    tx_buffer[0] = (uint8_t)(address >> 8);
    tx_buffer[1] = (uint8_t)(address & 0xFF);
    tx_buffer[2] = channel;

    memcpy(&tx_buffer[3], data, length);

    esp_err_t ret = lora_e220_wait_aux(lora, 1000);
    if (ret != ESP_OK)
        return ret;

    ret = lora_e220_uart_write(lora, tx_buffer, sizeof(tx_buffer));
    if (ret != ESP_OK)
        return ret;

    return lora_e220_wait_transmission_complete(lora, 1000);
}

int lora_e220_receive(lora_t *lora, uint8_t *buffer, uint16_t max_len, uint32_t timeout_ms)
{
    if (buffer == NULL || max_len == 0 || lora == NULL)
        return -1;

    return uart_read_bytes(
        lora->config.uart_port_num,
        buffer,
        max_len,
        pdMS_TO_TICKS(timeout_ms));
}

static uint8_t lora_build_reg0(lora_t *lora)
{
    return ((lora->config.uart_baud & 0x07) << 5) |
           ((lora->config.uart_parity & 0x03) << 3) |
           ((lora->config.air_rate & 0x07) << 0);
}

static uint8_t lora_build_reg1(lora_t *lora)
{
    return ((lora->config.sub_packet & 0x03) << 6) |
           ((lora->config.rssi_noise & 0x01) << 5) |
           ((lora->config.tx_power & 0x03) << 0);
}

static uint8_t lora_build_reg3(lora_t *lora)
{
    return ((lora->config.rssi_byte & 0x01) << 7) |
           ((lora->config.transmission_method & 0x01) << 6) |
           ((lora->config.lbt & 0x01) << 4) |
           ((lora->config.wor_cycle & 0x07) << 0);
}
