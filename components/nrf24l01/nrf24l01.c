#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "nrf24l01.h"

#define NRF24_SPI_BUFFER_SIZE       (NRF24_MAX_PAYLOAD_SIZE + 1U)
#define NRF24_REGISTER_MAX_SIZE     NRF24_MAX_ADDRESS_WIDTH
#define NRF24_RX_PIPE_EMPTY         0x07U
#define NRF24_POLL_INTERVAL_US      100U

static const char *TAG = "nrf24l01";

static bool nrf24l01_is_ready(const nrf24l01_t *device)
{
    return (device != NULL) && device->initialized &&
           (device->spi_handle != NULL);
}

static bool nrf24l01_config_is_valid(const nrf24l01_config_t *config)
{
    if ((config == NULL) || !GPIO_IS_VALID_OUTPUT_GPIO(config->ce_pin) ||
        !GPIO_IS_VALID_OUTPUT_GPIO(config->csn_pin)) {
        return false;
    }

    if (config->initialize_spi_bus &&
        (!GPIO_IS_VALID_OUTPUT_GPIO(config->mosi_pin) ||
         !GPIO_IS_VALID_GPIO(config->miso_pin) ||
         !GPIO_IS_VALID_OUTPUT_GPIO(config->sclk_pin))) {
        return false;
    }

    if ((config->irq_pin != GPIO_NUM_NC) &&
        !GPIO_IS_VALID_GPIO(config->irq_pin)) {
        return false;
    }

    if ((config->spi_clock_hz == 0U) ||
        (config->spi_clock_hz > NRF24_MAX_SPI_CLOCK_HZ) ||
        (config->channel > NRF24_MAX_CHANNEL) ||
        (config->payload_size == 0U) ||
        (config->payload_size > NRF24_MAX_PAYLOAD_SIZE) ||
        ((config->enabled_rx_pipes & ~NRF24_ALL_PIPES_MASK) != 0U) ||
        ((config->auto_ack_pipes & ~NRF24_ALL_PIPES_MASK) != 0U) ||
        (config->address_width < NRF24_ADDRESS_WIDTH_3_BYTES) ||
        (config->address_width > NRF24_ADDRESS_WIDTH_5_BYTES) ||
        (config->auto_retransmit_count > NRF24_RETRANSMIT_COUNT_15) ||
        (config->auto_retransmit_delay > NRF24_RETRANSMIT_DELAY_4000_US) ||
        (config->output_power > NRF24_OUTPUT_POWER_0_DBM) ||
        (config->air_data_rate > NRF24_DATA_RATE_250_KBPS) ||
        (config->crc_length > NRF24_CRC_2_BYTES)) {
        return false;
    }

    if ((config->auto_ack_pipes != 0U) &&
        (config->crc_length == NRF24_CRC_DISABLED)) {
        return false;
    }
    if (config->enable_ack_payloads && (config->auto_ack_pipes == 0U)) {
        return false;
    }

    /* At 250 kbps the ACK window requires an ARD of at least 500 us. */
    if ((config->air_data_rate == NRF24_DATA_RATE_250_KBPS) &&
        (config->auto_ack_pipes != 0U) &&
        (config->auto_retransmit_delay == NRF24_RETRANSMIT_DELAY_250_US)) {
        return false;
    }

    return true;
}

static esp_err_t nrf24l01_transfer(nrf24l01_t *device,
                                   const uint8_t *tx_data,
                                   uint8_t *rx_data,
                                   size_t data_size)
{
    if (!nrf24l01_is_ready(device) || (tx_data == NULL) ||
        (data_size == 0U) || (data_size > NRF24_SPI_BUFFER_SIZE)) {
        return ESP_ERR_INVALID_ARG;
    }

    spi_transaction_t transaction = {
        .length = data_size * 8U,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    esp_err_t error = spi_device_transmit(device->spi_handle, &transaction);
    if ((error == ESP_OK) && (rx_data != NULL)) {
        device->status = rx_data[0];
    }
    return error;
}

static esp_err_t nrf24l01_read_u8(nrf24l01_t *device, uint8_t reg,
                                  uint8_t *value)
{
    return nrf24l01_read_register(device, reg, value, sizeof(*value));
}

static esp_err_t nrf24l01_write_u8(nrf24l01_t *device, uint8_t reg,
                                   uint8_t value)
{
    return nrf24l01_write_register(device, reg, &value, sizeof(value));
}

static esp_err_t nrf24l01_write_payload(nrf24l01_t *device, uint8_t command,
                                        const void *data, size_t data_size)
{
    if ((data == NULL) || (data_size == 0U) ||
        (data_size > NRF24_MAX_PAYLOAD_SIZE)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t tx[NRF24_SPI_BUFFER_SIZE];
    uint8_t rx[NRF24_SPI_BUFFER_SIZE];
    tx[0] = command;
    memcpy(&tx[1], data, data_size);
    return nrf24l01_transfer(device, tx, rx, data_size + 1U);
}

static esp_err_t nrf24l01_read_payload_width(nrf24l01_t *device,
                                             uint8_t *payload_width)
{
    uint8_t tx[2] = {NRF24_CMD_R_RX_PL_WID, NRF24_CMD_NOP};
    uint8_t rx[2] = {0};
    esp_err_t error = nrf24l01_transfer(device, tx, rx, sizeof(tx));
    if (error == ESP_OK) {
        *payload_width = rx[1];
    }
    return error;
}

static esp_err_t nrf24l01_activate_features(nrf24l01_t *device)
{
    uint8_t tx[2] = {NRF24_CMD_ACTIVATE, NRF24_CMD_ACTIVATE_DATA};
    uint8_t rx[2] = {0};
    return nrf24l01_transfer(device, tx, rx, sizeof(tx));
}

static esp_err_t nrf24l01_configure_features(nrf24l01_t *device,
                                             uint8_t feature,
                                             uint8_t dynpd)
{
    esp_err_t error = nrf24l01_write_u8(device, NRF24_FEATURE, feature);
    if (error != ESP_OK) return error;

    uint8_t readback = 0U;
    if ((error = nrf24l01_read_u8(device, NRF24_FEATURE, &readback)) != ESP_OK) {
        return error;
    }

    /* Some compatible parts require ACTIVATE 0x73 before FEATURE is writable. */
    if (readback != feature) {
        if ((error = nrf24l01_activate_features(device)) != ESP_OK) return error;
        if ((error = nrf24l01_write_u8(device, NRF24_FEATURE, feature)) != ESP_OK) {
            return error;
        }
        if ((error = nrf24l01_read_u8(device, NRF24_FEATURE, &readback)) != ESP_OK) {
            return error;
        }
        if (readback != feature) return ESP_ERR_NOT_SUPPORTED;
    }

    return nrf24l01_write_u8(device, NRF24_DYNPD, dynpd);
}

static esp_err_t nrf24l01_read_payload(nrf24l01_t *device, void *data,
                                       size_t data_size, uint8_t *status)
{
    uint8_t tx[NRF24_SPI_BUFFER_SIZE];
    uint8_t rx[NRF24_SPI_BUFFER_SIZE];
    memset(tx, NRF24_CMD_NOP, data_size + 1U);
    tx[0] = NRF24_CMD_R_RX_PAYLOAD;

    esp_err_t error = nrf24l01_transfer(device, tx, rx, data_size + 1U);
    if (error == ESP_OK) {
        *status = rx[0];
        memcpy(data, &rx[1], data_size);
    }
    return error;
}

static esp_err_t nrf24l01_set_config_bits(nrf24l01_t *device,
                                          uint8_t set_mask,
                                          uint8_t clear_mask)
{
    uint8_t config = 0U;
    esp_err_t error = nrf24l01_read_u8(device, NRF24_CONFIG, &config);
    if (error != ESP_OK) {
        return error;
    }
    config = (config | set_mask) & (uint8_t)~clear_mask;
    return nrf24l01_write_u8(device, NRF24_CONFIG, config);
}

esp_err_t nrf24l01_read_register(nrf24l01_t *device, uint8_t reg,
                                 void *data, size_t data_size)
{
    if ((data == NULL) || (data_size == 0U) ||
        (data_size > NRF24_REGISTER_MAX_SIZE) ||
        ((reg & (uint8_t)~NRF24_CMD_REGISTER_MASK) != 0U)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t tx[NRF24_REGISTER_MAX_SIZE + 1U];
    uint8_t rx[NRF24_REGISTER_MAX_SIZE + 1U];
    memset(tx, NRF24_CMD_NOP, data_size + 1U);
    tx[0] = NRF24_CMD_R_REGISTER | reg;

    esp_err_t error = nrf24l01_transfer(device, tx, rx, data_size + 1U);
    if (error == ESP_OK) {
        memcpy(data, &rx[1], data_size);
    }
    return error;
}

esp_err_t nrf24l01_write_register(nrf24l01_t *device, uint8_t reg,
                                  const void *data, size_t data_size)
{
    if ((data == NULL) || (data_size == 0U) ||
        (data_size > NRF24_REGISTER_MAX_SIZE) ||
        ((reg & (uint8_t)~NRF24_CMD_REGISTER_MASK) != 0U)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t tx[NRF24_REGISTER_MAX_SIZE + 1U];
    uint8_t rx[NRF24_REGISTER_MAX_SIZE + 1U];
    tx[0] = NRF24_CMD_W_REGISTER | reg;
    memcpy(&tx[1], data, data_size);
    return nrf24l01_transfer(device, tx, rx, data_size + 1U);
}

esp_err_t nrf24l01_command(nrf24l01_t *device, uint8_t command)
{
    uint8_t rx = 0U;
    return nrf24l01_transfer(device, &command, &rx, sizeof(command));
}

static esp_err_t nrf24l01_apply_config(nrf24l01_t *device)
{
    const nrf24l01_config_t *config = &device->config;
    uint8_t value = 0U;

    if (config->crc_length != NRF24_CRC_DISABLED) {
        value |= NRF24_CONFIG_EN_CRC;
        if (config->crc_length == NRF24_CRC_2_BYTES) {
            value |= NRF24_CONFIG_CRCO;
        }
    }

    esp_err_t error = nrf24l01_write_u8(device, NRF24_CONFIG, value);
    if (error != ESP_OK) return error;
    if ((error = nrf24l01_write_u8(device, NRF24_EN_AA,
                                   config->auto_ack_pipes)) != ESP_OK) return error;
    if ((error = nrf24l01_write_u8(device, NRF24_EN_RXADDR,
                                   config->enabled_rx_pipes)) != ESP_OK) return error;
    if ((error = nrf24l01_write_u8(device, NRF24_SETUP_AW,
                                   (uint8_t)config->address_width)) != ESP_OK) return error;

    value = ((uint8_t)config->auto_retransmit_delay << NRF24_SETUP_RETR_ARD_POS) |
            (uint8_t)config->auto_retransmit_count;
    if ((error = nrf24l01_write_u8(device, NRF24_SETUP_RETR, value)) != ESP_OK) return error;
    if ((error = nrf24l01_write_u8(device, NRF24_RF_CH,
                                   config->channel)) != ESP_OK) return error;

    value = (uint8_t)config->output_power << NRF24_RF_SETUP_RF_PWR_POS;
    if (config->air_data_rate == NRF24_DATA_RATE_2_MBPS) {
        value |= NRF24_RF_SETUP_RF_DR_HIGH;
    } else if (config->air_data_rate == NRF24_DATA_RATE_250_KBPS) {
        value |= NRF24_RF_SETUP_RF_DR_LOW;
    }
    if ((error = nrf24l01_write_u8(device, NRF24_RF_SETUP, value)) != ESP_OK) return error;

    for (uint8_t pipe = 0U; pipe < NRF24_PIPE_COUNT; ++pipe) {
        value = ((config->enabled_rx_pipes & NRF24_PIPE_BIT(pipe)) != 0U)
                    ? config->payload_size : 0U;
        if ((error = nrf24l01_write_u8(device,
                                       (uint8_t)(NRF24_RX_PW_P0 + pipe),
                                       value)) != ESP_OK) return error;
    }

    bool dynamic_enabled = config->enable_dynamic_payloads ||
                           config->enable_ack_payloads;
    uint8_t feature = dynamic_enabled ? NRF24_FEATURE_EN_DPL : 0U;
    if (config->enable_ack_payloads) feature |= NRF24_FEATURE_EN_ACK_PAY;
    uint8_t dynpd = dynamic_enabled ? config->enabled_rx_pipes : 0U;
    if ((error = nrf24l01_configure_features(device, feature, dynpd)) != ESP_OK) {
        return error;
    }
    if ((error = nrf24l01_write_u8(device, NRF24_STATUS,
                                   NRF24_STATUS_IRQ_MASK)) != ESP_OK) return error;
    if ((error = nrf24l01_command(device, NRF24_CMD_FLUSH_RX)) != ESP_OK) return error;
    return nrf24l01_command(device, NRF24_CMD_FLUSH_TX);
}

esp_err_t nrf24l01_is_connected(nrf24l01_t *device, bool *connected)
{
    if (!nrf24l01_is_ready(device) || (connected == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t address_width = 0U;
    uint8_t channel = 0U;
    esp_err_t error = nrf24l01_read_u8(device, NRF24_SETUP_AW, &address_width);
    if (error != ESP_OK) return error;
    if ((error = nrf24l01_read_u8(device, NRF24_RF_CH, &channel)) != ESP_OK) return error;

    *connected = (address_width >= NRF24_ADDRESS_WIDTH_3_BYTES) &&
                 (address_width <= NRF24_ADDRESS_WIDTH_5_BYTES) &&
                 ((channel & (uint8_t)~NRF24_RF_CH_MASK) == 0U) &&
                 (channel <= NRF24_MAX_CHANNEL);
    return ESP_OK;
}

esp_err_t nrf24l01_init(nrf24l01_t *device,
                        const nrf24l01_config_t *config)
{
    if ((device == NULL) || !nrf24l01_config_is_valid(config)) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(device, 0, sizeof(*device));
    device->config = *config;

    esp_err_t error = ESP_OK;
    if (config->initialize_spi_bus) {
        const spi_bus_config_t bus_config = {
            .mosi_io_num = config->mosi_pin,
            .miso_io_num = config->miso_pin,
            .sclk_io_num = config->sclk_pin,
            .quadwp_io_num = GPIO_NUM_NC,
            .quadhd_io_num = GPIO_NUM_NC,
            .max_transfer_sz = NRF24_SPI_BUFFER_SIZE,
        };
        error = spi_bus_initialize(config->spi_host, &bus_config,
                                   config->spi_dma_chan);
        if (error != ESP_OK) return error;
    }

    const spi_device_interface_config_t spi_config = {
        .clock_speed_hz = (int)config->spi_clock_hz,
        .mode = 0,
        .spics_io_num = config->csn_pin,
        .queue_size = 1,
    };
    error = spi_bus_add_device(config->spi_host, &spi_config,
                               &device->spi_handle);
    if (error != ESP_OK) goto fail;

    gpio_config_t ce_config = {
        .pin_bit_mask = 1ULL << config->ce_pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if ((error = gpio_config(&ce_config)) != ESP_OK) goto fail;
    if ((error = gpio_set_level(config->ce_pin, 0)) != ESP_OK) goto fail;

    if (config->irq_pin != GPIO_NUM_NC) {
        gpio_config_t irq_config = {
            .pin_bit_mask = 1ULL << config->irq_pin,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        if ((error = gpio_config(&irq_config)) != ESP_OK) goto fail;
    }

    device->initialized = true;
    device->mode = NRF24_MODE_POWER_DOWN;
    vTaskDelay(pdMS_TO_TICKS(NRF24_POWER_ON_RESET_DELAY_MS));

    bool connected = false;
    error = nrf24l01_is_connected(device, &connected);
    if ((error != ESP_OK) || !connected) {
        if (error == ESP_OK) error = ESP_ERR_NOT_FOUND;
        ESP_LOGE(TAG, "nRF24L01+ no detectado por SPI");
        goto fail;
    }

    if ((error = nrf24l01_apply_config(device)) != ESP_OK) goto fail;
    size_t address_size = (size_t)config->address_width + 2U;
    if ((error = nrf24l01_read_register(device, NRF24_TX_ADDR,
                                        device->tx_address,
                                        address_size)) != ESP_OK) goto fail;
    if ((error = nrf24l01_read_register(device, NRF24_RX_ADDR_P0,
                                        device->rx_pipe0_address,
                                        address_size)) != ESP_OK) goto fail;

    ESP_LOGI(TAG, "nRF24L01+ inicializado: canal=%u, payload=%u bytes",
             config->channel, config->payload_size);
    return ESP_OK;

fail:
    (void)nrf24l01_deinit(device);
    return error;
}

esp_err_t nrf24l01_deinit(nrf24l01_t *device)
{
    if (device == NULL) return ESP_ERR_INVALID_ARG;

    esp_err_t first_error = ESP_OK;
    if (device->spi_handle != NULL) {
        esp_err_t error = spi_bus_remove_device(device->spi_handle);
        if (error != ESP_OK) first_error = error;
        device->spi_handle = NULL;
    }
    if (device->config.initialize_spi_bus) {
        esp_err_t error = spi_bus_free(device->config.spi_host);
        if ((first_error == ESP_OK) && (error != ESP_OK)) first_error = error;
    }
    device->initialized = false;
    device->mode = NRF24_MODE_POWER_DOWN;
    return first_error;
}

esp_err_t nrf24l01_power_down(nrf24l01_t *device)
{
    if (!nrf24l01_is_ready(device)) return ESP_ERR_INVALID_STATE;
    esp_err_t error = gpio_set_level(device->config.ce_pin, 0);
    if (error != ESP_OK) return error;
    error = nrf24l01_set_config_bits(device, 0U, NRF24_CONFIG_PWR_UP);
    if (error == ESP_OK) device->mode = NRF24_MODE_POWER_DOWN;
    return error;
}

esp_err_t nrf24l01_standby(nrf24l01_t *device)
{
    if (!nrf24l01_is_ready(device)) return ESP_ERR_INVALID_STATE;
    bool was_powered_down = device->mode == NRF24_MODE_POWER_DOWN;
    esp_err_t error = gpio_set_level(device->config.ce_pin, 0);
    if (error != ESP_OK) return error;
    error = nrf24l01_set_config_bits(device, NRF24_CONFIG_PWR_UP, 0U);
    if (error != ESP_OK) return error;
    if (was_powered_down) esp_rom_delay_us(NRF24_POWER_UP_DELAY_US);
    device->mode = NRF24_MODE_STANDBY;
    return ESP_OK;
}

esp_err_t nrf24l01_stop_listening(nrf24l01_t *device)
{
    return nrf24l01_standby(device);
}

esp_err_t nrf24l01_start_listening(nrf24l01_t *device)
{
    if (!nrf24l01_is_ready(device)) return ESP_ERR_INVALID_STATE;

    esp_err_t error = gpio_set_level(device->config.ce_pin, 0);
    if (error != ESP_OK) return error;
    size_t address_size = (size_t)device->config.address_width + 2U;
    if ((error = nrf24l01_write_register(device, NRF24_RX_ADDR_P0,
                                         device->rx_pipe0_address,
                                         address_size)) != ESP_OK) return error;

    bool was_powered_down = device->mode == NRF24_MODE_POWER_DOWN;
    error = nrf24l01_set_config_bits(device,
                                     NRF24_CONFIG_PWR_UP | NRF24_CONFIG_PRIM_RX,
                                     0U);
    if (error != ESP_OK) return error;
    if (was_powered_down) esp_rom_delay_us(NRF24_POWER_UP_DELAY_US);
    if ((error = gpio_set_level(device->config.ce_pin, 1)) != ESP_OK) return error;
    esp_rom_delay_us(NRF24_STANDBY_TO_ACTIVE_US);
    device->mode = NRF24_MODE_PRX;
    return ESP_OK;
}

esp_err_t nrf24l01_set_tx_address(nrf24l01_t *device,
                                  const uint8_t *address)
{
    if (!nrf24l01_is_ready(device) || (address == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t error = nrf24l01_stop_listening(device);
    if (error != ESP_OK) return error;
    size_t address_size = (size_t)device->config.address_width + 2U;
    memcpy(device->tx_address, address, address_size);
    return nrf24l01_write_register(device, NRF24_TX_ADDR, address,
                                   address_size);
}

esp_err_t nrf24l01_open_rx_pipe(nrf24l01_t *device, nrf24_pipe_t pipe,
                                const uint8_t *address)
{
    if (!nrf24l01_is_ready(device) || (address == NULL) ||
        (pipe > NRF24_PIPE_5)) return ESP_ERR_INVALID_ARG;

    esp_err_t error = nrf24l01_stop_listening(device);
    if (error != ESP_OK) return error;
    size_t address_size = (size_t)device->config.address_width + 2U;
    size_t register_size = (pipe <= NRF24_PIPE_1) ? address_size : 1U;
    if ((error = nrf24l01_write_register(device,
                                         (uint8_t)(NRF24_RX_ADDR_P0 + pipe),
                                         address, register_size)) != ESP_OK) return error;
    if (pipe == NRF24_PIPE_0) {
        memcpy(device->rx_pipe0_address, address, address_size);
    }
    if ((error = nrf24l01_write_u8(device, (uint8_t)(NRF24_RX_PW_P0 + pipe),
                                   device->config.payload_size)) != ESP_OK) return error;

    uint8_t enabled = 0U;
    if ((error = nrf24l01_read_u8(device, NRF24_EN_RXADDR, &enabled)) != ESP_OK) return error;
    enabled |= NRF24_PIPE_BIT(pipe);
    device->config.enabled_rx_pipes = enabled;
    if ((error = nrf24l01_write_u8(device, NRF24_EN_RXADDR, enabled)) != ESP_OK) {
        return error;
    }

    if (device->config.enable_dynamic_payloads ||
        device->config.enable_ack_payloads) {
        uint8_t dynpd = 0U;
        if ((error = nrf24l01_read_u8(device, NRF24_DYNPD, &dynpd)) != ESP_OK) {
            return error;
        }
        return nrf24l01_write_u8(device, NRF24_DYNPD,
                                 dynpd | NRF24_PIPE_BIT(pipe));
    }
    return ESP_OK;
}

esp_err_t nrf24l01_close_rx_pipe(nrf24l01_t *device, nrf24_pipe_t pipe)
{
    if (!nrf24l01_is_ready(device) || (pipe > NRF24_PIPE_5)) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t error = nrf24l01_stop_listening(device);
    if (error != ESP_OK) return error;
    uint8_t enabled = 0U;
    if ((error = nrf24l01_read_u8(device, NRF24_EN_RXADDR, &enabled)) != ESP_OK) return error;
    enabled &= (uint8_t)~NRF24_PIPE_BIT(pipe);
    device->config.enabled_rx_pipes = enabled;
    if ((error = nrf24l01_write_u8(device, NRF24_EN_RXADDR, enabled)) != ESP_OK) {
        return error;
    }

    uint8_t dynpd = 0U;
    if ((error = nrf24l01_read_u8(device, NRF24_DYNPD, &dynpd)) != ESP_OK) {
        return error;
    }
    return nrf24l01_write_u8(device, NRF24_DYNPD,
                             dynpd & (uint8_t)~NRF24_PIPE_BIT(pipe));
}

esp_err_t nrf24l01_write_ack_payload(nrf24l01_t *device,
                                     nrf24_pipe_t pipe,
                                     const void *data, size_t data_size)
{
    if (!nrf24l01_is_ready(device) || !device->config.enable_ack_payloads ||
        (pipe > NRF24_PIPE_5) || (data == NULL) || (data_size == 0U) ||
        (data_size > NRF24_MAX_PAYLOAD_SIZE)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t pipe_bit = NRF24_PIPE_BIT(pipe);
    if (((device->config.enabled_rx_pipes & pipe_bit) == 0U) ||
        ((device->config.auto_ack_pipes & pipe_bit) == 0U)) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t error = nrf24l01_write_payload(
        device, NRF24_CMD_W_ACK_PAYLOAD | (uint8_t)pipe, data, data_size);
    if ((error == ESP_OK) && ((device->status & NRF24_STATUS_TX_FULL) != 0U)) {
        return ESP_ERR_NO_MEM;
    }
    return error;
}

esp_err_t nrf24l01_transmit(nrf24l01_t *device, const void *data,
                            size_t data_size, uint32_t timeout_ms)
{
    bool dynamic_enabled = (device != NULL) &&
                           (device->config.enable_dynamic_payloads ||
                            device->config.enable_ack_payloads);
    if (!nrf24l01_is_ready(device) || (data == NULL) ||
        (data_size == 0U) || (data_size > NRF24_MAX_PAYLOAD_SIZE) ||
        (!dynamic_enabled && (data_size != device->config.payload_size)) ||
        (timeout_ms == 0U)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t error = gpio_set_level(device->config.ce_pin, 0);
    if (error != ESP_OK) return error;

    size_t address_size = (size_t)device->config.address_width + 2U;
    if ((device->config.auto_ack_pipes & NRF24_PIPE_BIT(NRF24_PIPE_0)) != 0U) {
        if ((error = nrf24l01_write_register(device, NRF24_RX_ADDR_P0,
                                             device->tx_address,
                                             address_size)) != ESP_OK) return error;
    }

    bool was_powered_down = device->mode == NRF24_MODE_POWER_DOWN;
    if ((error = nrf24l01_set_config_bits(device, NRF24_CONFIG_PWR_UP,
                                          NRF24_CONFIG_PRIM_RX)) != ESP_OK) return error;
    if (was_powered_down) esp_rom_delay_us(NRF24_POWER_UP_DELAY_US);
    if ((error = nrf24l01_write_u8(device, NRF24_STATUS,
                                   NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT)) != ESP_OK) return error;
    if ((error = nrf24l01_command(device, NRF24_CMD_FLUSH_TX)) != ESP_OK) return error;
    if ((error = nrf24l01_write_payload(device, NRF24_CMD_W_TX_PAYLOAD,
                                        data, data_size)) != ESP_OK) return error;

    device->mode = NRF24_MODE_PTX;
    if ((error = gpio_set_level(device->config.ce_pin, 1)) != ESP_OK) return error;
    esp_rom_delay_us(NRF24_CE_PULSE_US);
    if ((error = gpio_set_level(device->config.ce_pin, 0)) != ESP_OK) return error;

    int64_t deadline = esp_timer_get_time() + ((int64_t)timeout_ms * 1000LL);
    while (esp_timer_get_time() < deadline) {
        if ((error = nrf24l01_command(device, NRF24_CMD_NOP)) != ESP_OK) return error;
        if ((device->status & NRF24_STATUS_TX_DS) != 0U) {
            device->mode = NRF24_MODE_STANDBY;
            return nrf24l01_write_u8(device, NRF24_STATUS, NRF24_STATUS_TX_DS);
        }
        if ((device->status & NRF24_STATUS_MAX_RT) != 0U) {
            (void)nrf24l01_write_u8(device, NRF24_STATUS, NRF24_STATUS_MAX_RT);
            (void)nrf24l01_command(device, NRF24_CMD_FLUSH_TX);
            device->mode = NRF24_MODE_STANDBY;
            return ESP_ERR_TIMEOUT;
        }
        esp_rom_delay_us(NRF24_POLL_INTERVAL_US);
    }

    (void)nrf24l01_power_down(device);
    (void)nrf24l01_command(device, NRF24_CMD_FLUSH_TX);
    return ESP_ERR_TIMEOUT;
}

esp_err_t nrf24l01_data_available(nrf24l01_t *device, bool *available,
                                  nrf24_pipe_t *pipe)
{
    if (!nrf24l01_is_ready(device) || (available == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t fifo = 0U;
    esp_err_t error = nrf24l01_read_u8(device, NRF24_FIFO_STATUS, &fifo);
    if (error != ESP_OK) return error;
    *available = (fifo & NRF24_FIFO_RX_EMPTY) == 0U;
    if (*available && (pipe != NULL)) {
        if ((error = nrf24l01_command(device, NRF24_CMD_NOP)) != ESP_OK) return error;
        uint8_t pipe_number = (device->status & NRF24_STATUS_RX_P_NO_MASK) >>
                              NRF24_STATUS_RX_P_NO_POS;
        if (pipe_number >= NRF24_PIPE_COUNT) return ESP_ERR_INVALID_RESPONSE;
        *pipe = (nrf24_pipe_t)pipe_number;
    }
    return ESP_OK;
}

esp_err_t nrf24l01_receive(nrf24l01_t *device, void *data,
                           size_t data_capacity, size_t *received_size,
                           nrf24_pipe_t *pipe)
{
    if (!nrf24l01_is_ready(device) || (data == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    bool available = false;
    esp_err_t error = nrf24l01_data_available(device, &available, NULL);
    if (error != ESP_OK) return error;
    if (!available) return ESP_ERR_NOT_FOUND;

    size_t payload_size = device->config.payload_size;
    if (device->config.enable_dynamic_payloads ||
        device->config.enable_ack_payloads) {
        uint8_t dynamic_size = 0U;
        if ((error = nrf24l01_read_payload_width(device, &dynamic_size)) != ESP_OK) {
            return error;
        }
        if ((dynamic_size == 0U) || (dynamic_size > NRF24_MAX_PAYLOAD_SIZE)) {
            (void)nrf24l01_command(device, NRF24_CMD_FLUSH_RX);
            (void)nrf24l01_write_u8(device, NRF24_STATUS, NRF24_STATUS_RX_DR);
            return ESP_ERR_INVALID_RESPONSE;
        }
        payload_size = dynamic_size;
    }
    if (data_capacity < payload_size) return ESP_ERR_INVALID_SIZE;

    uint8_t status = 0U;
    error = nrf24l01_read_payload(device, data, payload_size, &status);
    if (error != ESP_OK) return error;
    uint8_t pipe_number = (status & NRF24_STATUS_RX_P_NO_MASK) >>
                          NRF24_STATUS_RX_P_NO_POS;
    if (pipe_number >= NRF24_PIPE_COUNT) return ESP_ERR_INVALID_RESPONSE;
    if (pipe != NULL) *pipe = (nrf24_pipe_t)pipe_number;
    if (received_size != NULL) *received_size = payload_size;
    return nrf24l01_write_u8(device, NRF24_STATUS, NRF24_STATUS_RX_DR);
}

static void nrf24l01_format_address(const uint8_t *address, size_t size,
                                    char *output, size_t output_size)
{
    size_t used = 0U;
    for (size_t i = 0U; (i < size) && (used < output_size); ++i) {
        int written = snprintf(&output[used], output_size - used,
                               "%s%02X", (i == 0U) ? "" : ":", address[i]);
        if (written < 0) break;
        used += (size_t)written;
    }
}

esp_err_t nrf24l01_print_diagnostics(nrf24l01_t *device)
{
    if (!nrf24l01_is_ready(device)) return ESP_ERR_INVALID_STATE;
    bool connected = false;
    esp_err_t error = nrf24l01_is_connected(device, &connected);
    if (error != ESP_OK) return error;
    if (!connected) {
        ESP_LOGE(TAG, "Diagnostico: sin conexion SPI con el nRF24L01+");
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t config, en_aa, en_rx, aw, retr, channel, rf_setup;
    uint8_t status, observe, rpd, fifo, dynpd, feature;
    uint8_t payload_width[NRF24_PIPE_COUNT];
#define READ_DIAG(reg_, value_) do {                                   \
        error = nrf24l01_read_u8(device, (reg_), &(value_));            \
        if (error != ESP_OK) return error;                              \
    } while (0)
    READ_DIAG(NRF24_CONFIG, config);
    READ_DIAG(NRF24_EN_AA, en_aa);
    READ_DIAG(NRF24_EN_RXADDR, en_rx);
    READ_DIAG(NRF24_SETUP_AW, aw);
    READ_DIAG(NRF24_SETUP_RETR, retr);
    READ_DIAG(NRF24_RF_CH, channel);
    READ_DIAG(NRF24_RF_SETUP, rf_setup);
    READ_DIAG(NRF24_STATUS, status);
    READ_DIAG(NRF24_OBSERVE_TX, observe);
    READ_DIAG(NRF24_RPD, rpd);
    READ_DIAG(NRF24_FIFO_STATUS, fifo);
    READ_DIAG(NRF24_DYNPD, dynpd);
    READ_DIAG(NRF24_FEATURE, feature);
    for (uint8_t pipe_number = 0U; pipe_number < NRF24_PIPE_COUNT;
         ++pipe_number) {
        READ_DIAG((uint8_t)(NRF24_RX_PW_P0 + pipe_number),
                  payload_width[pipe_number]);
    }
#undef READ_DIAG

    const uint8_t actual_aw = aw & NRF24_SETUP_AW_MASK;
    if (actual_aw < NRF24_ADDRESS_WIDTH_3_BYTES ||
        actual_aw > NRF24_ADDRESS_WIDTH_5_BYTES) {
        ESP_LOGE(TAG, "SETUP_AW invalido: 0x%02X", aw);
        return ESP_ERR_INVALID_RESPONSE;
    }
    size_t address_size = (size_t)actual_aw + 2U;
    uint8_t tx_address[NRF24_MAX_ADDRESS_WIDTH] = {0};
    uint8_t rx0_address[NRF24_MAX_ADDRESS_WIDTH] = {0};
    uint8_t rx1_address[NRF24_MAX_ADDRESS_WIDTH] = {0};
    uint8_t rx_lsb[4] = {0};
    if ((error = nrf24l01_read_register(device, NRF24_TX_ADDR,
                                        tx_address, address_size)) != ESP_OK) return error;
    if ((error = nrf24l01_read_register(device, NRF24_RX_ADDR_P0,
                                        rx0_address, address_size)) != ESP_OK) return error;
    if ((error = nrf24l01_read_register(device, NRF24_RX_ADDR_P1,
                                        rx1_address, address_size)) != ESP_OK) return error;
    for (uint8_t index = 0U; index < 4U; ++index) {
        if ((error = nrf24l01_read_u8(device,
                                     (uint8_t)(NRF24_RX_ADDR_P2 + index),
                                     &rx_lsb[index])) != ESP_OK) return error;
    }
    char tx_text[3U * NRF24_MAX_ADDRESS_WIDTH] = {0};
    char rx_text[3U * NRF24_MAX_ADDRESS_WIDTH] = {0};
    char rx1_text[3U * NRF24_MAX_ADDRESS_WIDTH] = {0};
    nrf24l01_format_address(tx_address, address_size, tx_text, sizeof(tx_text));
    nrf24l01_format_address(rx0_address, address_size, rx_text, sizeof(rx_text));
    nrf24l01_format_address(rx1_address, address_size, rx1_text, sizeof(rx1_text));

    const char *rate = ((rf_setup & NRF24_RF_SETUP_RF_DR_LOW) != 0U) ? "250 kbps" :
                       ((rf_setup & NRF24_RF_SETUP_RF_DR_HIGH) != 0U) ? "2 Mbps" : "1 Mbps";
    static const int8_t power_dbm[] = {-18, -12, -6, 0};
    uint8_t power_index = (rf_setup & NRF24_RF_SETUP_RF_PWR_MASK) >>
                          NRF24_RF_SETUP_RF_PWR_POS;

    ESP_LOGI(TAG, "========== nRF24L01+ DIAGNOSTICO ==========");
    ESP_LOGI(TAG, "Conexion SPI : OK");
    ESP_LOGI(TAG, "SPI asignado  : host=%d, reloj=%lu Hz",
             (int)device->config.spi_host,
             (unsigned long)device->config.spi_clock_hz);
    int actual_spi_khz = 0;
    error = spi_device_get_actual_freq(device->spi_handle, &actual_spi_khz);
    if (error == ESP_OK) {
        ESP_LOGI(TAG, "SPI efectivo  : modo=0, reloj=%d kHz", actual_spi_khz);
    } else {
        ESP_LOGW(TAG, "No se pudo consultar reloj SPI: %s", esp_err_to_name(error));
    }
    ESP_LOGI(TAG, "GPIO asignados: MOSI=%d MISO=%d SCLK=%d CSN=%d CE=%d",
             (int)device->config.mosi_pin, (int)device->config.miso_pin,
             (int)device->config.sclk_pin, (int)device->config.csn_pin,
             (int)device->config.ce_pin);
    if (device->config.irq_pin == GPIO_NUM_NC) {
        ESP_LOGI(TAG, "IRQ           : NC");
    } else {
        ESP_LOGI(TAG, "IRQ           : GPIO%d", (int)device->config.irq_pin);
    }
    ESP_LOGI(TAG, "Orden direcciones: LSByte primero, igual que shared.h");
    ESP_LOGI(TAG, "CONFIG       : 0x%02X (PWR_UP=%u PRIM_RX=%u CRC=%s)",
             config, !!(config & NRF24_CONFIG_PWR_UP),
             !!(config & NRF24_CONFIG_PRIM_RX),
             (config & NRF24_CONFIG_EN_CRC) ?
                 ((config & NRF24_CONFIG_CRCO) ? "2 bytes" : "1 byte") : "off");
    ESP_LOGI(TAG, "Canal         : %u (%u MHz)", channel, 2400U + channel);
    ESP_LOGI(TAG, "Radio         : %s, %d dBm", rate, power_dbm[power_index]);
    ESP_LOGI(TAG, "Direccion     : %u bytes, TX=%s, RX_P0=%s",
             (unsigned)address_size, tx_text, rx_text);
    const bool width_matches = actual_aw == device->config.address_width;
    const bool tx_matches = width_matches &&
        memcmp(tx_address, device->tx_address, address_size) == 0;
    /* En PTX, transmit() cambia RX_P0 a TX para recibir el ACK. */
    const uint8_t *expected_rx0 =
        ((config & NRF24_CONFIG_PRIM_RX) == 0U &&
         (en_aa & NRF24_PIPE_BIT(NRF24_PIPE_0)) != 0U &&
         memcmp(rx0_address, device->tx_address, address_size) == 0)
            ? device->tx_address : device->rx_pipe0_address;
    const bool rx_matches = width_matches &&
        memcmp(rx0_address, expected_rx0, address_size) == 0;
    ESP_LOGI(TAG, "Verificacion  : ancho=%s TX=%s RX_P0=%s canal=%s",
             width_matches ? "OK" : "DIFIERE",
             tx_matches ? "OK" : "DIFIERE", rx_matches ? "OK" : "DIFIERE",
             channel == device->config.channel ? "OK" : "DIFIERE");
    ESP_LOGI(TAG, "RX pipes      : P1=%s P2..P5 LSB=%02X:%02X:%02X:%02X",
             rx1_text, rx_lsb[0], rx_lsb[1], rx_lsb[2], rx_lsb[3]);
    ESP_LOGI(TAG, "Pipes         : EN_RX=0x%02X EN_AA=0x%02X DYNPD=0x%02X",
             en_rx, en_aa, dynpd);
    ESP_LOGI(TAG, "Payload RX    : P0..P5=%u,%u,%u,%u,%u,%u bytes",
             payload_width[0], payload_width[1], payload_width[2],
             payload_width[3], payload_width[4], payload_width[5]);
    ESP_LOGI(TAG, "Reintentos    : ARD=%u us ARC=%u", 250U * (((retr >> 4U) & 0x0FU) + 1U),
             retr & NRF24_SETUP_RETR_ARC_MASK);
    ESP_LOGI(TAG, "STATUS/FIFO   : 0x%02X / 0x%02X", status, fifo);
    ESP_LOGI(TAG, "Perdidas/ARC  : %u / %u", observe >> 4U,
             observe & NRF24_OBSERVE_TX_ARC_CNT_MASK);
    ESP_LOGI(TAG, "RPD/FEATURE   : %u / 0x%02X", !!(rpd & NRF24_RPD_DETECTED), feature);
    ESP_LOGI(TAG, "Funciones     : payload dinamico=%s ACK payload=%s",
             (feature & NRF24_FEATURE_EN_DPL) ? "ON" : "OFF",
             (feature & NRF24_FEATURE_EN_ACK_PAY) ? "ON" : "OFF");
    ESP_LOGI(TAG, "============================================");
    return ESP_OK;
}
