#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "nrf24l01_defs.h"

#include "shared.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /* SPI / GPIO */
    spi_host_device_t spi_host;
    gpio_num_t mosi_pin;
    gpio_num_t miso_pin;
    gpio_num_t sclk_pin;

    gpio_num_t ce_pin;
    gpio_num_t csn_pin;
    /* Use GPIO_NUM_NC when the IRQ pin is not connected. */
    gpio_num_t irq_pin;

    uint32_t spi_clock_hz;
    spi_dma_chan_t spi_dma_chan;
    /* False when the application has already initialized spi_host. */
    bool initialize_spi_bus;

    /* Radio configuration */
    uint8_t channel;
    uint8_t payload_size;
    uint8_t enabled_rx_pipes;
    uint8_t auto_ack_pipes;
    bool enable_dynamic_payloads;
    bool enable_ack_payloads;

    nrf24_address_width_t address_width;
    nrf24_auto_retransmit_count_t auto_retransmit_count;
    nrf24_auto_retransmit_delay_t auto_retransmit_delay;

    nrf24_output_power_t output_power;
    nrf24_air_data_rate_t air_data_rate;
    nrf24_crc_length_t crc_length;

} nrf24l01_config_t;


#define NRF24L01_TX_DEFAULT_CONFIG()                                   \
    {                                                                  \
        .spi_host = HOST_NRF24_TX_SPI3,                               \
        .mosi_pin = PIN_NRF24_TX_MOSI,                                \
        .miso_pin = PIN_NRF24_TX_MISO,                                \
        .sclk_pin = PIN_NRF24_TX_SCLK,                                \
        .ce_pin = PIN_NRF24_TX_CE,                                    \
        .csn_pin = PIN_NRF24_TX_CSN,                                  \
        .irq_pin = PIN_NRF24_TX_IRQ,                                  \
        .spi_clock_hz = 8000000U,                                      \
        .spi_dma_chan = SPI_DMA_CH_AUTO,                               \
        .initialize_spi_bus = true,                                    \
        .channel = 76U,                                                \
        .payload_size = NRF24_MAX_PAYLOAD_SIZE,                        \
        .enabled_rx_pipes = NRF24_PIPE_BIT(NRF24_PIPE_0),              \
        .auto_ack_pipes = NRF24_PIPE_BIT(NRF24_PIPE_0),                \
        .enable_dynamic_payloads = true,                               \
        .enable_ack_payloads = true,                                   \
        .address_width = NRF24_ADDRESS_WIDTH_5_BYTES,                  \
        .auto_retransmit_count = NRF24_RETRANSMIT_COUNT_10,            \
        .auto_retransmit_delay = NRF24_RETRANSMIT_DELAY_1500_US,       \
        .output_power = NRF24_OUTPUT_POWER_0_DBM,                      \
        .air_data_rate = NRF24_DATA_RATE_250_KBPS,                     \
        .crc_length = NRF24_CRC_2_BYTES,                               \
    }

#define NRF24L01_RX_DEFAULT_CONFIG()                                   \
    {                                                                  \
        .spi_host = HOST_NRF24_RX_SPI3,                               \
        .mosi_pin = PIN_NRF24_RX_MOSI,                                \
        .miso_pin = PIN_NRF24_RX_MISO,                                \
        .sclk_pin = PIN_NRF24_RX_SCLK,                                \
        .ce_pin = PIN_NRF24_RX_CE,                                    \
        .csn_pin = PIN_NRF24_RX_CSN,                                  \
        .irq_pin = PIN_NRF24_RX_IRQ,                                  \
        .spi_clock_hz = 8000000U,                                      \
        .spi_dma_chan = SPI_DMA_CH_AUTO,                               \
        .initialize_spi_bus = true,                                    \
        .channel = 76U,                                                \
        .payload_size = NRF24_MAX_PAYLOAD_SIZE,                        \
        .enabled_rx_pipes = NRF24_PIPE_BIT(NRF24_PIPE_0),              \
        .auto_ack_pipes = NRF24_PIPE_BIT(NRF24_PIPE_0),                \
        .enable_dynamic_payloads = true,                               \
        .enable_ack_payloads = true,                                   \
        .address_width = NRF24_ADDRESS_WIDTH_5_BYTES,                  \
        .auto_retransmit_count = NRF24_RETRANSMIT_COUNT_10,            \
        .auto_retransmit_delay = NRF24_RETRANSMIT_DELAY_1500_US,       \
        .output_power = NRF24_OUTPUT_POWER_0_DBM,                      \
        .air_data_rate = NRF24_DATA_RATE_250_KBPS,                     \
        .crc_length = NRF24_CRC_2_BYTES,                               \
    }

typedef struct
{
    spi_device_handle_t spi_handle;

    nrf24l01_config_t config;

    uint8_t status;
    nrf24_mode_t mode;
    bool initialized;

    uint8_t tx_address[NRF24_MAX_ADDRESS_WIDTH];
    uint8_t rx_pipe0_address[NRF24_MAX_ADDRESS_WIDTH];
} nrf24l01_t;

esp_err_t nrf24l01_init(nrf24l01_t *device,
                        const nrf24l01_config_t *config);
esp_err_t nrf24l01_deinit(nrf24l01_t *device);

esp_err_t nrf24l01_is_connected(nrf24l01_t *device, bool *connected);
esp_err_t nrf24l01_print_diagnostics(nrf24l01_t *device);

esp_err_t nrf24l01_power_down(nrf24l01_t *device);
esp_err_t nrf24l01_standby(nrf24l01_t *device);
esp_err_t nrf24l01_start_listening(nrf24l01_t *device);
esp_err_t nrf24l01_stop_listening(nrf24l01_t *device);

/** Addresses are supplied LSByte first, as required by the radio SPI protocol. */
esp_err_t nrf24l01_set_tx_address(nrf24l01_t *device,
                                  const uint8_t *address);
esp_err_t nrf24l01_open_rx_pipe(nrf24l01_t *device, nrf24_pipe_t pipe,
                                const uint8_t *address);
esp_err_t nrf24l01_close_rx_pipe(nrf24l01_t *device, nrf24_pipe_t pipe);

/** Queues one payload to be returned inside the next ACK for this pipe. */
esp_err_t nrf24l01_write_ack_payload(nrf24l01_t *device,
                                     nrf24_pipe_t pipe,
                                     const void *data, size_t data_size);

/** Blocking fixed-payload transmission. */
esp_err_t nrf24l01_transmit(nrf24l01_t *device, const void *data,
                            size_t data_size, uint32_t timeout_ms);

esp_err_t nrf24l01_data_available(nrf24l01_t *device, bool *available,
                                  nrf24_pipe_t *pipe);
esp_err_t nrf24l01_receive(nrf24l01_t *device, void *data,
                           size_t data_capacity, size_t *received_size,
                           nrf24_pipe_t *pipe);

/* Synchronous low-level access for future advanced features. */
esp_err_t nrf24l01_read_register(nrf24l01_t *device, uint8_t reg,
                                 void *data, size_t data_size);
esp_err_t nrf24l01_write_register(nrf24l01_t *device, uint8_t reg,
                                  const void *data, size_t data_size);
esp_err_t nrf24l01_command(nrf24l01_t *device, uint8_t command);

#ifdef __cplusplus
}
#endif
