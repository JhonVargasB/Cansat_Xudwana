# nRF24L01+ component

This component provides synchronous fixed-payload PTX and PRX operation for an
nRF24L01+ connected to an ESP32. It supports all six RX pipes, automatic
acknowledgement, automatic retransmission, blocking transmission with timeout,
FIFO reception, power modes, and register diagnostics. Dynamic payloads and ACK
payloads are intentionally left for a later layer.

## Ownership model

- With `initialize_spi_bus = true` (the default), the driver initializes and
  owns the selected SPI bus. `nrf24l01_deinit()` removes the radio and frees
  that bus.
- With `initialize_spi_bus = false`, the application must initialize the SPI
  bus first. The driver only adds/removes its device and never frees the bus.
- Calls are synchronous. The application must serialize access when several
  tasks use the same radio instance.

## Minimal initialization

```c
#include "esp_check.h"
#include "nrf24l01.h"

static nrf24l01_t radio;

static esp_err_t radio_init(void)
{
    nrf24l01_config_t config = NRF24L01_TX_DEFAULT_CONFIG();

    config.channel = 76;
    config.enable_dynamic_payloads = true;
    config.enable_ack_payloads = true;

    esp_err_t error = nrf24l01_init(&radio, &config);
    if (error != ESP_OK) {
        return error;
    }
    return nrf24l01_print_diagnostics(&radio);
}
```

The radio starts powered down with CE low, a fixed payload size on pipe 0, the
configured RF channel/data rate/power/CRC/retry values, cleared IRQ flags, and
empty FIFOs. Always call `nrf24l01_deinit()` when the instance is no longer
needed. Addresses passed to the API are LSByte first.

## Transmitter

```c
static const uint8_t address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
uint8_t payload[32] = "Hola desde PTX";

ESP_ERROR_CHECK(nrf24l01_set_tx_address(&radio, address));
ESP_ERROR_CHECK(nrf24l01_transmit(&radio, payload, sizeof(payload), 100));
```

## Receiver

```c
static const uint8_t address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
uint8_t payload[32];
bool available;

ESP_ERROR_CHECK(nrf24l01_open_rx_pipe(&radio, NRF24_PIPE_0, address));
ESP_ERROR_CHECK(nrf24l01_start_listening(&radio));

ESP_ERROR_CHECK(nrf24l01_data_available(&radio, &available, NULL));
if (available) {
    ESP_ERROR_CHECK(nrf24l01_receive(&radio, payload, sizeof(payload), NULL, NULL));
}
```

Calls are not internally synchronized yet. Protect one radio instance with a
mutex before sharing it between tasks.

## Bidirectional telemetry with ACK payload

The PRX queues a command before the next telemetry packet arrives:

```c
uint8_t command[] = {0x01, 0x2A};
ESP_ERROR_CHECK(nrf24l01_write_ack_payload(
    &radio, NRF24_PIPE_0, command, sizeof(command)));
```

The PTX sends telemetry normally. A successful ACK payload is placed in its RX
FIFO, so the existing receive API reads it:

```c
uint8_t command[32];
ESP_ERROR_CHECK(nrf24l01_transmit(
    &radio, telemetry, telemetry_size, 100));

bool command_available = false;
ESP_ERROR_CHECK(nrf24l01_data_available(
    &radio, &command_available, NULL));
if (command_available) {
    size_t command_size = 0;
    ESP_ERROR_CHECK(nrf24l01_receive(
        &radio, command, sizeof(command), &command_size, NULL));
}
```

ACK payloads imply dynamic payload support. The driver enables `EN_DPL`,
`EN_ACK_PAY`, and the corresponding `DYNPD` pipe bits when
`enable_ack_payloads` is true.
