#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nrf24_example_protocol.h"
#include "nrf24l01.h"
#include "shared.h"

/*
 * CanSat / PTX example:
 *   - sends telemetry periodically;
 *   - receives ground commands inside the automatic ACK payload;
 *   - reports the result of the last command in the next telemetry packet.
 */

static const char *TAG = "NRF24_TX";

/* nRF24L01+ sends the least-significant address byte first over the air. */
static const uint8_t s_radio_address[NRF24_MAX_ADDRESS_WIDTH] =
    NRF24_TELEMETRY_ADDRESS;

static bool command_is_valid(const nrf24_command_packet_t *command,
                             size_t payload_length)
{
    return (command != NULL)
           && (payload_length == sizeof(*command));
}

static nrf24_command_status_t execute_command(
    const nrf24_command_packet_t *command,
    uint32_t *telemetry_period_ms)
{
    switch ((nrf24_command_code_t)command->command) {
        case NRF24_COMMAND_PING:
            ESP_LOGI(TAG, "PING recibido en ACK (id=%u)", command->command_id);
            return NRF24_COMMAND_STATUS_OK;

        case NRF24_COMMAND_SET_TELEMETRY_PERIOD:
            if ((command->argument < TELEMETRY_PERIOD_MIN_MS)
                || (command->argument > TELEMETRY_PERIOD_MAX_MS)) {
                ESP_LOGW(TAG, "Periodo fuera de rango: %" PRIu32 " ms",
                         command->argument);
                return NRF24_COMMAND_STATUS_INVALID;
            }

            *telemetry_period_ms = command->argument;
            ESP_LOGI(TAG, "Nuevo periodo: %" PRIu32 " ms (id=%u)",
                     *telemetry_period_ms, command->command_id);
            return NRF24_COMMAND_STATUS_OK;

        default:
            ESP_LOGW(TAG, "Comando no soportado: %u (id=%u)",
                     command->command, command->command_id);
            return NRF24_COMMAND_STATUS_UNSUPPORTED;
    }
}

static void process_ack_payload(nrf24l01_t *radio,
                                uint16_t *last_command_id,
                                uint8_t *last_command_status,
                                uint32_t *telemetry_period_ms)
{
    bool available = false;
    nrf24_pipe_t pipe = NRF24_PIPE_0;
    esp_err_t err = nrf24l01_data_available(radio, &available, &pipe);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No se pudo consultar el ACK payload: %s",
                 esp_err_to_name(err));
        return;
    }

    if (!available) {
        ESP_LOGD(TAG, "ACK recibido sin comando");
        return;
    }

    nrf24_command_packet_t command = {0};
    size_t payload_length = 0;
    nrf24_pipe_t received_pipe = NRF24_PIPE_0;
    err = nrf24l01_receive(radio, &command, sizeof(command),
                           &payload_length, &received_pipe);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No se pudo leer el ACK payload: %s",
                 esp_err_to_name(err));
        return;
    }

    if (!command_is_valid(&command, payload_length)) {
        ESP_LOGW(TAG, "ACK payload invalido (%u bytes, pipe %u)",
                 (unsigned)payload_length, received_pipe);
        return;
    }

    /* A lost ACK can cause the radio to deliver the same command again. */
    if (command.command_id == *last_command_id) {
        ESP_LOGW(TAG, "Comando duplicado ignorado (id=%u)",
                 command.command_id);
        return;
    }

    *last_command_id = command.command_id;
    *last_command_status = execute_command(&command, telemetry_period_ms);
}

void app_main(void)
{
    nrf24l01_t radio = {0};
    nrf24l01_config_t config = NRF24L01_TX_DEFAULT_CONFIG();

    ESP_ERROR_CHECK(nrf24l01_init(&radio, &config));
    ESP_ERROR_CHECK(nrf24l01_set_tx_address(&radio, s_radio_address));

    bool connected = false;
    ESP_ERROR_CHECK(nrf24l01_is_connected(&radio, &connected));
    if (!connected) {
        ESP_LOGE(TAG, "El nRF24L01+ no responde por SPI");
        ESP_ERROR_CHECK(ESP_ERR_NOT_FOUND);
    }

    ESP_ERROR_CHECK(nrf24l01_print_diagnostics(&radio));

    uint32_t sequence = 0;
    uint32_t telemetry_period_ms = NRF24_EXAMPLE_TELEMETRY_PERIOD_DEFAULT_MS;
    uint16_t last_command_id = 0;
    uint8_t last_command_status = NRF24_COMMAND_STATUS_NONE;

    while (true) {
        telemetry_data_t telemetry = {
            .sequence = sequence++,
            /* Valores de prueba; reemplazar por los sensores del CanSat. */
            .latitude_e7 = -120467890,
            .longitude_e7 = -771234560,
            .altitude_cm = 10000,
            .pressure_pa = 101325,
            .vertical_speed_cms = 0,
            .temperature_centi_c = 2350,
            .yaw_centi_deg = 0,
            .roll_centi_deg = 0,
            .pitch_centi_deg = 0,
            .status = TELEMETRY_STATUS_GPS_FIX
                      | TELEMETRY_STATUS_BMP180_OK
                      | TELEMETRY_STATUS_MPU6050_OK
                      | TELEMETRY_STATUS_NEO6M_OK,
            .satellites = 8,
        };

        esp_err_t err = nrf24l01_transmit(
            &radio, &telemetry, sizeof(telemetry), NRF24_TX_TIMEOUT_MS);

        if (err == ESP_OK) {
            ESP_LOGI(TAG,
                     "Telemetria #%" PRIu32
                     " entregada: alt=%.2f m, temp=%.2f C, sat=%u",
                     telemetry.sequence,
                     telemetry.altitude_cm / TELEMETRY_CENTI_SCALE,
                     telemetry.temperature_centi_c / 100.0,
                     telemetry.satellites);
            process_ack_payload(&radio,
                                &last_command_id,
                                &last_command_status,
                                &telemetry_period_ms);
        } else {
            ESP_LOGW(TAG, "Fallo TX #%" PRIu32 ": %s",
                     telemetry.sequence, esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(telemetry_period_ms));
    }
}
