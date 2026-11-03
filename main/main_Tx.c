#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nrf24_command_protocol.h"
#include "nrf24l01.h"
#include "shared.h"

/* PTX: telemetria de 32 bytes, comandos por ACK de 7 y respuestas de 9.
 * Prueba de protocolo: las acciones son logs, sin SD ni calibracion real.
 */
#define COMMANDS_PER_CYCLE 6U
#define COMMAND_LED_GPIO GPIO_NUM_42

typedef struct {
    bool valid;
    uint16_t sequence;
    uint8_t command;
    uint8_t result;
    uint8_t state;
} command_history_t;

static const char *TAG = "NRF24_TX";
static const uint8_t s_radio_address[NRF24_MAX_ADDRESS_WIDTH] =
    NRF24_TELEMETRY_ADDRESS;

/* Prototipos. */
static esp_err_t gpio_init(void);
static const char *command_name(uint8_t command);
static bool command_is_valid(const nrf24_command_packet_t *command, size_t length);
static nrf24_command_result_t execute_command(const nrf24_command_packet_t *command);
static esp_err_t send_command_response(nrf24l01_t *radio,
                                      const nrf24_command_packet_t *command,
                                      uint8_t result, uint8_t state);
static void process_ack_payload(nrf24l01_t *radio, command_history_t *history);

void app_main(void)
{
    ESP_ERROR_CHECK(gpio_init());

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
    ESP_LOGI(TAG, "Prueba de comandos: ACCEPTED confirma el procesamiento del log; "
                 "sin registro SD ni calibracion real, state=0");

    uint32_t sequence = 0;
    uint32_t telemetry_period_ms = NRF24_EXAMPLE_TELEMETRY_PERIOD_DEFAULT_MS;
    command_history_t history = {0};

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
            process_ack_payload(&radio, &history);
        } else {
            ESP_LOGW(TAG, "Fallo TX #%" PRIu32 ": %s",
                     telemetry.sequence, esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(telemetry_period_ms));
    }
}

/* Definiciones de funciones auxiliares. */
static esp_err_t gpio_init(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = UINT64_C(1) << COMMAND_LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_set_level(COMMAND_LED_GPIO, 0);
    if (err != ESP_OK) {
        return err;
    }
    return gpio_config(&config);
}

static const char *command_name(uint8_t command)
{
    switch (command) {
        case NRF24_COMMAND_REC_ON: return "REC_ON";
        case NRF24_COMMAND_REC_OFF: return "REC_OFF";
        case NRF24_COMMAND_CALIBRATE: return "CALIBRAR";
        default: return "DESCONOCIDO";
    }
}

static bool command_is_valid(const nrf24_command_packet_t *command, size_t length)
{
    return command != NULL && length == sizeof(*command)
        && command->magic == NRF24_COMMAND_MAGIC
        && command->crc16 == nrf24_command_crc16(
            command, offsetof(nrf24_command_packet_t, crc16));
}

static nrf24_command_result_t execute_command(const nrf24_command_packet_t *command)
{
    switch (command->command) {
        case NRF24_COMMAND_REC_ON:
            ESP_LOGI(TAG, "REC_ON recibido (seq=%u): prueba de inicio de registro",
                     (unsigned)command->sequence);
            break;
        case NRF24_COMMAND_REC_OFF:
            ESP_LOGI(TAG, "REC_OFF recibido (seq=%u): prueba de fin de registro",
                     (unsigned)command->sequence);
            break;
        case NRF24_COMMAND_CALIBRATE:
            ESP_LOGI(TAG, "CALIBRAR recibido (seq=%u): prueba de solicitud de calibracion",
                     (unsigned)command->sequence);
            break;
        default:
            ESP_LOGW(TAG, "Comando desconocido: %u (seq=%u)",
                     (unsigned)command->command, (unsigned)command->sequence);
            return NRF24_COMMAND_RESULT_INVALID_COMMAND;
    }
    return NRF24_COMMAND_RESULT_ACCEPTED;
}

static esp_err_t send_command_response(nrf24l01_t *radio,
                                      const nrf24_command_packet_t *command,
                                      uint8_t result, uint8_t state)
{
    nrf24_command_ack_packet_t response = {
        .magic = NRF24_COMMAND_ACK_MAGIC,
        .sequence = command->sequence,
        .command = command->command,
        .result = result,
        .state = state,
    };
    response.crc16 = nrf24_command_crc16(
        &response, offsetof(nrf24_command_ack_packet_t, crc16));
    esp_err_t err = nrf24l01_transmit(
        radio, &response, sizeof(response), NRF24_TX_TIMEOUT_MS);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Respuesta entregada: seq=%u comando=%s resultado=%u estado=%u",
                 (unsigned)response.sequence, command_name(response.command),
                 (unsigned)response.result, (unsigned)response.state);
    } else {
        ESP_LOGW(TAG, "Fallo de respuesta seq=%u: %s; un reintento no repite la accion",
                 (unsigned)response.sequence, esp_err_to_name(err));
    }
    return err;
}

static void process_ack_payload(nrf24l01_t *radio, command_history_t *history)
{
    /* Iterar, sin recursion: el ACK de una respuesta tambien puede traer comandos.
     * Limitar el trabajo para que una cadena de reintentos no detenga telemetria.
     */
    for (unsigned count = 0; count < COMMANDS_PER_CYCLE; ++count) {
        bool available = false;
        nrf24_pipe_t pipe = NRF24_PIPE_0;
        esp_err_t err = nrf24l01_data_available(radio, &available, &pipe);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "No se pudo consultar ACK: %s", esp_err_to_name(err));
            return;
        }
        if (!available) return;

        uint8_t payload[NRF24_MAX_PAYLOAD_SIZE];
        size_t length = 0;
        err = nrf24l01_receive(radio, payload, sizeof(payload), &length, &pipe);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "No se pudo leer ACK: %s", esp_err_to_name(err));
            return;
        }
        if (length != sizeof(nrf24_command_packet_t)) {
            ESP_LOGW(TAG, "ACK descartado: longitud=%u, esperada=7", (unsigned)length);
            continue;
        }

        nrf24_command_packet_t command;
        memcpy(&command, payload, sizeof(command));
        if (!command_is_valid(&command, length)) {
            ESP_LOGW(TAG, "ACK descartado: magic o CRC incorrecto");
            continue;
        }

        ESP_ERROR_CHECK(gpio_set_level(COMMAND_LED_GPIO, 1));
        ESP_LOGI(TAG, "ACK recibido: seq=%u comando=%s (%u)",
                 (unsigned)command.sequence, command_name(command.command),
                 (unsigned)command.command);
        uint8_t result;
        uint8_t state = 0; /* Sin maquina de estados en esta prueba. */
        if (history->valid && command.sequence == history->sequence) {
            state = history->state;
            result = command.command == history->command
                ? NRF24_COMMAND_RESULT_DUPLICATE
                : NRF24_COMMAND_RESULT_INVALID_COMMAND;
            ESP_LOGW(TAG, "Secuencia repetida: %u, resultado=%u; no se ejecuta",
                     (unsigned)command.sequence, (unsigned)result);
        } else {
            result = execute_command(&command);
            *history = (command_history_t) {
                .valid = true,
                .sequence = command.sequence,
                .command = command.command,
                .result = result,
                .state = state,
            };
        }

        (void)send_command_response(radio, &command, result, state);
        /* Volver a consultar RX FIFO incluso si fallo la respuesta:
         * puede haber comandos pendientes recibidos anteriormente.
         */
    }
    ESP_LOGW(TAG, "Limite de comandos por ciclo; pendientes para siguiente telemetria");
}
