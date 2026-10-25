#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nrf24_example_protocol.h"
#include "nrf24l01.h"
#include "shared.h"

/*
 * Ground station / PRX example:
 *   - receives telemetry from the CanSat;
 *   - queues a command in the ACK payload;
 *   - that command travels automatically with the ACK for the next packet.
 */

#define RX_POLL_PERIOD_MS   10

static const uint8_t s_radio_address[NRF24_MAX_ADDRESS_WIDTH] =
    NRF24_TELEMETRY_ADDRESS;

static bool telemetry_is_valid(const telemetry_data_t *telemetry,
                               size_t payload_length)
{
    return (telemetry != NULL)
           && (payload_length == sizeof(*telemetry));
}

static esp_err_t queue_ping_command(nrf24l01_t *radio, uint16_t command_id)
{
    const nrf24_command_packet_t command = {
        .command_id = command_id,
        .command = NRF24_COMMAND_PING,
        .argument = 0,
    };

    esp_err_t err = nrf24l01_write_ack_payload(
        radio, NRF24_PIPE_0, &command, sizeof(command));
    return err;
}

static void send_telemetry_over_serial(const telemetry_data_t *telemetry)
{
    /*
     * Formato UART:
     * pitch,roll,yaw,alt_rel_m,temp_c,pres_pa,ax,ay,az,lat,lon,sats
     * La trama no transmite ax/ay/az: son ceros de compatibilidad, no mediciones.
     */
    printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
           "%.2f,%.2f,%.2f,%.7f,%.7f,%u\n",
           (double)telemetry->pitch_centi_deg / TELEMETRY_CENTI_SCALE,
           (double)telemetry->roll_centi_deg / TELEMETRY_CENTI_SCALE,
           (double)telemetry->yaw_centi_deg / TELEMETRY_CENTI_SCALE,
           (double)telemetry->altitude_cm / TELEMETRY_CENTI_SCALE,
           (double)telemetry->temperature_centi_c / TELEMETRY_CENTI_SCALE,
           (double)telemetry->pressure_pa,
           0.0,
           0.0,
           0.0,
           (double)telemetry->latitude_e7 / TELEMETRY_COORDINATE_SCALE,
           (double)telemetry->longitude_e7 / TELEMETRY_COORDINATE_SCALE,
           (unsigned)telemetry->satellites);
    fflush(stdout);
}

void app_main(void)
{
    /* Los logs intermedios permanecen desactivados. */
    esp_log_level_set("*", ESP_LOG_NONE);

    printf("[SERIAL] Inicializando receptor nRF24L01+\n");
    fflush(stdout);

    nrf24l01_t radio = {0};
    nrf24l01_config_t config = NRF24L01_RX_DEFAULT_CONFIG();

    const char *data_rate =
        (config.air_data_rate == NRF24_DATA_RATE_250_KBPS) ? "250 kbps" :
        (config.air_data_rate == NRF24_DATA_RATE_2_MBPS) ? "2 Mbps" :
                                                           "1 Mbps";
    printf("[SERIAL] SPI host=%d MOSI=%d MISO=%d SCLK=%d CSN=%d CE=%d IRQ=%d (-1=NC)\n",
           (int)config.spi_host,
           (int)config.mosi_pin,
           (int)config.miso_pin,
           (int)config.sclk_pin,
           (int)config.csn_pin,
           (int)config.ce_pin,
           (int)config.irq_pin);
    printf("[SERIAL] Radio canal=%u (%u MHz) velocidad=%s payload=%u bytes\n",
           config.channel,
           2400U + config.channel,
           data_rate,
           config.payload_size);
    printf("[SERIAL] Direccion RX=%02X:%02X:%02X:%02X:%02X\n",
           s_radio_address[0], s_radio_address[1], s_radio_address[2],
           s_radio_address[3], s_radio_address[4]);
    fflush(stdout);

    ESP_ERROR_CHECK(nrf24l01_init(&radio, &config));
    printf("[SERIAL] Radio inicializada\n");

    ESP_ERROR_CHECK(nrf24l01_open_rx_pipe(
        &radio, NRF24_PIPE_0, s_radio_address));
    printf("[SERIAL] Pipe RX abierto\n");

    bool connected = false;
    ESP_ERROR_CHECK(nrf24l01_is_connected(&radio, &connected));
    if (!connected) {
        // ESP_LOGE(TAG, "El nRF24L01+ no responde por SPI");
        ESP_ERROR_CHECK(ESP_ERR_NOT_FOUND);
    }

    /*
     * El primer ACK payload debe cargarse antes de recibir telemetria.
     * Luego se carga un comando nuevo por cada paquete recibido, cuando el
     * payload anterior ya salio del FIFO de TX.
     */
    uint16_t next_command_id = 1;
    ESP_ERROR_CHECK(queue_ping_command(&radio, next_command_id++));
    ESP_ERROR_CHECK(nrf24l01_start_listening(&radio));

    /* Permitir solo el diagnostico inicial y volver a CSV durante recepcion. */
    esp_log_level_set("nrf24l01", ESP_LOG_INFO);
    ESP_ERROR_CHECK(nrf24l01_print_diagnostics(&radio));
    esp_log_level_set("nrf24l01", ESP_LOG_NONE);

    printf("[SERIAL] CSV: pitch,roll,yaw,alt_rel_m,temp_c,pres_pa,ax,ay,az,lat,lon,sats\n");
    printf("[SERIAL] Receptor listo\n");
    fflush(stdout);

    while (true) {
        bool available = false;
        nrf24_pipe_t pipe = NRF24_PIPE_0;
        esp_err_t err = nrf24l01_data_available(
            &radio, &available, &pipe);

        if (err != ESP_OK) {
            // ESP_LOGW(TAG, "No se pudo consultar RX: %s",
            //          esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(RX_POLL_PERIOD_MS));
            continue;
        }

        if (!available) {
            vTaskDelay(pdMS_TO_TICKS(RX_POLL_PERIOD_MS));
            continue;
        }

        telemetry_data_t telemetry = {0};
        size_t payload_length = 0;
        nrf24_pipe_t received_pipe = NRF24_PIPE_0;
        err = nrf24l01_receive(&radio,
                               &telemetry,
                               sizeof(telemetry),
                               &payload_length,
                               &received_pipe);
        if (err != ESP_OK) {
            // ESP_LOGW(TAG, "No se pudo leer la telemetria: %s",
            //          esp_err_to_name(err));
            continue;
        }

        if (!telemetry_is_valid(&telemetry, payload_length)) {
            // ESP_LOGW(TAG, "Payload invalido: %u bytes en pipe %u",
            //          (unsigned)payload_length, received_pipe);
        } else {
            send_telemetry_over_serial(&telemetry);
        }

        err = queue_ping_command(&radio, next_command_id++);
        if (err != ESP_OK) {
            // ESP_LOGW(TAG, "No se pudo preparar el ACK payload: %s",
            //          esp_err_to_name(err));
        }
    }
}
