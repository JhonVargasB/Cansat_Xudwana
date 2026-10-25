#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"

#include "lora_test_protocol.h"
#include "lorae220_900.h"
#include "shared.h"

static const char *TAG = "lora_test_rx";

static void print_telemetry(const lora_test_telemetry_t *telemetry)
{
    printf("LORA_RX,seq=%" PRIu32 ",altitude=%.2f m,temperature=%.2f C,"
           "battery=%" PRIu16 " mV,status=%" PRIu8 "\r\n",
           telemetry->sequence,
           (double)telemetry->altitude_cm / 100.0,
           (double)telemetry->temperature_centi_c / 100.0,
           telemetry->battery_mv,
           telemetry->status);
    fflush(stdout);
}

void app_main(void)
{
    lora_t rx_lora = {
        .config = (lora_config_t)LORA_DEFAULT_CONFIG(),
    };

    rx_lora.config.uart_port_num = UART_LORA;
    rx_lora.config.tx_pin = PIN_LORA_UART_TX;
    rx_lora.config.rx_pin = PIN_LORA_UART_RX;
    rx_lora.config.m0_pin = PIN_LORA_M0;
    rx_lora.config.m1_pin = PIN_LORA_M1;
    rx_lora.config.aux_pin = PIN_LORA_AUX;
    rx_lora.config.address = RECEIVER_LORA_ADDRESS;
    rx_lora.config.channel = RECEIVER_LORA_CHANNEL;
    rx_lora.config.transmission_method = LORA_FIXED_MODE;

    ESP_LOGI(TAG,
             "RX: direccion=0x%04X canal=0x%02X "
             "M0=GPIO%d M1=GPIO%d AUX=GPIO%d",
             (unsigned)rx_lora.config.address,
             (unsigned)rx_lora.config.channel,
             (int)rx_lora.config.m0_pin,
             (int)rx_lora.config.m1_pin,
             (int)rx_lora.config.aux_pin);

    ESP_ERROR_CHECK(lora_e220_900_init(&rx_lora));

    uint8_t frame[sizeof(lora_test_telemetry_t)] = {0};
    size_t received_bytes = 0U;
    bool sequence_valid = false;
    uint32_t previous_sequence = 0U;

    while (true)
    {
        const int received = lora_e220_receive(
            &rx_lora,
            &frame[received_bytes],
            (uint16_t)(sizeof(frame) - received_bytes),
            RECEIVER_READ_TIMEOUT_MS);

        if (received < 0)
        {
            ESP_LOGE(TAG, "Error leyendo la UART del E220");
            received_bytes = 0U;
            continue;
        }

        if (received == 0)
            continue;

        received_bytes += (size_t)received;
        if (received_bytes != sizeof(frame))
            continue;

        lora_test_telemetry_t telemetry;
        memcpy(&telemetry, frame, sizeof(telemetry));
        received_bytes = 0U;

        if (sequence_valid && telemetry.sequence != (previous_sequence + 1U))
        {
            ESP_LOGW(TAG,
                     "Salto de secuencia: anterior=%" PRIu32
                     " actual=%" PRIu32,
                     previous_sequence,
                     telemetry.sequence);
        }

        previous_sequence = telemetry.sequence;
        sequence_valid = true;
        print_telemetry(&telemetry);
    }
}
