#include <inttypes.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lora_test_protocol.h"
#include "lorae220_900.h"
#include "shared.h"

static const char *TAG = "lora_test_tx";

void app_main(void)
{
    lora_t tx_lora = {
        .config = (lora_config_t)LORA_DEFAULT_CONFIG(),
    };

    tx_lora.config.uart_port_num = UART_LORA;
    tx_lora.config.tx_pin = PIN_LORA_UART_TX;
    tx_lora.config.rx_pin = PIN_LORA_UART_RX;
    tx_lora.config.m0_pin = PIN_LORA_M0;
    tx_lora.config.m1_pin = PIN_LORA_M1;
    tx_lora.config.aux_pin = PIN_LORA_AUX;
    tx_lora.config.address = TRANSMITTER_LORA_ADDRESS;
    tx_lora.config.channel = TRANSMITTER_LORA_CHANNEL;
    tx_lora.config.transmission_method = LORA_FIXED_MODE;

    ESP_LOGI(TAG,
             "TX: direccion=0x%04X destino=0x%04X canal=0x%02X "
             "M0=GPIO%d M1=GPIO%d AUX=GPIO%d",
             (unsigned)tx_lora.config.address,
             (unsigned)RECEIVER_LORA_ADDRESS,
             (unsigned)RECEIVER_LORA_CHANNEL,
             (int)tx_lora.config.m0_pin,
             (int)tx_lora.config.m1_pin,
             (int)tx_lora.config.aux_pin);

    ESP_ERROR_CHECK(lora_e220_900_init(&tx_lora));

    lora_test_telemetry_t telemetry = {
        .sequence = 0U,
        .altitude_cm = 10000U,
        .temperature_centi_c = 2350,
        .battery_mv = 4200U,
        .status = 1U,
    };

    while (true)
    {
        telemetry.sequence++;
        telemetry.altitude_cm =
            (uint16_t)(10000U + (telemetry.sequence % 1000U));
        telemetry.temperature_centi_c =
            (int16_t)(2350 + (int16_t)(telemetry.sequence % 50U));
        telemetry.battery_mv =
            (uint16_t)(4200U - (telemetry.sequence % 200U));

        const esp_err_t result = lora_e220_send_fixed(
            &tx_lora,
            RECEIVER_LORA_ADDRESS,
            RECEIVER_LORA_CHANNEL,
            (const uint8_t *)&telemetry,
            (uint16_t)sizeof(telemetry));

        if (result == ESP_OK)
        {
            ESP_LOGI(TAG,
                     "TX seq=%" PRIu32 " alt=%.2f m temp=%.2f C "
                     "bat=%" PRIu16 " mV status=%" PRIu8,
                     telemetry.sequence,
                     (double)telemetry.altitude_cm / 100.0,
                     (double)telemetry.temperature_centi_c / 100.0,
                     telemetry.battery_mv,
                     telemetry.status);
        }
        else
        {
            ESP_LOGE(TAG, "Error transmitiendo: %s", esp_err_to_name(result));
        }

        vTaskDelay(pdMS_TO_TICKS(LORA_TEST_TX_PERIOD_MS));
    }
}
