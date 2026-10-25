#ifndef LORA_E220_900_H
#define LORA_E220_900_H
#include "esp_err.h"
#include "lorae220_900_defs.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "shared.h"

typedef struct
{
    uint32_t baud_rate;
    uart_port_t uart_port_num;

    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    gpio_num_t m0_pin;
    gpio_num_t m1_pin;
    gpio_num_t aux_pin;

    lora_e220_mode_t mode;

    uint16_t address;
    uint8_t channel;

    lora_uart_baud_t uart_baud;
    lora_uart_parity_t uart_parity;
    lora_air_rate_t air_rate;

    lora_sub_packet_t sub_packet;
    lora_rssi_noise_t rssi_noise;
    lora_tx_power_t tx_power;

    lora_rssi_byte_t rssi_byte;
    lora_transmission_method_t transmission_method;
    lora_lbt_t lbt;
    lora_wor_cycle_t wor_cycle;

    uint16_t crypt_key;

} lora_config_t;


typedef struct
{
    lora_config_t config;

} lora_t;

#define LORA_DEFAULT_CONFIG()                         \
    {                                                 \
        .uart_port_num = UART_LORA,                  \
        .tx_pin = PIN_LORA_UART_TX,                        \
        .rx_pin = PIN_LORA_UART_RX,                        \
        .m0_pin = PIN_LORA_M0,                        \
        .m1_pin = PIN_LORA_M1,                        \
        .aux_pin = PIN_LORA_AUX,                       \
        .baud_rate = 9600,                            \
                                                      \
        .address = TRANSMITTER_LORA_ADDRESS,            \
        .channel = TRANSMITTER_LORA_CHANNEL,                                \
                                                      \
        .uart_baud = LORA_UART_BAUD_9600,             \
        .uart_parity = LORA_UART_PARITY_8N1,          \
        .air_rate = LORA_AIR_RATE_2K4,                \
        .sub_packet = LORA_SUB_PACKET_200,            \
        .rssi_noise = LORA_RSSI_NOISE_DISABLE,        \
        .tx_power = LORA_TX_POWER_22DBM,              \
        .rssi_byte = LORA_RSSI_BYTE_DISABLE,          \
        .transmission_method = LORA_FIXED_MODE,       \
        .lbt = LORA_LBT_DISABLE,                      \
        .wor_cycle = LORA_WOR_2000MS,                 \
        .crypt_key = 0x0000,                          \
    }

esp_err_t lora_e220_900_init(lora_t *lora);

int lora_e220_receive(lora_t *lora, uint8_t *buffer, uint16_t max_len, uint32_t timeout_ms);

esp_err_t lora_e220_send(lora_t *lora, const uint8_t *data, uint16_t length);

esp_err_t lora_e220_send_fixed(
    lora_t *lora,
    uint16_t address,
    uint8_t channel,
    const uint8_t *data,
    uint16_t length);

#endif
