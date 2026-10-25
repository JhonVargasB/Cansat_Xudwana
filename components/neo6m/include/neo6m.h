#ifndef NEO6M_H
#define NEO6M_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "neo6m_defs.h"
#include "shared.h"
typedef struct
{
    // Configuracion de Hw
    gpio_num_t rx_pin;
    gpio_num_t tx_pin;
    uint32_t baud_rate;
    uart_port_t uart_num;

    uint32_t rx_buffer_size;
    uint32_t tx_buffer_size;
} neo6m_config_t;

typedef struct
{
    neo6m_config_t config;
    neo6m_data_t data;
} neo6m_t;

#define NEO6M_DEFAULT_CONFIG()       \
    {                                \
        .rx_pin = PIN_GPS_UART_RX,       \
        .tx_pin = PIN_GPS_UART_TX,       \
        .baud_rate = 9600,           \
        .uart_num = UART_GPS, \
        .rx_buffer_size = 1024,      \
        .tx_buffer_size = 1024,      \
    }



esp_err_t neo6m_init(neo6m_t *neo6m);

bool neo6m_has_fix(const neo6m_data_t *neo6m);
esp_err_t neo6m_update(neo6m_t *neo6m, uint32_t timeout_ms);
void neo6m_print_data(neo6m_t *neo6m);

#endif