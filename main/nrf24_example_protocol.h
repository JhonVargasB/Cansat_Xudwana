#ifndef NRF24_EXAMPLE_PROTOCOL_H
#define NRF24_EXAMPLE_PROTOCOL_H

#include <stdint.h>

typedef enum {
    NRF24_COMMAND_NONE = 0,
    NRF24_COMMAND_PING = 1,
    NRF24_COMMAND_SET_TELEMETRY_PERIOD = 2,
} nrf24_command_code_t;

typedef enum {
    NRF24_COMMAND_STATUS_NONE = 0,
    NRF24_COMMAND_STATUS_OK = 1,
    NRF24_COMMAND_STATUS_INVALID = 2,
    NRF24_COMMAND_STATUS_UNSUPPORTED = 3,
} nrf24_command_status_t;

typedef struct __attribute__((packed)) {
    uint16_t command_id;
    uint8_t command;
    uint32_t argument;
} nrf24_command_packet_t;

_Static_assert(sizeof(nrf24_command_packet_t) <= 32,
               "Command packet exceeds the nRF24L01+ payload");

#endif
