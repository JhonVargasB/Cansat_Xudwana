#ifndef NRF24_COMMAND_PROTOCOL_H
#define NRF24_COMMAND_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define NRF24_COMMAND_MAGIC UINT16_C(0x4D43)
#define NRF24_COMMAND_ACK_MAGIC UINT16_C(0x414B)

typedef enum {
    NRF24_COMMAND_REC_ON = 1,
    NRF24_COMMAND_REC_OFF = 2,
    NRF24_COMMAND_CALIBRATE = 3,
} nrf24_command_code_t;

typedef enum {
    NRF24_COMMAND_RESULT_ACCEPTED = 0,
    NRF24_COMMAND_RESULT_DUPLICATE = 1,
    NRF24_COMMAND_RESULT_REJECTED_STATE = 2,
    NRF24_COMMAND_RESULT_INVALID_COMMAND = 3,
    NRF24_COMMAND_RESULT_ACTUATOR_ERROR = 4,
} nrf24_command_result_t;

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint16_t sequence;
    uint8_t command;
    uint16_t crc16;
} nrf24_command_packet_t;

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint16_t sequence;
    uint8_t command;
    uint8_t result;
    uint8_t state;
    uint16_t crc16;
} nrf24_command_ack_packet_t;

_Static_assert(sizeof(nrf24_command_packet_t) == 7U, "Comando debe ocupar 7 bytes");
_Static_assert(sizeof(nrf24_command_ack_packet_t) == 9U, "Respuesta debe ocupar 9 bytes");

static inline uint16_t nrf24_command_crc16(const void *data, size_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint16_t crc = UINT16_C(0xFFFF);
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)bytes[i] << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & UINT16_C(0x8000)) != 0
                ? (uint16_t)((crc << 1) ^ UINT16_C(0x1021))
                : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

#endif
