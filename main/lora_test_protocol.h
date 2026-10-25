#ifndef LORA_TEST_PROTOCOL_H
#define LORA_TEST_PROTOCOL_H

#include <stdint.h>

#define LORA_TEST_TX_PERIOD_MS UINT32_C(200)

/*
 * Trama minima para comprobar el enlace. Los valores enteros evitan depender
 * de la representacion de float y packed mantiene el mismo tamano en ambos
 * extremos.
 */
typedef struct __attribute__((packed))
{
    uint32_t sequence;
    uint16_t altitude_cm;
    int16_t temperature_centi_c;
    uint16_t battery_mv;
    uint8_t status;
} lora_test_telemetry_t;

_Static_assert(sizeof(lora_test_telemetry_t) == 11U,
               "lora_test_telemetry_t debe medir 11 bytes");

#endif
