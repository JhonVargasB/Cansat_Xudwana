#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    double latitude;
    double longitude;
    float altitude_m;
    float speed_kmh;

    uint8_t fix_quality; // GGA: 0 sin fix, 1 GPS fix
    uint8_t fix_type;    // GSA: 1 no fix, 2 2D, 3 3D

    uint8_t satellites_used;    // GGA
    uint8_t satellites_visible; // GSV

    float pdop;
    float hdop;
    float vdop;

    char utc_time[16];
    char date[16];

    bool valid;
} neo6m_data_t;

