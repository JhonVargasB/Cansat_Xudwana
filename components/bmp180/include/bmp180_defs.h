#pragma once

#include <stdint.h>

/* Fixed 7-bit I2C address and identification value. */
#define BMP180_I2C_ADDRESS             UINT8_C(0x77)
#define BMP180_CHIP_ID_VALUE           UINT8_C(0x55)
#define BMP180_MAX_I2C_CLOCK_HZ        UINT32_C(3400000)

/* Register map. */
#define BMP180_REG_CALIB_START         UINT8_C(0xAA)
#define BMP180_REG_CHIP_ID             UINT8_C(0xD0)
#define BMP180_REG_SOFT_RESET          UINT8_C(0xE0)
#define BMP180_REG_CONTROL             UINT8_C(0xF4)
#define BMP180_REG_DATA_MSB            UINT8_C(0xF6)
#define BMP180_REG_DATA_LSB            UINT8_C(0xF7)
#define BMP180_REG_DATA_XLSB           UINT8_C(0xF8)

#define BMP180_CALIBRATION_DATA_SIZE   UINT8_C(22)

/* Commands and control-register fields. */
#define BMP180_SOFT_RESET_COMMAND      UINT8_C(0xB6)
#define BMP180_TEMPERATURE_COMMAND     UINT8_C(0x2E)
#define BMP180_PRESSURE_COMMAND        UINT8_C(0x34)
#define BMP180_CONTROL_SCO_MASK        UINT8_C(0x20)
#define BMP180_CONTROL_OSS_POS         UINT8_C(6)

/* Datasheet maximum conversion and startup times, rounded up to milliseconds. */
#define BMP180_STARTUP_TIME_MS         UINT32_C(10)
#define BMP180_TEMPERATURE_TIME_MS     UINT32_C(5)
#define BMP180_PRESSURE_TIME_ULP_MS    UINT32_C(5)
#define BMP180_PRESSURE_TIME_STD_MS    UINT32_C(8)
#define BMP180_PRESSURE_TIME_HR_MS     UINT32_C(14)
#define BMP180_PRESSURE_TIME_UHR_MS    UINT32_C(26)

#define BMP180_DEFAULT_SEA_LEVEL_PRESSURE_PA 101325.0

typedef enum
{
    BMP180_OVERSAMPLING_ULTRA_LOW_POWER = 0,
    BMP180_OVERSAMPLING_STANDARD = 1,
    BMP180_OVERSAMPLING_HIGH_RESOLUTION = 2,
    BMP180_OVERSAMPLING_ULTRA_HIGH_RESOLUTION = 3,
} bmp180_oversampling_t;

typedef struct
{
    int16_t ac1;
    int16_t ac2;
    int16_t ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;
    int16_t b1;
    int16_t b2;
    int16_t mb;
    int16_t mc;
    int16_t md;
} bmp180_calib_data_t;

typedef struct
{
    uint32_t temperature;
    uint32_t pressure;
} bmp180_uncomp_data_t;

typedef struct
{
    double temperature;
    double pressure;
    double altitude;
} bmp180_data_t;

