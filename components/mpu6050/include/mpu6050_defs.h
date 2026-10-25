#pragma once
#define MPU6050_I2C_ADDR_PRIM UINT8_C(0x68)
#define MPU6050_I2C_ADDR_ALT UINT8_C(0x69)
#define MPU6050_MAX_I2C_CLOCK_HZ UINT32_C(400000)
#define MPU6050_GRAVITY_M_S2 9.80665f

// Self test registers
#define SELF_TEST_X UINT8_C(0x0D)
#define SELF_TEST_Y UINT8_C(0x0E)
#define SELF_TEST_Z UINT8_C(0x0F)
#define SELF_TEST_A UINT8_C(0x10)

/*Sample rate divider
Sampla rate = GyrOutRate/(1+SMPRT_DIV )
*/
#define SMPRT_DIV UINT8_C(0x19)

// Configuration
#define CONFIG UINT8_C(0x1A)
#define GYRO_CONFIG UINT8_C(0x1B)
#define ACCEL_CONFIG UINT8_C(0x1C)

// FIFO ENABLE
#define FIFO_EN UINT8_C(0x23)

// Master Control
#define I2C_MST_CTRL UINT8_C(0x24)
// Slave0:
#define I2C_SLV0_ADDR UINT8_C(0x25)
#define I2C_SLV0_REG UINT8_C(0x26)
#define I2C_SLV0_CTRL UINT8_C(0x27)
// Slave1:
#define I2C_SLV1_ADDR UINT8_C(0x28)
#define I2C_SLV1_REG UINT8_C(0x29)
#define I2C_SLV1_CTRL UINT8_C(0x2A)
// Slave2:
#define I2C_SLV2_ADDR UINT8_C(0x2B)
#define I2C_SLV2_REG UINT8_C(0x2C)
#define I2C_SLV2_CTRL UINT8_C(0x2D)
// Slave3:
#define I2C_SLV3_ADDR UINT8_C(0x2E)
#define I2C_SLV3_REG UINT8_C(0x2F)
#define I2C_SLV3_CTRL UINT8_C(0x30)
// Slave4:
#define I2C_SLV4_ADDR UINT8_C(0x31)
#define I2C_SLV4_REG UINT8_C(0x32)
#define I2C_SLV4_DO UINT8_C(0x33)
#define I2C_SLV4_CTRL UINT8_C(0x34)
#define I2C_SLV4_DI UINT8_C(0x35)
// i2c master status
#define I2C_MST_STATUS UINT8_C(0x36)
// INT pin config
#define INT_PIN_CONFIG UINT8_C(0x37)
// INT Enable
#define INT_ENABLE UINT8_C(0x38)
// INT Satatus
#define INT_STATUS UINT8_C(0x3A)

// ACC Measurements
#define ACCEL_XOUT_H UINT8_C(0x3B)
#define ACCEL_XOUT_L UINT8_C(0x3C)
#define ACCEL_YOUT_H UINT8_C(0x3D)
#define ACCEL_YOUT_L UINT8_C(0x3E)
#define ACCEL_ZOUT_H UINT8_C(0x3F)
#define ACCEL_ZOUT_L UINT8_C(0x40)

// Temperature Measurements
#define TEMP_OUT_H UINT8_C(0x41)
#define TEMP_OUT_L UINT8_C(0x42)

// Gyroscope Measurements
#define GYRO_XOUT_H UINT8_C(0x43)
#define GYRO_XOUT_L UINT8_C(0x44)
#define GYRO_YOUT_H UINT8_C(0x45)
#define GYRO_YOUT_L UINT8_C(0x46)
#define GYRO_ZOUT_H UINT8_C(0x47)
#define GYRO_ZOUT_L UINT8_C(0x48)

// Extern sensors data 0x49 -> 0x60
//  I wnto copy all that s... we are not using it!!

// Master delay Control
#define I2C_MST_DELAY_CTRL UINT8_C(0x67)

// Signal Path reset
#define SIGNAL_PATH_RESET UINT8_C(0x68)

// User Control
#define USER_CTRL UINT8_C(0x6A)

// Power Management
#define PWR_MGMT_1 UINT8_C(0x6B)
#define PWR_MGMT_2 UINT8_C(0x6C)

// FIFO read Write
#define FIFO_R_W UINT8_C(0x74)

// WHo am I? no one knows the answer
#define MPU6050_WHO_AM_I UINT8_C(0x75)

/*========================================================================================
Definicion de Valores de Configuración
*/
//      .clk_sel
#define INTERNAL_8MHZ_CLK (0x00)
#define PLL_X_GYRO_REF_CLK (0x01)
#define PLL_Y_GYRO_REF_CLK (0x02)
#define PLL_Z_GYRO_REF_CLK (0x03)
#define PLL_EXT_37_768K_CLK (0x04)
#define PLL_EXT_19_2MHZ_CLK (0x05)
#define STOP_CLK (0x07)

//          .gfs_sel
#define GFS_SEL_250 (0x00)
#define GFS_SEL_500 (0X01)
#define GFS_SEL_1000 (0x02)
#define GFS_SEL_2000 (0x03)

//          .afs_sel
#define AFS_SEL_2G (0x00)
#define AFS_SEL_4G (0X01)
#define AFS_SEL_8G (0x02)
#define AFS_SEL_16G (0x03)

//          .dlpf_cfg
#define BWA_260_BWG_256 (0x00)
#define BWA_184_BWG_188 (0x01)
#define BWA_94_BWG_98 (0x02)
#define BWA_44_BWG_42 (0x03)
#define BWA_21_BWG_20 (0x04)
#define BWA_10_BWG_10 (0x05)
#define BWA_5_BWG_5 (0x06)

typedef struct
{
    int16_t accel_x_offset;
    int16_t accel_y_offset;
    int16_t accel_z_offset;

    int16_t gyro_x_offset;
    int16_t gyro_y_offset;
    int16_t gyro_z_offset;

} mpu6050_calib_data;

typedef struct
{
    uint8_t device_address;
    uint32_t scl_speed_hz;
    uint32_t scl_wait_us;

    uint8_t smpltr_div; // divide 1k(DLPF enabled) o 8k(DLPF disabled) entre este valor
    uint8_t dlpf_cfg;
    uint8_t gfs_sel;
    uint8_t afs_sel;
    uint8_t clk_sel;

} mpu6050_config_t;

/* Nombre anterior conservado para no romper codigo existente. */
typedef mpu6050_config_t mpu6050_settings;

typedef struct
{
    float gyro_scale;
    float accel_scale;
} mpu6050_scale;

typedef struct
{
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
} mpu6050_data_t;

