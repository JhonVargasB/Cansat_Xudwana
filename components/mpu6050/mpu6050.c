#include <stdio.h>
#include "mpu6050.h"
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

static const char *TAG = "mpu6050";

static bool mpu6050_config_is_valid(const mpu6050_config_t *config)
{
    return (config != NULL)
           && ((config->device_address == MPU6050_I2C_ADDR_PRIM)
               || (config->device_address == MPU6050_I2C_ADDR_ALT))
           && (config->scl_speed_hz > 0U)
           && (config->scl_speed_hz <= MPU6050_MAX_I2C_CLOCK_HZ)
           && (config->dlpf_cfg <= BWA_5_BWG_5)
           && (config->gfs_sel <= GFS_SEL_2000)
           && (config->afs_sel <= AFS_SEL_16G)
           && (config->clk_sel <= STOP_CLK)
           && (config->clk_sel != UINT8_C(0x06));
}

static void mpu6050_update_scale(mpu6050_t *mpu6050)
{
    static const float accel_lsb_per_g[] = {
        16384.0f, 8192.0f, 4096.0f, 2048.0f,
    };
    static const float gyro_lsb_per_dps[] = {
        131.0f, 65.5f, 32.8f, 16.4f,
    };

    mpu6050->scale.accel_scale =
        accel_lsb_per_g[mpu6050->config.afs_sel];
    mpu6050->scale.gyro_scale =
        gyro_lsb_per_dps[mpu6050->config.gfs_sel];
}

static esp_err_t mpu6050_device_create(mpu6050_t *mpu6050)
{
    if (mpu6050 == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    mpu6050->dev_cfg = (i2c_device_config_t){
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = mpu6050->config.device_address,
        .scl_speed_hz = mpu6050->config.scl_speed_hz,
        .scl_wait_us = mpu6050->config.scl_wait_us,
    };

    // ESP_LOGI(TAG, "device_create for mpu6050 sensors on ADDR %X", mpu6050->dev_cfg.device_address);

    esp_err_t err = i2c_master_bus_add_device(mpu6050->bus_handle, &mpu6050->dev_cfg, &mpu6050->i2c_dev);
    if (err == ESP_OK)
    {
        // ESP_LOGI(TAG, "device_create success on 0x%x", mpu6050->dev_cfg.device_address);
        return err;
    }
    else
    {
        ESP_LOGE(TAG, "device_create error on 0x%x", mpu6050->dev_cfg.device_address);
        return err;
    }
}

static esp_err_t mpu6050_read(mpu6050_t *mpu6050, uint8_t addr, uint8_t *dout, size_t size)
{
    return i2c_master_transmit_receive(mpu6050->i2c_dev, &addr, sizeof(addr), dout, size, -1);
}

static esp_err_t mpu6050_write(mpu6050_t *mpu6050, uint8_t addr, const uint8_t *din, size_t size)
{
    esp_err_t err;

    for (uint8_t i = 0; i < size; i++)
    {
        uint8_t dat[2] = {(addr + i), din[i]};
        if ((err = i2c_master_transmit(mpu6050->i2c_dev, dat, 2, -1)) != ESP_OK)
            return err;
    }
    return ESP_OK;
}

static esp_err_t mpu6050_set_settings(mpu6050_t *mpu6050,
                                      const mpu6050_config_t *config)
{
    uint8_t dout;
    uint8_t dwrite;

    // Configurar PWR_MGMT_1
    esp_err_t err = mpu6050_read(mpu6050, PWR_MGMT_1, &dout, sizeof(dout));
    if (err != ESP_OK)
        return err;
    dwrite = (uint8_t)((dout & UINT8_C(0x08)) | config->clk_sel);
    err = mpu6050_write(mpu6050, PWR_MGMT_1, &dwrite, sizeof(dwrite));
    if (err != ESP_OK)
        return err;

    vTaskDelay(pdMS_TO_TICKS(100));
    // Configurar GYRO_CONFIG    [XG_ST  YG_ST  ZG_ST  FS_SEL[1:0]  -   -   -]
    err = mpu6050_read(mpu6050, GYRO_CONFIG, &dout, sizeof(dout));
    if (err != ESP_OK)
        return err;
    dwrite = (uint8_t)((dout & 0x07) | (config->gfs_sel << 3));
    err = mpu6050_write(mpu6050, GYRO_CONFIG, &dwrite, sizeof(dwrite));
    if (err != ESP_OK)
        return err;
    // Configurar ACCEL_CONFIG    [XA_ST  YA_ST  ZA_ST  AFS_SEL[1:0]  -   -   -]
    err = mpu6050_read(mpu6050, ACCEL_CONFIG, &dout, sizeof(dout));
    if (err != ESP_OK)
        return err;
    dwrite = (uint8_t)((dout & 0x07) | (config->afs_sel << 3));
    err = mpu6050_write(mpu6050, ACCEL_CONFIG, &dwrite, sizeof(dwrite));
    if (err != ESP_OK)
        return err;
    // Configurar SMPRT_DIV[0 - 7]
    dwrite = config->smpltr_div;
    err = mpu6050_write(mpu6050, SMPRT_DIV, &dwrite, sizeof(dwrite));
    if (err != ESP_OK)
        return err;
    // Configurar CONFIG[ -   -  EXT_SYNC_SET[2:0]   DPLF_CFG[2:0]]
    err = mpu6050_read(mpu6050, CONFIG, &dout, sizeof(dout));
    if (err != ESP_OK)
        return err;
    dwrite = (uint8_t)((dout & 0xF8) | config->dlpf_cfg);
    err = mpu6050_write(mpu6050, CONFIG, &dwrite, sizeof(dwrite));
    if (err != ESP_OK)
        return err;
    return ESP_OK;
}

esp_err_t mpu6050_init(mpu6050_t *mpu6050)
{
    if ((mpu6050 == NULL) || (mpu6050->bus_handle == NULL))
        return ESP_ERR_INVALID_ARG;

    if (mpu6050->initialized)
        return ESP_ERR_INVALID_STATE;

    if (!mpu6050_config_is_valid(&mpu6050->config))
        return ESP_ERR_INVALID_ARG;

    esp_err_t err = mpu6050_device_create(mpu6050);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "error crando el dispositivpo");
        return err;
    }
    uint8_t dout;
    err = mpu6050_read(mpu6050, MPU6050_WHO_AM_I, &dout, sizeof(dout));
    if (err != ESP_OK)
        goto fail;

    if ((dout & UINT8_C(0x7E)) != MPU6050_I2C_ADDR_PRIM)
    {
        ESP_LOGE(TAG, "WHO_AM_I inesperado: 0x%02X", dout);
        err = ESP_ERR_NOT_FOUND;
        goto fail;
    }

    err = mpu6050_set_settings(mpu6050, &mpu6050->config);
    if (err != ESP_OK)
        goto fail;

    memset(&mpu6050->calib, 0, sizeof(mpu6050->calib));
    memset(&mpu6050->data, 0, sizeof(mpu6050->data));
    mpu6050_update_scale(mpu6050);
    mpu6050->initialized = true;

    ESP_LOGI(TAG,
             "MPU6050 listo: addr=0x%02X I2C=%lu Hz AFS=%u GFS=%u",
             mpu6050->config.device_address,
             (unsigned long)mpu6050->config.scl_speed_hz,
             (unsigned)mpu6050->config.afs_sel,
             (unsigned)mpu6050->config.gfs_sel);

    return ESP_OK;

fail:
    i2c_master_bus_rm_device(mpu6050->i2c_dev);
    mpu6050->i2c_dev = NULL;
    return err;
}

esp_err_t mpu6050_deinit(mpu6050_t *mpu6050)
{
    if (mpu6050 == NULL)
        return ESP_ERR_INVALID_ARG;

    if (!mpu6050->initialized || (mpu6050->i2c_dev == NULL))
        return ESP_ERR_INVALID_STATE;

    esp_err_t err = i2c_master_bus_rm_device(mpu6050->i2c_dev);
    if (err == ESP_OK)
    {
        mpu6050->i2c_dev = NULL;
        mpu6050->initialized = false;
    }

    return err;
}

esp_err_t selftest_xg(mpu6050_t *mpu6050)
{
    float promedio_td = 0;
    float promedio_te = 0;
    float FT = 0;
    esp_err_t err;
    uint8_t dout;
    uint8_t douthl[2] = {0};
    int16_t dxout = 0;
    uint8_t dwrite = 0;
    uint8_t XG_TEST = 0;
    for (uint8_t i = 1; i <= 100; i++)
    {
        err = mpu6050_read(mpu6050, GYRO_XOUT_H, douthl, sizeof(douthl));

        dxout = (int16_t)(douthl[0] << 8 | douthl[1]);
        promedio_td = promedio_td + (float)(dxout) / 100;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Configurar GYRO_CONFIG    [XG_ST  YG_ST  ZG_ST  FS_SEL[1:0]  -   -   -]
    err = mpu6050_read(mpu6050, GYRO_CONFIG, &dout, sizeof(dout));
    dwrite = (uint8_t)((dout & 0x1F) | (1 << 7));
    err = mpu6050_write(mpu6050, GYRO_CONFIG, &dwrite, sizeof(dwrite));

    vTaskDelay(pdMS_TO_TICKS(500));
    for (uint8_t i = 1; i <= 100; i++)
    {
        err = mpu6050_read(mpu6050, GYRO_XOUT_H, douthl, sizeof(douthl));

        dxout = (int16_t)(douthl[0] << 8 | douthl[1]);
        promedio_te = promedio_te + (float)(dxout) / 100;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    err = mpu6050_read(mpu6050, SELF_TEST_X, &dout, sizeof(dout));
    XG_TEST = (dout & 0x1f);
    ESP_LOGI(TAG, "xg_test_ Oobtendio: %d ", XG_TEST);
    float STR = promedio_te - promedio_td;
    if (XG_TEST == 0)
    {
        FT = 0;
    }
    else
    {
        FT = 25 * 131 * powf(1.046f, ((float)(XG_TEST)-1));
    }

    float C = ((STR - FT) / FT);
    ESP_LOGI(TAG, "valor obtendio: %f", C);
    ESP_LOGI(TAG, "promedio normal: %f", promedio_td);
    ESP_LOGI(TAG, "promedio self-test: %f", promedio_te);
    ESP_LOGI(TAG, "STR: %f", STR);
    ESP_LOGI(TAG, "FT: %f", FT);
    ESP_LOGI(TAG, "C porcentaje: %.2f %%", C * 100.0f);
    err = mpu6050_read(mpu6050, GYRO_CONFIG, &dout, sizeof(dout));
    dwrite = (uint8_t)((dout & 0x1F) | (0 << 7));
    err = mpu6050_write(mpu6050, GYRO_CONFIG, &dwrite, sizeof(dwrite));
    return ESP_OK;
}

esp_err_t selftest_yg(mpu6050_t *mpu6050)
{
    float promedio_td = 0;
    float promedio_te = 0;
    float FT = 0;
    esp_err_t err;
    uint8_t dout;
    uint8_t douthl[2] = {0};
    int16_t dyout = 0;
    uint8_t dwrite = 0;
    uint8_t YG_TEST = 0;
    for (uint8_t i = 1; i <= 100; i++)
    {
        err = mpu6050_read(mpu6050, GYRO_YOUT_H, douthl, sizeof(douthl));
        dyout = (int16_t)(douthl[0] << 8 | douthl[1]);
        promedio_td = promedio_td + (float)(dyout) / 100;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Configurar GYRO_CONFIG    [XG_ST  YG_ST  ZG_ST  FS_SEL[1:0]  -   -   -]
    err = mpu6050_read(mpu6050, GYRO_CONFIG, &dout, sizeof(dout));
    dwrite = (uint8_t)((dout & 0x1F) | (1 << 6));
    err = mpu6050_write(mpu6050, GYRO_CONFIG, &dwrite, sizeof(dwrite));

    vTaskDelay(pdMS_TO_TICKS(500));
    for (uint8_t i = 1; i <= 100; i++)
    {
        err = mpu6050_read(mpu6050, GYRO_YOUT_H, douthl, sizeof(douthl));
        dyout = (int16_t)(douthl[0] << 8 | douthl[1]);
        promedio_te = promedio_te + (float)(dyout) / 100;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    err = mpu6050_read(mpu6050, SELF_TEST_Y, &dout, sizeof(dout));
    YG_TEST = (dout & 0x1f);
    // ESP_LOGI(TAG, "yg_test_ Oobtendio: %d ", YG_TEST);
    float STR = promedio_te - promedio_td;
    if (YG_TEST == 0)
    {
        FT = 0;
    }
    else
    {
        FT = -25 * 131 * powf(1.046f, ((float)(YG_TEST)-1));
    }

    float C = ((STR - FT) / FT);
    ESP_LOGI(TAG, "valor obtendio yg: %f", C);
    ESP_LOGI(TAG, "promedio normal yg: %f", promedio_td);
    ESP_LOGI(TAG, "promedio self-test yg: %f", promedio_te);
    ESP_LOGI(TAG, "STR yg: %f", STR);
    ESP_LOGI(TAG, "FT yg: %f", FT);
    ESP_LOGI(TAG, "C porcentaje yg: %.2f %%", C * 100.0f);

    err = mpu6050_read(mpu6050, GYRO_CONFIG, &dout, sizeof(dout));
    dwrite = (uint8_t)((dout & 0x1F) | (0 << 6));
    err = mpu6050_write(mpu6050, GYRO_CONFIG, &dwrite, sizeof(dwrite));
    return ESP_OK;
}

esp_err_t selftest_zg(mpu6050_t *mpu6050)
{
    float promedio_td = 0;
    float promedio_te = 0;
    float FT = 0;
    esp_err_t err;
    uint8_t dout;
    uint8_t douthl[2] = {0};
    int16_t dzout = 0;
    uint8_t dwrite = 0;
    uint8_t ZG_TEST = 0;
    for (uint8_t i = 1; i <= 100; i++)
    {
        err = mpu6050_read(mpu6050, GYRO_ZOUT_H, douthl, sizeof(douthl));

        dzout = (int16_t)(douthl[0] << 8 | douthl[1]);
        promedio_td = promedio_td + (float)(dzout) / 100;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Configurar GYRO_CONFIG    [XG_ST  YG_ST  ZG_ST  FS_SEL[1:0]  -   -   -]
    err = mpu6050_read(mpu6050, GYRO_CONFIG, &dout, sizeof(dout));
    dwrite = (uint8_t)((dout & 0x1F) | (1 << 5));
    err = mpu6050_write(mpu6050, GYRO_CONFIG, &dwrite, sizeof(dwrite));

    vTaskDelay(pdMS_TO_TICKS(500));
    for (uint8_t i = 1; i <= 100; i++)
    {
        err = mpu6050_read(mpu6050, GYRO_ZOUT_H, douthl, sizeof(douthl));

        dzout = (int16_t)(douthl[0] << 8 | douthl[1]);
        promedio_te = promedio_te + (float)(dzout) / 100;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    err = mpu6050_read(mpu6050, SELF_TEST_Z, &dout, sizeof(dout));
    ZG_TEST = (dout & 0x1f);
    // ESP_LOGI(TAG, "zg_test_ Oobtendio: %d ", ZG_TEST);
    float STR = promedio_te - promedio_td;
    if (ZG_TEST == 0)
    {
        FT = 0;
    }
    else
    {
        FT = 25 * 131 * powf(1.046f, ((float)(ZG_TEST)-1));
    }

    float C = ((STR - FT) / FT);
    ESP_LOGI(TAG, "valor obtendio gz: %f", C);
    ESP_LOGI(TAG, "promedio normal gz: %f", promedio_td);
    ESP_LOGI(TAG, "promedio self-test gz: %f", promedio_te);
    ESP_LOGI(TAG, "STR gz: %f", STR);
    ESP_LOGI(TAG, "FT gz: %f", FT);
    ESP_LOGI(TAG, "C porcentaje gz: %.2f %%", C * 100.0f);
    err = mpu6050_read(mpu6050, GYRO_CONFIG, &dout, sizeof(dout));
    dwrite = (uint8_t)((dout & 0x1F));
    err = mpu6050_write(mpu6050, GYRO_CONFIG, &dwrite, sizeof(dwrite));
    return ESP_OK;
}

esp_err_t mpu6050_read_accel(mpu6050_t *mpu6050)
{
    if (mpu6050 == NULL)
        return ESP_ERR_INVALID_ARG;

    if (!mpu6050->initialized)
        return ESP_ERR_INVALID_STATE;

    esp_err_t err;

    uint8_t buf[6] = {0};

    int16_t ax_raw = 0;
    int16_t ay_raw = 0;
    int16_t az_raw = 0;

    int32_t ax_raw_c = 0;
    int32_t ay_raw_c = 0;
    int32_t az_raw_c = 0;

    float ax_g = 0.0f;
    float ay_g = 0.0f;
    float az_g = 0.0f;

    float ax_ms2 = 0.0f;
    float ay_ms2 = 0.0f;
    float az_ms2 = 0.0f;

    /* Leer ACCEL_XOUT_H hasta ACCEL_ZOUT_L */
    err = mpu6050_read(mpu6050, ACCEL_XOUT_H, buf, sizeof(buf));

    if (err != ESP_OK)
        return err;

    /* Reconstruir enteros signed de 16 bits */
    ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
    ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
    az_raw = (int16_t)((buf[4] << 8) | buf[5]);

    ax_raw_c = (int32_t)ax_raw - (int32_t)mpu6050->calib.accel_x_offset;
    ay_raw_c = (int32_t)ay_raw - (int32_t)mpu6050->calib.accel_y_offset;
    az_raw_c = (int32_t)az_raw - (int32_t)mpu6050->calib.accel_z_offset;

    /*
        Conversión para AFS_SEL = 0 (±2g)

        Sensibilidad:
        16384 LSB/g
    */

    ax_g = (float)ax_raw_c / mpu6050->scale.accel_scale;
    ay_g = (float)ay_raw_c / mpu6050->scale.accel_scale;
    az_g = (float)az_raw_c / mpu6050->scale.accel_scale;

    /* Convertir a m/s² */
    ax_ms2 = ax_g * MPU6050_GRAVITY_M_S2;
    ay_ms2 = ay_g * MPU6050_GRAVITY_M_S2;
    az_ms2 = az_g * MPU6050_GRAVITY_M_S2;

    ////ESP_LOGI(TAG,  "ACC RAW -> X:%d Y:%d Z:%d", ax_raw, ay_raw, az_raw);

    ////ESP_LOGI(TAG,"ACC [g] -> X:%.3f Y:%.3f Z:%.3f",ax_g, ay_g,az_g);
    mpu6050->data.ax = ax_ms2;
    mpu6050->data.ay = ay_ms2;
    mpu6050->data.az = az_ms2;
    // printf("A,X:%.3f Y:%.3f Z:%.3f\n",           ax_ms2,           ay_ms2,           az_ms2);

    return ESP_OK;
}

esp_err_t mpu6050_calibrate_accel(mpu6050_t *mpu6050)
{
    if (mpu6050 == NULL)
        return ESP_ERR_INVALID_ARG;

    if (!mpu6050->initialized)
        return ESP_ERR_INVALID_STATE;

    esp_err_t err;

    uint8_t buf[6];

    int32_t ax_sum = 0;
    int32_t ay_sum = 0;
    int32_t az_sum = 0;

    const uint16_t samples = 1000;

    // ESP_LOGI(TAG, "Mantener el sensor QUIETO...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    for (uint16_t i = 0; i < samples; i++)
    {
        err = mpu6050_read(mpu6050,
                           ACCEL_XOUT_H,
                           buf,
                           sizeof(buf));

        if (err != ESP_OK)
            return err;

        int16_t ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
        int16_t ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
        int16_t az_raw = (int16_t)((buf[4] << 8) | buf[5]);

        ax_sum += ax_raw;
        ay_sum += ay_raw;
        az_sum += az_raw;

        esp_rom_delay_us(2000U);
    }

    int16_t ax_avg = ax_sum / samples;
    int16_t ay_avg = ay_sum / samples;
    int16_t az_avg = az_sum / samples;

    /*
        AFS_SEL = ±8g
        Sensibilidad = 4096 LSB/g

        Esperado:
        X = 0g
        Y = 0g
        Z = +1g = 4096
    */

    mpu6050->calib.accel_x_offset = ax_avg;
    mpu6050->calib.accel_y_offset = ay_avg;
    mpu6050->calib.accel_z_offset =
        (int16_t)(az_avg - (int16_t)mpu6050->scale.accel_scale);
    /*
        //ESP_LOGI(TAG,
                 "Offsets acelerometro -> X:%d Y:%d Z:%d",
                 calib->accel_x_offset,
                 calib->accel_y_offset,
                 calib->accel_z_offset);
    */
    return ESP_OK;
}

esp_err_t mpu6050_calibrate_gyro(mpu6050_t *mpu6050)
{
    if (mpu6050 == NULL)
        return ESP_ERR_INVALID_ARG;

    if (!mpu6050->initialized)
        return ESP_ERR_INVALID_STATE;

    esp_err_t err;
    uint8_t buf[6];

    int32_t gx_sum = 0;
    int32_t gy_sum = 0;
    int32_t gz_sum = 0;

    const uint16_t samples = 1000;

    // ESP_LOGI(TAG, "Mantener el sensor QUIETO para calibrar gyro...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    for (uint16_t i = 0; i < samples; i++)
    {
        err = mpu6050_read(mpu6050,
                           GYRO_XOUT_H,
                           buf,
                           sizeof(buf));

        if (err != ESP_OK)
            return err;

        int16_t gx_raw = (int16_t)((buf[0] << 8) | buf[1]);
        int16_t gy_raw = (int16_t)((buf[2] << 8) | buf[3]);
        int16_t gz_raw = (int16_t)((buf[4] << 8) | buf[5]);

        gx_sum += gx_raw;
        gy_sum += gy_raw;
        gz_sum += gz_raw;

        esp_rom_delay_us(2000U);
    }

    int16_t gx_avg = gx_sum / samples;
    int16_t gy_avg = gy_sum / samples;
    int16_t gz_avg = gz_sum / samples;

    /*
        GFS_SEL = ±250 °/s
        Sensibilidad = 131 LSB/(°/s)

        Esperado en reposo:
        GX = 0
        GY = 0
        GZ = 0
    */

    mpu6050->calib.gyro_x_offset = gx_avg;
    mpu6050->calib.gyro_y_offset = gy_avg;
    mpu6050->calib.gyro_z_offset = gz_avg;

    ESP_LOGI(TAG,
             "Offsets gyro -> X:%d Y:%d Z:%d",
             mpu6050->calib.gyro_x_offset,
             mpu6050->calib.gyro_y_offset,
             mpu6050->calib.gyro_z_offset);

    return ESP_OK;
}

esp_err_t mpu6050_read_gyro(mpu6050_t *mpu6050)
{
    if (mpu6050 == NULL)
        return ESP_ERR_INVALID_ARG;

    if (!mpu6050->initialized)
        return ESP_ERR_INVALID_STATE;

    esp_err_t err;

    uint8_t buf[6] = {0};

    int16_t gx_raw = 0;
    int16_t gy_raw = 0;
    int16_t gz_raw = 0;

    int32_t gx_raw_c = 0;
    int32_t gy_raw_c = 0;
    int32_t gz_raw_c = 0;

    float gx_dps = 0.0f;
    float gy_dps = 0.0f;
    float gz_dps = 0.0f;

    err = mpu6050_read(mpu6050,
                       GYRO_XOUT_H,
                       buf,
                       sizeof(buf));

    if (err != ESP_OK)
        return err;

    gx_raw = (int16_t)((buf[0] << 8) | buf[1]);
    gy_raw = (int16_t)((buf[2] << 8) | buf[3]);
    gz_raw = (int16_t)((buf[4] << 8) | buf[5]);

    gx_raw_c = (int32_t)gx_raw - (int32_t)mpu6050->calib.gyro_x_offset;
    gy_raw_c = (int32_t)gy_raw - (int32_t)mpu6050->calib.gyro_y_offset;
    gz_raw_c = (int32_t)gz_raw - (int32_t)mpu6050->calib.gyro_z_offset;

    /*
        Conversión para GFS_SEL = 0 (±250 °/s)

        Sensibilidad:
        131 LSB/(°/s)
    */

    gx_dps = (float)gx_raw_c / mpu6050->scale.gyro_scale;
    gy_dps = (float)gy_raw_c / mpu6050->scale.gyro_scale;
    gz_dps = (float)gz_raw_c / mpu6050->scale.gyro_scale;
    /*
        ESP_LOGI(TAG,
                 "GYRO RAW -> X:%d Y:%d Z:%d",
                 gx_raw,
                 gy_raw,
                 gz_raw);

        ESP_LOGI(TAG,
                 "GYRO corrected RAW -> X:%ld Y:%ld Z:%ld",
                 gx_raw_c,
                 gy_raw_c,
                 gz_raw_c);
    */

    mpu6050->data.gx = gx_dps;
    mpu6050->data.gy = gy_dps;
    mpu6050->data.gz = gz_dps;

    return ESP_OK;
}
