#include "bmp280.h"

static const char *TAG_BMP = "BMP280";

void bmp280_init_default_params(bmp280_params_t *params)
{
    params->mode = BMP280_MODE_NORMAL;
    params->filter = BMP280_FILTER_OFF;
    params->oversampling_pressure = BMP280_STANDARD;
    params->oversampling_temperature = BMP280_STANDARD;
    params->oversampling_humidity = BMP280_STANDARD;
    params->standby = BMP280_STANDBY_05;
}

esp_err_t bmp280_init_id(bmp280_t *dev)
{
    uint8_t conn_attempt = 0;
    esp_err_t ret = ESP_FAIL;

    while ((conn_attempt++ < 5) && (dev->id != BMP280_CHIP_ID))
    {
        ret = i2c_read_from_slave(BMP280_ADDRESS_0, BMP280_CHIP_ID, 1, &dev->id);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    if (ret != ESP_OK)
        logE(TAG_BMP, "%s", "Sensor not found");

    if (dev->id != BMP280_CHIP_ID)
        logE(TAG_BMP, "%s", "Invalid Chip Id");

    return ret;
}

esp_err_t bmp280_reset(void)
{
    uint8_t value = BMP280_RESET_VALUE;
    esp_err_t ret = i2c_write_to_slave(BMP280_ADDRESS_0, BMP280_REG_RESET, 1, &value);
    if (ret != ESP_OK)
        logE(TAG_BMP, "%s", "Failed to reset sensor!");

    return ret;
}

esp_err_t bmp280_init_calibration(bmp280_t *dev)
{
    esp_err_t ret = ESP_FAIL;

    IS_ESP_OK(i2c_read_int16_little_endian(BMP280_ADDRESS_0, BMP280_REGISTER_DIG_T1, (int16_t *)&dev->dig_T1), EXIT);
    IS_ESP_OK(i2c_read_int16_little_endian(BMP280_ADDRESS_0, BMP280_REGISTER_DIG_T2, &dev->dig_T2), EXIT);
    IS_ESP_OK(i2c_read_int16_little_endian(BMP280_ADDRESS_0, BMP280_REGISTER_DIG_T3, &dev->dig_T3), EXIT);

    IS_ESP_OK(i2c_read_int16_little_endian(BMP280_ADDRESS_0, BMP280_REGISTER_DIG_P1, (int16_t *)&dev->dig_P1), EXIT);
    IS_ESP_OK(i2c_read_int16_little_endian(BMP280_ADDRESS_0, BMP280_REGISTER_DIG_P2, &dev->dig_P2), EXIT);
    IS_ESP_OK(i2c_read_int16_little_endian(BMP280_ADDRESS_0, BMP280_REGISTER_DIG_P3, &dev->dig_P3), EXIT);
    IS_ESP_OK(i2c_read_int16_little_endian(BMP280_ADDRESS_0, BMP280_REGISTER_DIG_P4, &dev->dig_P4), EXIT);
    IS_ESP_OK(i2c_read_int16_little_endian(BMP280_ADDRESS_0, BMP280_REGISTER_DIG_P5, &dev->dig_P5), EXIT);
    IS_ESP_OK(i2c_read_int16_little_endian(BMP280_ADDRESS_0, BMP280_REGISTER_DIG_P6, &dev->dig_P6), EXIT);
    IS_ESP_OK(i2c_read_int16_little_endian(BMP280_ADDRESS_0, BMP280_REGISTER_DIG_P7, &dev->dig_P7), EXIT);
    IS_ESP_OK(i2c_read_int16_little_endian(BMP280_ADDRESS_0, BMP280_REGISTER_DIG_P8, &dev->dig_P8), EXIT);
    IS_ESP_OK(i2c_read_int16_little_endian(BMP280_ADDRESS_0, BMP280_REGISTER_DIG_P9, &dev->dig_P9), EXIT);

    ret = ESP_OK;

EXIT:
    return ret;
}

esp_err_t bmp280_read_status(void)
{
    uint8_t status = 1;
    esp_err_t ret = i2c_read_from_slave(BMP280_ADDRESS_0, BMP280_REG_STATUS, 1, &status);
    if (ret == ESP_OK && (status & 1) == 0)
        logI(TAG_BMP, "%s", "BMP280 Ready!");

    return ret;
}

esp_err_t bmp280_init_config(bmp280_params_t *params)
{
    uint8_t config = (params->standby << 5) | (params->filter << 2);
    esp_err_t ret = i2c_write_to_slave(BMP280_ADDRESS_0, BMP280_REG_CONFIG, 1, &config);
    if (ret == ESP_OK)
        logI(TAG_BMP, "%s", "BMP280 Configured!");

    return ret;
}

esp_err_t bmp280_init_ctrl(bmp280_params_t *params)
{
    uint8_t ctrl = (params->oversampling_temperature << 5) | (params->oversampling_pressure << 2) | (params->mode);
    esp_err_t ret = i2c_write_to_slave(BMP280_ADDRESS_0, BMP280_REG_CTRL, 1, &ctrl);

    if (ret == ESP_OK)
        logI(TAG_BMP, "%s", "BMP280 Data Acquisition parameters set!");

    return ret;
}

esp_err_t enable_bmp280(bmp280_params_t *params, bmp280_t *dev)
{
    esp_err_t ret = ESP_FAIL;
    
    bmp280_init_default_params(params);
    memset(dev, 0, sizeof(bmp280_t));

    IS_ESP_OK(bmp280_init_id(dev), EXIT);
    IS_ESP_OK(bmp280_reset(), EXIT);

    while (ret != ESP_FAIL)
    {
        ret = bmp280_read_status();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    IS_ESP_OK(bmp280_init_calibration(dev), EXIT);
    IS_ESP_OK(bmp280_init_config(params), EXIT);
    IS_ESP_OK(bmp280_init_ctrl(params), EXIT);

    ret = ESP_OK;

    EXIT:
        return ret;
}

esp_err_t bmp280_read_raw(bmp280_t *dev, int32_t *raw_pressure, int32_t *raw_temperature)
{
    esp_err_t ret = ESP_FAIL;

    IS_ESP_OK(i2c_read_int32_big_endian(BMP280_ADDRESS_0, BMP280_REGISTER_PRESSUREDATA, raw_pressure), EXIT);
    IS_ESP_OK(i2c_read_int32_big_endian(BMP280_ADDRESS_0, BMP280_REGISTER_TEMPDATA, raw_temperature), EXIT);
    ret = ESP_OK;

EXIT:
    return ret;
}

int32_t compensate_temperature(bmp280_t *dev, int32_t raw_temp, int32_t *fine_temp)
{
    int32_t var1, var2, t;

    var1 = ((((raw_temp >> 3) - ((int32_t)dev->dig_T1 << 1))) * (int32_t)dev->dig_T2) >> 11;
    var2 = (((((raw_temp >> 4) - (int32_t)dev->dig_T1) * ((raw_temp >> 4) - (int32_t)dev->dig_T1)) >> 12) * (int32_t)dev->dig_T3) >> 14;

    *fine_temp = var1 + var2;
    t = (*fine_temp * 5 + 128) >> 8;

    return t;
}

uint32_t compensate_pressure(bmp280_t *dev, int32_t adc_pressure, int32_t fine_temp)
{
    int64_t var1, var2, p;

    var1 = (int64_t)fine_temp - 128000;
    var2 = var1 * var1 * (int64_t)dev->dig_P6;
    var2 = var2 + ((var1 * (int64_t)dev->dig_P5) << 17);
    var2 = var2 + (((int64_t)dev->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dev->dig_P3) >> 8) + ((var1 * (int64_t)dev->dig_P2) << 12);
    var1 = (((int64_t)1 << 47) + var1) * ((int64_t)dev->dig_P1) >> 33;

    if (var1 == 0)
        return 0; // avoid exception caused by division by zero

    p = 1048576 - adc_pressure;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)dev->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)dev->dig_P8 * p) >> 19;

    p = ((p + var1 + var2) >> 8) + ((int64_t)dev->dig_P7 << 4);
    return (uint32_t)p;
}

esp_err_t read_temp_and_pressure(bmp280_t *dev, float *temperature, float *pressure)
{
    int32_t raw_temp, raw_pressure, fine_temp;
    esp_err_t ret = bmp280_read_raw(dev, &raw_pressure, &raw_temp);

    *temperature = (float)compensate_temperature(dev, raw_temp, &fine_temp) / 100;
    *pressure = (float)compensate_pressure(dev, raw_pressure, fine_temp) / 256;

    return ret;
}
