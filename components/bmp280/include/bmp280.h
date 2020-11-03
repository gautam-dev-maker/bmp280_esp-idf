#ifndef _BMP280_H
#define _BMP280_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "i2c_utils.h"

#define BMP280_ADDRESS_0 0x76 //when SD0 pin is low
#define BMP280_ADDRESS_1 0x77 //when SD0 pin is high
#define BMP280_CHIP_ID 0x58

#define BMP280_RESET_VALUE 0xB6
#define BMP280_REG_STATUS 0xF3
#define BMP280_REG_ID 0xD0
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_CTRL 0xF4
#define BMP280_REG_RESET 0xE0
#define BMP280_REG_CALIB 0x88

#define BMP280_REGISTER_DIG_T1 0x88
#define BMP280_REGISTER_DIG_T2 0x8A
#define BMP280_REGISTER_DIG_T3 0x8C
#define BMP280_REGISTER_DIG_P1 0x8E
#define BMP280_REGISTER_DIG_P2 0x90
#define BMP280_REGISTER_DIG_P3 0x92
#define BMP280_REGISTER_DIG_P4 0x94
#define BMP280_REGISTER_DIG_P5 0x96
#define BMP280_REGISTER_DIG_P6 0x98
#define BMP280_REGISTER_DIG_P7 0x9A
#define BMP280_REGISTER_DIG_P8 0x9C
#define BMP280_REGISTER_DIG_P9 0x9E
#define BMP280_REGISTER_CHIPID 0xD0
#define BMP280_REGISTER_VERSION 0xD1
#define BMP280_REGISTER_SOFTRESET 0xE0
#define BMP280_REGISTER_CAL26 0xE1 /**< R calibration = 0xE1-0xF0 */
#define BMP280_REGISTER_STATUS 0xF3
#define BMP280_REGISTER_CONTROL 0xF4
#define BMP280_REGISTER_CONFIG 0xF5
#define BMP280_REGISTER_PRESSUREDATA 0xF7
#define BMP280_REGISTER_TEMPDATA 0xFA

typedef struct
{
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t id; // Chip ID
} bmp280_t;

typedef enum
{
    BMP280_MODE_SLEEP = 0,  //!< Sleep mode
    BMP280_MODE_FORCED = 1, //!< Measurement is initiated by user
    BMP280_MODE_NORMAL = 3  //!< Continues measurement
} BMP280_Mode;

typedef enum
{
    BMP280_FILTER_OFF = 0,
    BMP280_FILTER_2 = 1,
    BMP280_FILTER_4 = 2,
    BMP280_FILTER_8 = 3,
    BMP280_FILTER_16 = 4
} BMP280_Filter;

typedef enum
{
    BMP280_SKIPPED = 0,         //!< no measurement
    BMP280_ULTRA_LOW_POWER = 1, //!< oversampling x1
    BMP280_LOW_POWER = 2,       //!< oversampling x2
    BMP280_STANDARD = 3,        //!< oversampling x4
    BMP280_HIGH_RES = 4,        //!< oversampling x8
    BMP280_ULTRA_HIGH_RES = 5   //!< oversampling x16
} BMP280_Oversampling;

typedef enum
{
    BMP280_STANDBY_05 = 0,   //!< stand by time 0.5ms
    BMP280_STANDBY_62 = 1,   //!< stand by time 62.5ms
    BMP280_STANDBY_125 = 2,  //!< stand by time 125ms
    BMP280_STANDBY_250 = 3,  //!< stand by time 250ms
    BMP280_STANDBY_500 = 4,  //!< stand by time 500ms
    BMP280_STANDBY_1000 = 5, //!< stand by time 1s
    BMP280_STANDBY_2000 = 6, //!< stand by time 2s BMP280, 10ms BME280
    BMP280_STANDBY_4000 = 7, //!< stand by time 4s BMP280, 20ms BME280
} BMP280_StandbyTime;

typedef struct
{
    BMP280_Mode mode;
    BMP280_Filter filter;
    BMP280_Oversampling oversampling_pressure;
    BMP280_Oversampling oversampling_temperature;
    BMP280_Oversampling oversampling_humidity;
    BMP280_StandbyTime standby;
} bmp280_params_t;

void bmp280_init_default_params(bmp280_params_t *params);

esp_err_t bmp280_init_id(bmp280_t *dev);

esp_err_t bmp280_reset(void);

esp_err_t bmp280_init_calibration(bmp280_t *dev);

esp_err_t bmp280_read_status(void);

esp_err_t bmp280_init_config(bmp280_params_t *params);

esp_err_t bmp280_init_ctrl(bmp280_params_t *params);

esp_err_t enable_bmp280(bmp280_params_t *params, bmp280_t *dev);

esp_err_t bmp280_read_raw(bmp280_t *dev, int32_t *raw_pressure, int32_t *raw_temperature);

int32_t compensate_temperature(bmp280_t *dev, int32_t raw_temp, int32_t *fine_temp);

uint32_t compensate_pressure(bmp280_t *dev, int32_t adc_pressure, int32_t fine_temp);

esp_err_t read_temp_and_pressure(bmp280_t *dev, float *temperature, float *pressure);

#endif