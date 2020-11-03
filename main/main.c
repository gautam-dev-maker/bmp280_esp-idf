#include "stdio.h"
#include "stdlib.h"

#include "sdkconfig.h"
#include "esp_attr.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "bmp280.h"

static const char *TAG = "main";

void app_main()
{
    logI(TAG, "%s", "Hello from ESP32!");

    if (i2c_master_init() == ESP_OK)
    {
        bmp280_params_t *params = malloc(sizeof(bmp280_params_t));
        bmp280_t *dev = malloc(sizeof(bmp280_t));

        float *pressure = malloc(sizeof(float));
        float *temperature = malloc(sizeof(float));

        if (enable_bmp280(params, dev) == ESP_OK)
        {
            logI(TAG, "%s", "BMP280 Initialization Success!");

            while (1)
            {
                if (read_temp_and_pressure(dev, temperature, pressure) == ESP_OK)
                    logD(TAG, "Pressure: %0.2f | Temperature: %0.2f", pressure, temperature);
                else
                    logE(TAG, "%s", "BMP280 Read Failure!");
            }
        }
        else
            logE(TAG, "%s", "BMP280 Initialization Failure!");

        free(params);
        free(dev);

        free(pressure);
        free(temperature);
    }
}