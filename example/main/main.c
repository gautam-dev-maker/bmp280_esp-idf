#include <stdio.h>
#include "bmp280.h"


void app_main(){

    // initialising the sensor
    if(bmp280_init()==ESP_OK){              //verifying if the initialisation is successful
        float pressure, temperature;
        while (1)
        {
            vTaskDelay(100/ portTICK_PERIOD_MS);
            if (bmp280_read_float( &temperature, &pressure) != ESP_OK)
            {
                printf("Temperature/pressure reading failed\n");
                continue;
            }

            printf("Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
            printf("\n");
        }
    }
}