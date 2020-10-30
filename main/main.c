// #include <stdio.h>
// #include <string.h>
// #include <stdint.h>
// #include "freertos/FreeRTOS.h"
// #include "driver/i2c.h"
// #include "freertos/task.h"

// #define SDA_GPIO 21
// #define SCL_GPIO 22
// #define BMP280_address 0X76
// //0X76

// void app_main(){

//     //creating configuration structure
    // i2c_config_t i2c_config={
    //     .mode = I2C_MODE_MASTER,
    //     .sda_io_num=SDA_GPIO,
    //     .scl_io_num=SCL_GPIO,
    //     .sda_pullup_en= GPIO_PULLUP_ENABLE,
    //     .scl_pullup_en=GPIO_PULLUP_ENABLE,
    //     .master.clk_speed=1000000
    // };

//     i2c_param_config(I2C_NUM_0,&i2c_config);
//     i2c_driver_install(I2C_NUM_0,I2C_MODE_MASTER,0,0,0);

//     uint8_t raw[4];
    
//     i2c_cmd_handle_t cmd_handle=i2c_cmd_link_create();
    
//     i2c_master_start(cmd_handle);
//     i2c_master_write_byte(cmd_handle,( BMP280_address << 1 ) | I2C_MASTER_READ,true);
//     i2c_master_read(cmd_handle,(uint8_t *)&raw,4,I2C_MASTER_ACK);
//     i2c_master_stop(cmd_handle);
    
//     i2c_master_cmd_begin(I2C_NUM_0,cmd_handle,1000/portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd_handle);
    
//     int16_t raw_pressure=raw[0]<<8|raw[1];
//     int16_t raw_temperature=raw[2]<<8|raw[3];

//     printf("The pressure is %d\nThe temperatur is %d\n",raw_pressure,raw_temperature);



// }

#include <stdio.h>
#include "logger.h"
#include "bmp280.h"

#define SDA_GPIO 21
#define SCL_GPIO 22

#define SLV_RX_BUF_LEN 0
#define SLV_TX_BUF_LEN 0
#define INTR_ALLOC_FLAGS 0

#define MASTER_CLK_SPEED 1000000




void app_main(){
    i2c_driver_install(I2C_NUM_0,I2C_MODE_MASTER,SLV_RX_BUF_LEN,SLV_TX_BUF_LEN,INTR_ALLOC_FLAGS);
    i2c_config_t i2c_config={
        .mode = I2C_MODE_MASTER,
        .sda_io_num=SDA_GPIO,
        .scl_io_num=SCL_GPIO,
        .sda_pullup_en= GPIO_PULLUP_ENABLE,
        .scl_pullup_en=GPIO_PULLUP_ENABLE,
        .master.clk_speed= MASTER_CLK_SPEED
    };

    bmp280_params_t params;

    bmp280_init_default_params(&params);

    bmp280_t dev;
    memset(&dev, 0, sizeof(bmp280_t));

    i2c_port_t i2c_num=I2C_NUM_0;
    
    bmp280_init_id(&dev,i2c_config);

    bmp280_resetting(i2c_config);

    while (1)
    {
        uint8_t status;
        if (!read_data(1,&status,i2c_config,BMP280_REG_STATUS) && (status & 1) == 0)
            break;
    }
    
    bmp280_init_calibration(i2c_num,&dev,i2c_config);

    bmp280_init_config(&params,i2c_config);

    bmp280_init_ctrl(&params,i2c_config);

    float pressure, temperature;

    while (1)
    {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        if (bmp280_read_float(&dev, &temperature, &pressure,i2c_config) != ESP_OK)
        {
            printf("Temperature/pressure reading failed\n");
            continue;
        }

        printf("Pressure: %.2f Pa, Temperature: %.2f C", pressure, temperature);
        printf("\n");
    }



}