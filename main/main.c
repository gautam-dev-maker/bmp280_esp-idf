#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "freertos/task.h"

#define SDA_GPIO 21
#define SCL_GPIO 22
#define BMP280_address 0X76
//0X76

void app_main(){

    //creating configuration structure
    i2c_config_t i2c_config={
        .mode = I2C_MODE_MASTER,
        .sda_io_num=SDA_GPIO,
        .scl_io_num=SCL_GPIO,
        .sda_pullup_en= GPIO_PULLUP_ENABLE,
        .scl_pullup_en=GPIO_PULLUP_ENABLE,
        .master.clk_speed=1000000
    };

    i2c_param_config(I2C_NUM_0,&i2c_config);
    i2c_driver_install(I2C_NUM_0,I2C_MODE_MASTER,0,0,0);
while(1){
    uint8_t raw[4];
    //creating command handler 
    i2c_cmd_handle_t cmd_handle=i2c_cmd_link_create();
    //starting the i2c bus command handle 
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle,( BMP280_address << 1 ) | I2C_MASTER_READ,true);
    i2c_master_read(cmd_handle,(uint8_t *)&raw,4,I2C_MASTER_ACK);
    i2c_master_stop(cmd_handle);
    //Now we have written required instructions in my command handle
    //Now let's command handle begin
    i2c_master_cmd_begin(I2C_NUM_0,cmd_handle,1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
    
    int16_t raw_pressure=raw[0]<<8|raw[1];
    int16_t raw_temperature=raw[2]<<8|raw[3];

    printf("The pressure is %d\nThe temperatur is %d\n",raw_pressure,raw_temperature);



}}