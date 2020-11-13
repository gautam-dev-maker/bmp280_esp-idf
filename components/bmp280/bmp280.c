#include "bmp280.h"

// declaring configuration required configuration for I2C communication
i2c_config_t i2c_config={
    .mode = I2C_MODE_MASTER,
    .sda_io_num=SDA_GPIO,
    .scl_io_num=SCL_GPIO,
    .sda_pullup_en= GPIO_PULLUP_ENABLE,
    .scl_pullup_en=GPIO_PULLUP_ENABLE,
    .master.clk_speed= MASTER_CLK_SPEED
};

esp_err_t write_8bits_data_to_slave(i2c_config_t i2c_config,uint8_t *value,size_t size,uint8_t reg){
    i2c_param_config(I2C_PORT,&i2c_config);

    // creating a command handle
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Sending a Start sequence
    i2c_master_start(cmd);

    // Sending the I2C address of the slave with the R/W bit low
    i2c_master_write_byte(cmd, BMP280_ADDRESS_0 << 1|I2C_MASTER_WRITE, true);

    // Sending the internal register number you want to write to
    i2c_master_write_byte(cmd,reg,true);

    // Sending the data byte
    i2c_master_write(cmd,value,size,I2C_MASTER_ACK);

    // stopping the command handle
    i2c_master_stop(cmd);

    esp_err_t res = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return res;
}

esp_err_t read_8bit_data_from_slave(uint8_t size,uint8_t* data,i2c_config_t i2c_config,uint8_t reg){
    i2c_param_config(I2C_PORT,&i2c_config);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    //sending internal address of the bearing register
    if (reg !=0x00){

        // sending a start sequence
        i2c_master_start(cmd);
        // sending the i2c address with R/W bit low
        i2c_master_write_byte(cmd, BMP280_ADDRESS_0<< 1, true);
        // sending the address of the register
        i2c_master_write(cmd, &reg, 1, true);

    }

    //sending a start sequence again
    i2c_master_start(cmd);
    //sending i2c address with R/W bit high
    i2c_master_write_byte(cmd, (BMP280_ADDRESS_0 << 1) | I2C_MASTER_READ, true);
    //reading the data byte 
    i2c_master_read(cmd, data, size, I2C_MASTER_LAST_NACK);
    //sending the stop sequence
    i2c_master_stop(cmd);

    esp_err_t res = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return res;
}

esp_err_t read_16bit_data_from_slave(uint16_t *r,i2c_config_t i2c_config,uint8_t reg){
    uint8_t d[] = { 0, 0 };
    i2c_param_config(I2C_PORT,&i2c_config);
    esp_err_t ret=read_8bit_data_from_slave(2,d,i2c_config,reg);
    *r = d[0] | (d[1] << 8);

    return ret;
}

esp_err_t bmp280_init_default_params(bmp280_params_t *params){

    params->mode = BMP280_MODE_NORMAL;
    params->filter = BMP280_FILTER_OFF;
    params->oversampling_pressure = BMP280_STANDARD;
    params->oversampling_temperature = BMP280_STANDARD;
    params->oversampling_humidity = BMP280_STANDARD;
    params->standby = BMP280_STANDBY_250;
    return ESP_OK;
}

esp_err_t bmp280_init_id(bmp280_t *dev,i2c_config_t i2c_config){
    uint8_t retry=0;
    esp_err_t ret=ESP_FAIL;
    while((retry++<5)&& (dev->id!=BMP280_CHIP_ID)){
        ret=read_8bit_data_from_slave(1,&dev->id,i2c_config,BMP280_REG_ID);
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    if(ret!=ESP_OK){
        logE("Error-Reading","Sensor not found");
    }

    if(dev->id!=BMP280_CHIP_ID){
        logE("Chip-ID","Invalid Chip Id");
    }
    else {
        logD("Chip","Chip initialise successfully");
    }
    return ret;
}

esp_err_t bmp280_resetting(i2c_config_t i2c_config){
    uint8_t value=BMP280_RESET_VALUE;
    esp_err_t ret=write_8bits_data_to_slave(i2c_config,&value,1, 0x00);
    if(ret!=ESP_OK){
        logE("Reset","Failed to reset sensor");
    }
    else{
        logD("Reset","Successfully reset done");
    }
    return ret;
}

esp_err_t bmp280_init_calibration(i2c_port_t i2c_num,bmp280_t *dev,i2c_config_t i2c_config){
    read_16bit_data_from_slave(&dev->dig_T1,i2c_config, 0x88);
    read_16bit_data_from_slave((uint16_t*)&dev->dig_T2,i2c_config, 0x8a);
    read_16bit_data_from_slave((uint16_t*)&dev->dig_T3,i2c_config, 0x8c);
    read_16bit_data_from_slave(&dev->dig_P1,i2c_config, 0x8e);
    read_16bit_data_from_slave((uint16_t*)&dev->dig_P2,i2c_config, 0x90);
    read_16bit_data_from_slave((uint16_t*)&dev->dig_P3,i2c_config, 0x92);
    read_16bit_data_from_slave((uint16_t*)&dev->dig_P4,i2c_config, 0x94);
    read_16bit_data_from_slave((uint16_t*)&dev->dig_P5,i2c_config,0x96);
    read_16bit_data_from_slave((uint16_t*)&dev->dig_P6,i2c_config, 0x98);
    read_16bit_data_from_slave((uint16_t*)&dev->dig_P7,i2c_config,0x9a);
    read_16bit_data_from_slave((uint16_t*)&dev->dig_P8,i2c_config, 0x9c);
    esp_err_t ret=read_16bit_data_from_slave((uint16_t*)&dev->dig_P9,i2c_config, 0x9e);
    if(ret!=ESP_OK){
        logE("Error Initialising","Not able to initialise the bmp280");
    }
    
    return ret;
}

esp_err_t bmp280_init_config(bmp280_params_t *params,i2c_config_t i2c_config){
    uint8_t config =(params->standby << 5) | (params->filter << 2);
    esp_err_t ret=write_8bits_data_to_slave(i2c_config,&config,1, BMP280_REG_CONFIG);

    if (params->mode == BMP280_MODE_FORCED){
        params->mode = BMP280_MODE_SLEEP;
    }
    return ret;
}

esp_err_t bmp280_init_ctrl(bmp280_params_t* params,i2c_config_t i2c_config){
    uint8_t ctrl = (params->oversampling_temperature << 5) | (params->oversampling_pressure << 2) | (params->mode);
    esp_err_t ret=write_8bits_data_to_slave(i2c_config,&ctrl,1,BMP280_REG_CTRL);
    if(ret!=ESP_OK){
        logE("Control","Failed to control the sensor");
    }
    return ret;
}

esp_err_t bmp280_init(){


    bmp280_params_t params;

    // setting default parameters
    esp_err_t ret1=bmp280_init_default_params(&params);

    memset(&dev, 0, sizeof(bmp280_t));

    i2c_port_t i2c_num=I2C_PORT;

    i2c_driver_install(I2C_PORT,I2C_MODE_MASTER,SLV_RX_BUF_LEN,SLV_TX_BUF_LEN,INTR_ALLOC_FLAGS);

    esp_err_t ret2=bmp280_init_id(&dev,i2c_config);

    esp_err_t ret3=bmp280_resetting(i2c_config);

    while (1)
    {
        uint8_t status;
        if (!read_8bit_data_from_slave(1,&status,i2c_config,BMP280_REG_STATUS) && (status & 1) == 0)
            break;
    }
    
    esp_err_t ret4=bmp280_init_calibration(i2c_num,&dev,i2c_config);

    esp_err_t ret5=bmp280_init_config(&params,i2c_config);

    esp_err_t ret6=bmp280_init_ctrl(&params,i2c_config);

    return ret1 && ret2 && ret3 && ret4 && ret5 && ret6;
}

int32_t compensate_temperature(bmp280_t *dev,int32_t adc_temp,int64_t *fine_temp){
    int32_t var1, var2;

    var1 = ((((adc_temp >> 3) - ((int32_t)dev->dig_T1 << 1))) * (int32_t)dev->dig_T2) >> 11;
    var2 = (((((adc_temp >> 4) - (int32_t)dev->dig_T1) * ((adc_temp >> 4) - (int32_t)dev->dig_T1)) >> 12) * (int32_t)dev->dig_T3) >> 14;

    *fine_temp = var1 + var2;
    return (*fine_temp * 5 + 128) >> 8;
}

int64_t compensate_pressure(bmp280_t *dev,int32_t adc_presure,int64_t *fine_temp){
    int64_t var1, var2, p;

    var1 = *fine_temp - 128000;
    var2 = var1 * var1 * (int64_t)dev->dig_P6;
    var2 = var2 + ((var1 * (int64_t)dev->dig_P5) << 17);
    var2 = var2 + (((int64_t)dev->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dev->dig_P3) >> 8) + ((var1 * (int64_t)dev->dig_P2) << 12);
    var1 = (((int64_t)1 << 47) + var1) * ((int64_t)dev->dig_P1) >> 33;

    if (var1 == 0)
    {
        return 0;  // to avoid exception caused by division by zero
    }

    p = 1048576 - adc_presure;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)dev->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)dev->dig_P8 * p) >> 19;

    p = ((p + var1 + var2) >> 8) + ((int64_t)dev->dig_P7 << 4);
    return p;
}

esp_err_t bmp280_read_fixed(bmp280_t *dev,int32_t *temperature,int32_t *pressure,i2c_config_t i2c_config){
    size_t size=6;
    uint8_t data[6]; 
    int32_t adc_pressure;
    int32_t adc_temp; 
    if(read_8bit_data_from_slave(size,&data[0],i2c_config, 0xf7)!=ESP_OK){
        logE("Error","Not able to read data,please check the connection");
    }
    adc_pressure=data[0] << 12 | data[1] << 4 | data[2] >> 4;
    adc_temp=data[3] << 12 | data[4] << 4 | data[5] >> 4;

    int64_t fine_temp;
    *temperature=compensate_temperature(dev,adc_temp,&fine_temp);
    *pressure=compensate_pressure(dev,adc_pressure,&fine_temp);
    return ESP_OK;
}

esp_err_t bmp280_read_float(float *temperature,float *pressure){
    int32_t fixed_temperature;
    int32_t fixed_pressure;
    bmp280_read_fixed(&dev,&fixed_temperature,&fixed_pressure,i2c_config);
    *temperature=(float)fixed_temperature/100;
    *pressure=(float)fixed_pressure/256;
    return ESP_OK;
}

