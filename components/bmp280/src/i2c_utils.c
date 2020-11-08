#include "i2c_utils.h"

static const char *TAG_I2C = "i2c_utils";

esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    esp_err_t ret = i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK)
        ESP_LOGE(TAG_I2C, "I2C Master Initialisation Failed!");
    return ret;
}

esp_err_t i2c_write_to_slave(uint8_t slave_addr, uint8_t reg_addr, size_t len, uint8_t *data_wr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t i2c_read_from_slave(uint8_t slave_addr, uint8_t reg_add, size_t len, uint8_t *data_rd)
{
    if (len == 0)
        return ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_add, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | READ_BIT, ACK_CHECK_EN);
    if (len > 1)
        i2c_master_read(cmd, data_rd, len - 1, ACK_VAL);
    i2c_master_read_byte(cmd, data_rd + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t i2c_read_int16_little_endian(uint8_t slave_addr, uint8_t reg_add, int16_t *data_rd)
{
    uint8_t raw_data_8bit[2];
    esp_err_t ret = i2c_read_from_slave(slave_addr, reg_add, 2, raw_data_8bit);
    *data_rd = (raw_data_8bit[1] << 8) + raw_data_8bit[0];
    return ret;
}

esp_err_t i2c_read_int32_big_endian(uint8_t slave_addr, uint8_t reg_add, int32_t *data_rd)
{
    uint8_t raw_data_8bit[3];
    *data_rd = 0;
    
    esp_err_t ret = i2c_read_from_slave(slave_addr, reg_add, 3, raw_data_8bit);
    *data_rd = (raw_data_8bit[0] << 12) | (raw_data_8bit[1] << 4) | (raw_data_8bit[2] >> 4);

    return ret;
}

