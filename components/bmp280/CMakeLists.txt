set(srcs
	"src/i2c_utils.c"
	"src/bmp280.c")
    
idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "include"
                       REQUIRES driver efuse esp32 esp_adc_cal esp_common esp_event 
                                esp_rom freertos heap logger soc spiffs)
