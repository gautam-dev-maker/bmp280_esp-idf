# BMP280 Component
This repository is the component to read pressure and temperature from BMP280 using ESP-IDF. The communication protocol used to communicate with the sensor is I2C. 

## Table of Contents

* [About The BMP280](#about-the-bmp280)
* [I2C Communication Protocol](#i2c-communication-protocol)
  * [Writing Data To Slave](#writing-data-to-slave)
  * [Reading Data From Slave](#reading-data-from-slave)
* [File Structure](#file-structure)
* [Getting Started](#getting-started)
  * [Prerequisites](#prerequisites)
  * [Installation](#installation)
  * [Configuration](#configuration)
* [Usage](#usage)
* [Results and Demo](#results-and-demo)
* [Troubleshooting](#troubleshooting)
* [Contributors](#contributors)
* [Acknowledgements and Resources](#acknowledgements-and-resources)
* [License](#license)

<!-- ABOUT THE PROJECT -->
## About The BMP280
The BMP280 is an absolute barometric pressure sensor, which is especially feasible for mobile applications. Its small dimensions and its low power consumption allow for the implementation in battery-powered devices such as mobile phones, GPS modules or watches.
   
   ![](https://cdn-shop.adafruit.com/1200x900/2651-07.jpg)
   
## I2C Communication Protocol
With I2C, data is transferred in messages. Messages are broken up into frames of data. Each message has an address frame that contains the binary address of the slave, and one or more data frames that contain the data being transmitted. The message also includes start and stop conditions, read/write bits, and ACK/NACK bits between each data frame:
  ![](https://www.circuitbasics.com/wp-content/uploads/2016/01/Introduction-to-I2C-Message-Frame-and-Bit-2.png)
  
  * **Start Condition**: The SDA line switches from a high voltage level to a low voltage level before the SCL line switches from high to low.
  * **Stop Condition**: The SDA line switches from a low voltage level to a high voltage level after the SCL line switches from low to high.
  * **Address Frame**: A 7 or 10 bit sequence unique to each slave that identifies the slave when the master wants to talk to it.
                       The master sends the address of the slave it wants to communicate with to every slave connected to it. Each slave then compares the address         sent from the master to its own address. If the address matches, it sends a low voltage ACK bit back to the master. If the address doesn’t match, the slave does nothing and the SDA line remains high.


  * **Read/Write Bit**: A single bit specifying whether the master is sending data to the slave (low voltage level) or requesting data from it (high voltage level).The address frame includes a single bit at the end that informs the slave whether the master wants to write data to it or receive data from it. If the master wants to send data to the slave, the read/write bit is a low voltage level. If the master is requesting data from the slave, the bit is a high voltage level.
  * **ACK/NACK Bit**: Each frame in a message is followed by an acknowledge/no-acknowledge bit. If an address frame or data frame was successfully received, an ACK bit is returned to the sender from the receiving device.
  * **Data Frame**: After the master detects the ACK bit from the slave, the first data frame is ready to be sent.
The data frame is always 8 bits long, and sent with the most significant bit first. Each data frame is immediately followed by an ACK/NACK bit to verify that the frame has been received successfully. The ACK bit must be received by either the master or the slave (depending on who is sending the data) before the next data frame can be sent.After all of the data frames have been sent, the master can send a stop condition to the slave to halt the transmission. The stop condition is a voltage transition from low to high on the SDA line after a low to high transition on the SCL line, with the SCL line remaining high.

### Writing Data To Slave

The first thing that will happen is that the master will send out a start sequence. This will alert all the slave devices on the bus that a transaction is starting and they should listen in incase it is for them. So the following sequence is followed
* Send a start sequence
* Send the I2C address of the slave with the R/W bit low 
* Send the internal register number you want to write to
* Send the data byte
* Optionally, send any further data bytes
* Send the stop sequence.

![](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/_images/blockdiag-4d6de291dd48a58aacb49d8227add38d18b30991.png)

The following describes how a command link for a “master write” is set up and what comes inside:

* Create a command link with i2c_cmd_link_create().

* Then, populate it with the series of data to be sent to the slave:

* Start bit - ```i2c_master_start()```

* Slave address - ```i2c_master_write_byte()```. The single byte address is provided as an argument of this function call.

* Data - One or more bytes as an argument of ```i2c_master_write()```

* Stop bit - ```i2c_master_stop()```

* Both functions ```i2c_master_write_byte()``` and ```i2c_master_write()``` have an additional argument specifying whether the master should ensure that it has received the ACK bit.

* Trigger the execution of the command link by I2C controller by calling ```i2c_master_cmd_begin()```. Once the execution is triggered, the command link cannot be modified.

* After the commands are transmitted, release the resources used by the command link by calling ```i2c_cmd_link_delete()```.

```c
esp_err_t write_data8(i2c_config_t i2c_config,uint8_t *value,size_t size,uint8_t reg){
    i2c_param_config(I2C_NUM_0,&i2c_config);

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

    esp_err_t res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return res;
}

```

for more info on API refer [this](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html#master-write)




### Reading Data From Slave
Before reading data from the slave device, you must tell it which of its internal addresses you want to read. So a read of the slave actually starts off by writing to it. So the following sequence is followed :- 

![](https://www.robot-electronics.co.uk/images/i2c.GIF)

* Send a Start Sequence 
* Send I2C Address of the Master with the R/W bit low (To write Data to the slave)
* Send Internal Address of the bearing register
* Send a start sequence again (repeated start)
* Read data byte from the register
* Send Stop Sequence 

![](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/_images/blockdiag-b03a38ff34e4be444769f59680b11809fdb47fa0.png)

```c
esp_err_t read_data(uint8_t size,uint8_t* data,i2c_config_t i2c_config,uint8_t reg){
    i2c_param_config(I2C_NUM_0,&i2c_config);

    //creating a command handle
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

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

    esp_err_t res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return res;
}

```
for more info on this API refer [this](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html#master-read)
### File Structure
    .
    ├── Components              
    │      ├──bmp280                  # Component to read from BMP280
    │      │   ├── include            # header file of the bmp280 component
    │      │   ├── CMakeLists.txt     # contains execution command for the command
    │      │   └── bmp280.c           # Source file of bmp280 component
    │      └──logger                  # logging component used to generate logs
    │          ├── include            # header file of the logger component
    │          ├── CMakeLists.txt     # contains execution Command for the component
    │          └── logger.c           # Source file of Logger component
    ├── example                       # Contains the example to use the component
    │     ├── main.c                  # Main Source code to be executed
    │     └── CMakeLists.txt          # contains commands to execute the example file
    ├── References                    # Various resources reffered 
    ├── Assets                        # contains screenshots of the result
    ├── LICENSE                       
    └── README.md

## Getting Started

### Prerequisites
Install ESP-IDF : https://github.com/espressif/esp-idf

### Installation
Clone the project
```
https://github.com/gautam-dev-maker/bmp280_esp_idf.git

cd bmp280_esp_idf
```
## Usage

Build
```
idf.py build
```
Flash
```
idf.py -p (PORT) flash monitor

```
### Configuration

```
idf.py menuconfig
```
* `Example Connection Configuration`
  * `Bluetooth Name` - Set Bluetooth Name
  
* `MPU6050 Configuration
  * `SDA Pin No.` - Set SDA Pin No.
  * `CLK Pin No.` - Set CLK Pin No.
* The default Pin configuration used to connect MPU_6050 with ESP32 in this project is shown ![](docs/results/Esp-32andmpu6050_pin_connection.png)  ![](docs/results/Air-Mouse_diagram.png)
  
## Results and Demo
The use of Right and Left Capacitive touch pins has been demonstrated in the following videos

 [Right/left buttons](https://github.com/gautam-dev-maker/Air-Mouse/blob/master/docs/results/Right-Left%20click.mp4)
 
 ## Troubleshooting
 While Configuring for the first time if Bluetooth is not working then ,go to terminal
 
```
idf.py menuconfig
```
Then go to components/bluetooth and enable bluetooth
Press ctrl+s to save the configuration
then
```
idf.py build
```
## Contributors
* [Laukik Hase](https://github.com/laukik-hase)
* [Gautam Agrawal](https://github.com/gautam-dev-maker)
 
  
## License
The [License](https://github.com/gautam-dev-maker/Air-Mouse/blob/master/LICENSE) Used for this Project.
  
  
  
