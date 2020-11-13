# BMP280 Component
This repository is the component to read pressure and temperature from BMP280 using ESP-IDF. The communication protocol used to communicate with the sensor is I2C. 

## Table of Contents

* [About The BMP280](#about-the-bmp280)
* [I2C Communication Protocol](#i2c-communication-protocol)
  * [Reading Data From Slave] (#reading-data-from-slave)
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

### Reading Data From Slave
Before reading data from the slave device, you must tell it which of its internal addresses you want to read. So a read of the slave actually starts off by writing to it. So the following sequence is followed

![](https://www.robot-electronics.co.uk/images/i2c.GIF)

* Send a Start Sequence 
* Send I2C Address of the Master with the R/W bit low (To write Data to the slave)
* Send Internal Address of the bearing register
* Send a start sequence again (repeated start)
* Read data byte from the register
* Send Stop Sequence 


  

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
  
  
  
