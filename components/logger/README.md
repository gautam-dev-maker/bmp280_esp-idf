# LOGGER
logging library for ESP32.

## Table of Contents

* [INTRODUCTION](#introduction)
* [REQUIREMENTS](#requirements)
* [INSTALLATION](#installation)
* [USAGE](#usage)
  * [PARAMETERS](#parameters)
  * [EXAMPLE](#example)
* [DRAWBACK](#drawback)
* [CONTRIBUTORS](#contributors)
* [REFERENCES](#references)
* [LICENSE](#license)

## INTRODUCTION
Debugging a microcontroller can be a tedious task and often requires special hardware. Many developers even prefer to use the good old ```printf()```. 
However ESP32 has limited SRAM and storing debug strings is a waste of memory.

The logger provides solution to this problem. It:
- supports printing format string from flash memory
- suports different types of log levels
- makes logging easy and efficient

## REQUIREMENTS
* ESP-IDF v4.0 or later

## INSTALLATION
To use this logger in esp project follow this steps:- 
```
cd <your_esp_idf_project>
mkdir components
cd components
git clone <repository>
```

## USAGE
This library provides 5 different types of log as listed below:-

Log Type | Function | Description 
--- | --- | --- 
ERROR | ```logE(TAG, fmt...)``` | Generate log with log level ERROR. It prints the error in red colour to distinguish from other log levels
WARNING | ```logW(TAG, fmt...)```| Generate log with log level WARNING. It prints the warning in yellow colour.
INFO | ```logI(TAG, fmt...)```| Generate log with log level INFO. It prints the info in green colour.
DEBUG | ```logD(TAG, fmt...)```| Generate log with log level DEBUG. It prints the debug in white colour.
VERBOSE| ```logV(TAG, fmt...)```| Generate log with log level VERBOSE. It prints the verbose in white colour.

### PARAMETERS
Each function mentioned above takes these parameters:-
* TAG :- A constant char to identify the module
* fmt :- string format of the log to print

## EXAMPLE
```C
#include<logger.h>

void app_main(){

    logE("log-Error","Never ever code like this"); // write log with log level -> ERROR
    logD("log-Debugging","This is for debugging"); // write log with log level -> DEBUG
    logI("log-Info","Donald Trump is over");       // write log with log level -> INFO
    logW("log-warning","Melody is choclatey");     // write log with log level -> WARNING
    logV("log-Verbose","Just some gibberish");     // write log with log level -> VERBOSE
}
```
## DRAWBACK
ESP32 only supports 120 character at a time, so it can print only 120 characters at a time. So the only solution is to keep the log short.

## CONTRIBUTORS
* [Laukik Hase](https://github.com/laukik-hase)

## REFERENCES
* [RXI](https://github.com/rxi/log.c)
* [JoasoLopesF](https://github.com/JoaoLopesF/Esp-Idf-Improved-Logging)
