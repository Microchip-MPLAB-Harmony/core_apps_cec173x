---
parent: Harmony 3 driver and system service application examples for CEC173x family
title: W25 SQI flash driver - Flash read write in Quad IO mode 
has_children: false
has_toc: false
---

[![MCHP](https://www.microchip.com/ResourcePackages/Microchip/assets/dist/images/logo.png)](https://www.microchip.com)

# W25 SQI flash driver - Flash read write in Quad IO mode

This example application shows how to use the W25 SQI flash driver to perform block operations on the On-Board W25 Flash memory in Quad IO mode.

## Description

This application uses the W25 driver to Erase/Write/Read on the On-Board W25 Flash memory using the QMSPI peripheral library in Quad IO mode.

The application consists of APP_W25_Tasks() which is called through SYS_Tasks() routine.

## Downloading and building the application

To clone or download this application from Github, go to the [main page of this repository](https://github.com/Microchip-MPLAB-Harmony/core_apps_cec173x) and then click Clone button to clone this repository or download as zip file.
This content can also be downloaded using content manager by following these [instructions](https://github.com/Microchip-MPLAB-Harmony/contentmanager/wiki).

Path of the application within the repository is **apps/driver/sqi_flash/W25/W25_sqi_read_write/firmware** .

To build the application, refer to the following table and open the project using its IDE.

| Project Name      | Description                                    |
| ----------------- | ---------------------------------------------- |
| cec1736_evb.X | MPLABX project for [CEC 1736 Development Board](https://www.microchip.com/en-us/development-tool/EV19K07A)     |

## Setting up the hardware

The following table shows the target hardware for the application projects.

| Project Name| Board|
|:---------|:---------:|
| cec1736_evb.X | MPLABX project for [CEC 1736 Development Board](https://www.microchip.com/en-us/development-tool/EV19K07A)   |

### Setting up [CEC 1736 Development Board](https://www.microchip.com/en-us/development-tool/EV19K07A)

- Connect the USB port P2 on the board to the computer using a micro USB cable

## Running the Application

1. Build and program the application using its IDE
2. LED is turned ON when the data read from W25 flash matches with the data written in it

Refer to the following table for LED name:

| Board                                                                                   | LED Name |
| --------------------------------------------------------------------------------------- | -------- |
| [CEC 1736 Development Board](https://www.microchip.com/en-us/development-tool/EV19K07A) | LED5     |
