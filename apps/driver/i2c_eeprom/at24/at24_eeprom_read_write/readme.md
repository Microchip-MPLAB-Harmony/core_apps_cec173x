---
parent: Harmony 3 driver and system service application examples for CEC173x family
title: I2C EEPROM AT24 driver - EEPROM Read Write 
has_children: false
has_toc: false
---

[![MCHP](https://www.microchip.com/ResourcePackages/Microchip/assets/dist/images/logo.png)](https://www.microchip.com)

# I2C EEPROM AT24 driver - EEPROM Read Write

This example application shows how to use the AT24 driver to perform read and write operations on AT24 series of EEPROM.

## Description

- This example uses the AT24 driver to communicate with I2C based On-Board AT24MAC402 series EEPROMs to perform write and read operations in Bare-Metal environment.

## Downloading and building the application

To clone or download this application from Github, go to the [main page of this repository](https://github.com/Microchip-MPLAB-Harmony/core_apps_cec173x) and then click Clone button to clone this repository or download as zip file.
This content can also be downloaded using content manager by following these [instructions](https://github.com/Microchip-MPLAB-Harmony/contentmanager/wiki).

Path of the application within the repository is **apps/driver/i2c_eeprom/at24/at24_eeprom_read_write/firmware** .

To build the application, refer to the following table and open the project using its IDE.

| Project Name      | Description                                    |
| ----------------- | ---------------------------------------------- |
| cec1736_evb.X | MPLABX project for [CEC 1736 Development Board](https://www.microchip.com/en-us/development-tool/EV19K07A)     |
|||

## Setting up the hardware

The following table shows the target hardware for the application projects.

| Project Name| Board|
|:---------|:---------:|
| cec1736_evb.X | [CEC 1736 Development Board](https://www.microchip.com/en-us/development-tool/EV19K07A) 
|||

### Setting up [CEC 1736 Development Board](https://www.microchip.com/en-us/development-tool/EV19K07A)

- Interface the [EEPROM 3 click Board](https://www.mikroe.com/eeprom-3-click) by making the following connections:
    - Connect a wire from GPIO140 (I2C06_SCL) available on Pin 28 of P4 header to the SCL pin of the EEPROM 3 click board
    - Connect a wire from GPIO132 (I2C06_SDA) available on Pin 26 of P4 header to the SDA pin of the EEPROM 3 click board
    - Connect GND and VCC between the [EEPROM 3 click Board](https://www.mikroe.com/eeprom-3-click) and the [CEC 1736 Development Board](https://www.microchip.com/en-us/development-tool/EV19K07A) 
- Connect the Debug USB port on the board to the computer using a micro USB cable

## Running the Application

1. Build and Program the application using its IDE
2. LED indicates success or failure:
    - The LED is turned ON when the value read from the EEPROM matched with the written value
    - The LED is turned OFF when the value read from the EEPROM did not match with the written value

The following table provides the LED name:

| Board      | LED Name |
| ---------- | ---------------- |
| [CEC 1736 Development Board](https://www.microchip.com/en-us/development-tool/EV19K07A) | LED6 |
|||
