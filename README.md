# Peltier Controller

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.15282554.svg)](https://doi.org/10.5281/zenodo.15282554)

## Overview
This code is used to set up and control a measurement setup. The measurement setup contains a peltier element to set temperatures between -5 °C and 150 °C and a TFT touch display with SPI and the ILI9488 driver to display current information and to be able to configure and start tests. In addition, two external hydrogen sensors can be connected which output a voltage in relation to the hydrogen concentration. Furthermore, a custom chip with an integrated PT1000 is required to determine the current temperature of the Peltier element. A MAX31865 with the Adafruid PCB layout is used for this purpose.

To control the tests and adjust settings, the GUI is divided into four tabs. The first tab is the control tab. This tab can be used to manually set the temperature of the peltier element, to set the correct nominal temperature of the PT1000 and to start the measurement with the hydrogen sensors 1 and 2. In addition, this tab is used to monitor the LT8722 (peltier driver), as the current temperature, output voltage and output current are displayed.

The next tab is the tests tab. This tab is used to set different tests and to start them. In this tab it is possible to set the start and end temperature, the rise and fall time, the rise step size and fall step size and the number of test rounds.

The last two tabs are used for further monitoring of the LT8722. The status tab shows the status register of the LT8722. Possible errors are saved here. The control tab shows the control register of the LT8722, in which the current settings are stored.

## Packages
- Adafruit BusIO
- Adafruit MAX31865 library
- QuickPID
- TFT_eSPI
    - uncomment the correct setup file for the used driver
        - for example: #include <User_Setups/Setup21_ILI9488.h>
    - changed to the used SPI PINs
        - TFT_CS:   2 
        - TFT_MOSI: 35
        - TFT_SCLK: 36
        - TFT_MISO: 37
        - TFT_DC:   7
        - TFT_RST:  6
        - TOUCH_CS: 1

## Used Hardware
- ESP32 S3
- 3.5-Zoll TFT touch display with SPI and ILI9488
- Adafruit MAX31865
- 2x FH2-HY11 H2 Sensor
- custom chip with PT1000
- peltier element (QuickCool QC-31-1.0-3.9AS)
- heat sink with fan for peltier cooling

## Note
This code was written as part of my master's thesis at the Institute for Microsensors, -actuators and -systems (IMSAS) at the University of Bremen.
