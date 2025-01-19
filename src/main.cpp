/*
 * File Name: main.cpp
 * Description: The following code can be used to control the measurement
 *              setup of my master's thesis. It uses a TFT display, a 
 *              peltier element for cooling or heating, the DC3145A evaluation 
 *              board from Analog Devices to control the peltier element and 
 *              the MAX31865 to determine the temperature using a PT1000. In 
 *              addition, a specially designed and manufactured chip is 
 *              required, which also contains the PT1000. A PID controller 
 *              is implemented to regulate the temperature.
 *
 * Revision History:
 * Date: 2025-01-06 Author: Jan kleine Piening Comments: Initial version created
 *
 * Author: Jan kleine Piening Start Date: 2025-01-06
 *
 * Notes: This code was written as part of my master's thesis at the 
 *        Institute for Microsensors, -actuators and -systems (IMSAS) 
 *        at the University of Bremen.
 */

#include <Arduino.h>
#include <LT8722.h>

LT8722 peltierDriver;                                           //create a LT8722 object with FSPI

bool error;                                                     //error variable for the SPI communication

void setup() {
  Serial.begin(115200);
  delay(5000);

  peltierDriver.begin();                                        //initialize the SPI interface with the standard pins
  peltierDriver.softStart();                                    //softstart of the LT8722 (resets all registers)
  peltierDriver.setPositiveVoltageLimit(VOLTAGE_LIMIT_5_00);    //set the positive voltage limit to 5V
  peltierDriver.setNegativeVoltageLimit(VOLTAGE_LIMIT_5_00);    //set the negative voltage limit to -5V
  peltierDriver.setPositiveCurrentLimit(4.5);                   //set the positive current limit to 4.5A
  peltierDriver.setNegativeCurrentLimit(4.5);                   //set the negative current limit to -4.5A
}

void loop() {

}