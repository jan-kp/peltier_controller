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
#include <Adafruit_MAX31865.h>

LT8722 peltierDriver;                                           //create a LT8722 object with FSPI
bool error;                                                     //error variable for the SPI communication

Adafruit_MAX31865 pt1000 = Adafruit_MAX31865(9);
// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      4300.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  942.0

void setup() {
  Serial.begin(115200);
  delay(5000);

  pt1000.begin(MAX31865_2WIRE);  // set to 2WIRE or 4WIRE as necessary

  peltierDriver.begin();                                                //initialize the SPI interface with the standard pins
  peltierDriver.softStart();                                            //softstart of the LT8722 (resets all registers)
  peltierDriver.setPositiveVoltageLimit(VOLTAGE_LIMIT::LIMIT_5_00);     //set the positive voltage limit to 5V
  peltierDriver.setNegativeVoltageLimit(VOLTAGE_LIMIT::LIMIT_5_00);     //set the negative voltage limit to -5V
  peltierDriver.setPositiveCurrentLimit(4.5);                           //set the positive current limit to 4.5A
  peltierDriver.setNegativeCurrentLimit(4.5);                           //set the negative current limit to -4.5A
}

void loop() {
  error = peltierDriver.setVoltage(0);
  Serial.println("LT8722 ERROR:");
  Serial.println(error);

  uint16_t rtd = pt1000.readRTD();

  Serial.print("RTD value: "); Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
  Serial.print("Ratio = "); Serial.println(ratio,8);
  Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  Serial.print("Temperature = "); Serial.println(pt1000.temperature(RNOMINAL, RREF));

  // Check and print any faults
  uint8_t fault = pt1000.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    pt1000.clearFault();
  }
  Serial.println();
  delay(1000);
}