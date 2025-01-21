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
 * Date: 2025-01-21 Author: Jan kleine Piening Comments: Added temperature reading and PID control
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
#include <QuickPID.h>

LT8722 peltierDriver;                                           //create a LT8722 object with FSPI

SPIClass fspi(FSPI);
Adafruit_MAX31865 pt1000 = Adafruit_MAX31865(9, &fspi);         //create a Adafruit_MAX31865 object with FSPI

#define RREF      4300.0                                        //the value of the Rref resistor
#define RNOMINAL  942.0                                         //the 'nominal' 0-degrees-C resistance of the sensor

float Kp = 0.09, Ki = 0.25, Kd = 0;                             //PID tuning parameters 
float Setpoint, Input, Output;                                  //variables for PID      
QuickPID peltierPID(&Input, &Output, &Setpoint);                //specify PID links

void checkPT1000Falts();

void setup() {
  Serial.begin(115200);
  delay(5000);

  pt1000.begin(MAX31865_2WIRE);                                         //initialize pt1000 in 2 wire mode
  
  peltierDriver.begin();                                                //initialize the SPI interface with the standard pins
  peltierDriver.softStart();                                            //softstart of the LT8722 (resets all registers)
  peltierDriver.setPositiveVoltageLimit(VOLTAGE_LIMIT::LIMIT_5_00);     //set the positive voltage limit to 5V
  peltierDriver.setNegativeVoltageLimit(VOLTAGE_LIMIT::LIMIT_5_00);     //set the negative voltage limit to -5V
  peltierDriver.setPositiveCurrentLimit(4.5);                           //set the positive current limit to 4.5A
  peltierDriver.setNegativeCurrentLimit(4.5);                           //set the negative current limit to -4.5A

  Setpoint = 40;                                                        //initial setpoint for PID in °C
  peltierPID.SetTunings(Kp, Ki, Kd);                                    //apply PID gains
  peltierPID.SetOutputLimits(-3.5, 3.5);                                //set min and max output limits in V
  peltierPID.SetControllerDirection(peltierPID.Action::reverse);        //set PID to reverse mode
  peltierPID.SetMode(peltierPID.Control::automatic);                    //turn the PID on

  bool error = peltierDriver.setVoltage(0);                             //set LT8722 output to 0V
  Serial.println("LT8722 ERROR:");
  Serial.println(error);
  delay(5000);
}

void loop() {
  double temperature = pt1000.temperature(RNOMINAL, RREF);
  Serial.print("Temperature = "); Serial.println(temperature);

  Input = temperature;
  peltierPID.Compute();
  Serial.print("Voltage = "); Serial.println(Output);

  bool error = peltierDriver.setVoltage(Output);
  Serial.println("LT8722 ERROR:");
  Serial.println(error);

  checkPT1000Falts();
}

void checkPT1000Falts() {
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
}
