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
#include <Adafruit_MAX31865.h>
#include <QuickPID.h>

#include "LT8722.h"
#include "ControlTFT.h"

#define H2SENSOR1 15
#define H2SENSOR2 16

TaskHandle_t TaskCompute;
TaskHandle_t TaskDisplay;
TaskHandle_t TaskPID;
SemaphoreHandle_t SemaphoreDataControl;
SemaphoreHandle_t SemaphoreDataTests;
SemaphoreHandle_t SemaphoreDataStatus;
SemaphoreHandle_t SemaphoreDataCommand;
SemaphoreHandle_t SemaphoreDataPID;

LT8722 peltierDriver;                                           //create a LT8722 object with FSPI

SPIClass fspi(FSPI);
Adafruit_MAX31865 pt1000 = Adafruit_MAX31865(9, &fspi);         //create a Adafruit_MAX31865 object with FSPI

const float refResistance = 4300.0;                             //the value of the Rref resistor
float nominalResistance   = 933.5;                              //the 'nominal' 0-degrees-C resistance of the sensor

float startTemperature  = 0.0;
float endTemperature    = 40.0;
float riseTime          = 20.0;
float fallTime          = 20.0;
float numberCycles      = 5.0;
float timeAtTemperature = 0.0;
float numberSteps       = 0.0;
float progress          = 0.0;

float Kp = 0.09, Ki = 0.25, Kd = 0;                             //PID tuning parameters 
float Setpoint = 10, Input, Output;                             //variables for PID      
QuickPID peltierPID(&Input, &Output, &Setpoint);                //specify PID links

ControlTFT display;
struct controlData dataControl = {};
struct testsData dataTests = {};
struct statusData dataStatus = {};
struct commandData dataCommand = {};

void checkPT1000Falts();
void changeValue(bool increase, bool decrease, double step, float* value);

void taskcompute_loop(void * parameter) {
  for( ;; ) {
    xSemaphoreTake(SemaphoreDataPID, portMAX_DELAY);
    Input = pt1000.temperature(nominalResistance, refResistance);
    xSemaphoreGive(SemaphoreDataPID);

    if (display.getTab() == PRESSED_TAB::CONTROL) {
      xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
      dataControl.valueChipTemperature    = peltierDriver.readAnalogOutput(ANALOG_OUTPUT::TEMPERATURE);
      dataControl.valueOutputVoltage      = peltierDriver.readAnalogOutput(ANALOG_OUTPUT::VOLTAGE);
      dataControl.valueOutputCurrent      = peltierDriver.readAnalogOutput(ANALOG_OUTPUT::CURRENT);
      dataControl.valueNominalResistance  = nominalResistance;
      dataControl.valueSetTemperature     = Setpoint;
      dataControl.valueCurrentTemperature = pt1000.temperature(nominalResistance, refResistance);

      if (display.getMeasurementH2Sensor1Running()) {
        double voltage = analogRead(H2SENSOR1);
        voltage /= 1000;
        dataControl.valueMeasureH2Sensor1 = voltage;
      } else {
        dataControl.valueMeasureH2Sensor1 = 0.0;
      }

      if (display.getMeasurementH2Sensor2Running()) {
        double voltage = analogRead(H2SENSOR2);
        voltage /= 1000;
        dataControl.valueMeasureH2Sensor2 = voltage;
      } else {
        dataControl.valueMeasureH2Sensor2 = 0.0;
      }
      xSemaphoreGive(SemaphoreDataControl);
      
      xSemaphoreTake(SemaphoreDataPID, portMAX_DELAY);
      changeValue(dataControl.buttonIncreaseSetTemperaturePressed, dataControl.buttonDecreaseSetTemperaturePressed, 0.5, &Setpoint);
      xSemaphoreGive(SemaphoreDataPID);

      if (dataControl.buttonStartPressed) {
        peltierDriver.softStart();                                            //softstart of the LT8722 (resets all registers)
        peltierDriver.setPositiveVoltageLimit(VOLTAGE_LIMIT::LIMIT_5_00);     //set the positive voltage limit to 5V
        peltierDriver.setNegativeVoltageLimit(VOLTAGE_LIMIT::LIMIT_5_00);     //set the negative voltage limit to -5V
        peltierDriver.setPositiveCurrentLimit(4.5);                           //set the positive current limit to 4.5A
        peltierDriver.setNegativeCurrentLimit(4.5);                           //set the negative current limit to -4.5A
      } else if (dataControl.buttonStopPressed) {
        peltierDriver.powerOff();
      } else if (dataControl.buttonResetPressed) {
        peltierDriver.reset();
        xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
        dataControl.eventCommunicationError = false;
        xSemaphoreGive(SemaphoreDataControl);
      }

      changeValue(dataControl.buttonIncreaseNominalResistancePressed, dataControl.buttonDecreaseNominalResistancePressed, 0.1, &nominalResistance);

    } else if (display.getTab() == PRESSED_TAB::TESTS) {
      xSemaphoreTake(SemaphoreDataTests, portMAX_DELAY);
        dataTests.valueStartTemperature  = startTemperature;
        dataTests.valueEndTemperature    = endTemperature;
        dataTests.valueRiseTime          = riseTime;
        dataTests.valueFallTime          = fallTime;
        dataTests.valueNumberCycles      = numberCycles;
        dataTests.valueTimeAtTemperature = timeAtTemperature;
        dataTests.valueNumberSteps       = numberSteps;
      xSemaphoreGive(SemaphoreDataTests);

      changeValue(dataTests.buttonIncreaseStartTemperaturePressed , dataTests.buttonDecreaseStartTemperaturePressed , 0.5, &startTemperature);
      changeValue(dataTests.buttonIncreaseEndTemperaturePressed   , dataTests.buttonDecreaseEndTemperaturePressed   , 0.5, &endTemperature);
      changeValue(dataTests.buttonIncreaseRiseTimePressed         , dataTests.buttonDecreaseRiseTimePressed         , 0.5, &riseTime);
      changeValue(dataTests.buttonIncreaseFallTimePressed         , dataTests.buttonDecreaseFallTimePressed         , 0.5, &fallTime);
      changeValue(dataTests.buttonIncreaseNumberCyclesPressed     , dataTests.buttonDecreaseNumberCyclesPressed     ,   1, &numberCycles);
      changeValue(dataTests.buttonIncreaseTimeAtTemperaturePressed, dataTests.buttonDecreaseTimeAtTemperaturePressed, 0.5, &timeAtTemperature);
      changeValue(dataTests.buttonIncreaseNumberStepsPressed      , dataTests.buttonDecreaseNumberStepsPressed      ,   1, &numberSteps);

      if (timeAtTemperature == 0 && numberSteps == 0) {
        delay(10);
      }

    } else if (display.getTab() == PRESSED_TAB::STATUS) {
      uint16_t status = peltierDriver.getStatus();

      xSemaphoreTake(SemaphoreDataStatus, portMAX_DELAY);
      dataStatus.eventPWMSwitchingEvent      = status & 1;
      dataStatus.eventCurrentLimitEvent      = (status >> 1) & 1;
      dataStatus.eventPowerLimitEvent        = (status >> 2) & 1;
      dataStatus.eventSoftResetEvent         = (status >> 4) & 1;
      dataStatus.eventOverCurrentEvent       = (status >> 5) & 1;
      dataStatus.eventOverttemperatureEvent  = (status >> 6) & 1;
      dataStatus.eventVCCUnderVoltageEvent   = (status >> 7) & 1;
      dataStatus.eventVDDIOUnderVOltageEvent = (status >> 8) & 1;
      xSemaphoreGive(SemaphoreDataStatus);
    } else if (display.getTab() == PRESSED_TAB::COMMAND) {
      uint32_t command = peltierDriver.getCommand();

      xSemaphoreTake(SemaphoreDataCommand, portMAX_DELAY);
      dataCommand.eventPowerStage         = command & 1;
      dataCommand.eventPWMSwitching       = (command >> 1) & 1;
      dataCommand.infoPWMFrequency        = (command >> 2) & 0b111;
      dataCommand.infoPWMAdjust           = (command >> 5) & 0b11;
      dataCommand.infoPWMDutyCycle        = (command >> 7) & 0b11;
      dataCommand.eventLDORegulation      = (command >> 9) & 1;
      dataCommand.infoPeakInductorCurrent = (command >> 11) & 0b111;
      dataCommand.infoPowerLimit          = (command >> 15) & 0b11;
      xSemaphoreGive(SemaphoreDataCommand);
    }
  }
  vTaskDelete( NULL );
}

void taskdisplay_loop(void * parameter) {
  for( ;; ) {
    display.drawTabSelect();

    if (display.getTab() == PRESSED_TAB::CONTROL) {
      xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
      dataControl = display.drawControlTab(dataControl);
      xSemaphoreGive(SemaphoreDataControl);
    } else if (display.getTab() == PRESSED_TAB::TESTS) {
      xSemaphoreTake(SemaphoreDataTests, portMAX_DELAY);
      dataTests = display.drawTestsTab(dataTests);
      xSemaphoreGive(SemaphoreDataTests);
    } else if (display.getTab() == PRESSED_TAB::STATUS) {
      xSemaphoreTake(SemaphoreDataStatus, portMAX_DELAY);
      display.drawStatusTab(dataStatus);
      xSemaphoreGive(SemaphoreDataStatus);
    } else if (display.getTab() == PRESSED_TAB::COMMAND) {
      xSemaphoreTake(SemaphoreDataCommand, portMAX_DELAY);
      display.drawCommandTab(dataCommand);
      xSemaphoreGive(SemaphoreDataCommand);
    }
  }
  vTaskDelete( NULL );
}

void taskpid_loop(void * parameter) {
  for( ;; ) {
    if (display.getControlRunning() || display.getTestRunning()) {
      xSemaphoreTake(SemaphoreDataPID, portMAX_DELAY);
      peltierPID.Compute();
      xSemaphoreGive(SemaphoreDataPID);

      bool error = false;
      error = peltierDriver.setVoltage(Output);

      if (error) {
        xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
        dataControl.eventCommunicationError = true;
        xSemaphoreGive(SemaphoreDataControl);
        display.setControlRunning(false);
        display.setTestRunning(false);
        peltierDriver.powerOff();
      }
    }
  }
  vTaskDelete( NULL );
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  display.begin();
  display.drawTabSelect(true);

  pt1000.begin(MAX31865_2WIRE);                                         //initialize pt1000 in 2 wire mode
  
  peltierDriver.begin();                                                //initialize the SPI interface with the standard pins
  
  peltierPID.SetTunings(Kp, Ki, Kd);                                    //apply PID gains
  peltierPID.SetOutputLimits(-3.5, 3.5);                                //set min and max output limits in V
  peltierPID.SetControllerDirection(peltierPID.Action::reverse);        //set PID to reverse mode
  peltierPID.SetMode(peltierPID.Control::automatic);                    //turn the PID on

  SemaphoreDataControl = xSemaphoreCreateMutex();
  SemaphoreDataTests   = xSemaphoreCreateMutex();
  SemaphoreDataStatus  = xSemaphoreCreateMutex();
  SemaphoreDataCommand = xSemaphoreCreateMutex();
  SemaphoreDataPID     = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(
    taskcompute_loop,
    "TaskCompute",
    10000,
    NULL,
    5,
    &TaskCompute,
    1);

  xTaskCreatePinnedToCore(
    taskdisplay_loop,
    "TaskDisplay",
    20000,
    NULL,
    15,
    &TaskDisplay,
    1);

  xTaskCreatePinnedToCore(
    taskpid_loop,
    "TaskPID",
    10000,
    NULL,
    2,
    &TaskPID,
    1);
}

void loop() { 
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

void changeValue(bool increase, bool decrease, double step, float* value) {
  if (increase) {
    *value += step;
  } else if (decrease) {
    *value -= step;
  }
}