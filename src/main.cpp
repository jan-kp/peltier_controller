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
 * Date: 2025-01-06 Author: Jan kleine Piening Comments: initial version created
 * Date: 2025-01-21 Author: Jan kleine Piening Comments: added temperature reading and PID control
 * Date: 2025-01-22 Author: Jan kleine Piening Comments: added ControlTFT class to control the display with own widgets
 * Date: 2025-01-24 Author: Jan kleine Piening Comments: added control, tests, status and command tab
 * Date: 2025-01-27 Author: Jan kleine Piening Comments: finalized tests tab
 * Date: 2025-01-28 Author: Jan kleine Piening Comments: added data logging and improved test behavior
 * Date: 2025-01-28 Author: Jan kleine Piening Comments: docs: added more comments
 * Date: 2025-01-29 Author: Jan kleine Piening Comments: docs: improved comments
 * Date: 2025-01-29 Author: Jan kleine Piening Comments: docs: added README
 * Date: 2025-01-31 Author: Jan kleine Piening Comments: func: added the functionality to control temperature and voltage
 * Date: 2025-02-03 Author: Jan kleine Piening Comments: func: changed functions to improve touch responsiveness
 * Date: 2025-02-03 Author: Jan kleine Piening Comments: (1.0.0) docs: added LICENSE and changed README.md
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

#define H2SENSOR1 15      //analog read pin of the first H2 sensor
#define H2SENSOR2 16      //analog read pin of the second H2 sensor

#define RTD_A 2.283e-3    //alpha of the PT1000
#define RTD_B -5.775e-7   //beta of the PT1000

//taskhandle for the different tasks
TaskHandle_t TaskCompute;
TaskHandle_t TaskDisplay;
TaskHandle_t TaskPID;
TaskHandle_t TaskTest;
TaskHandle_t TaskLogging;

//semaphores to make the data transfer between the tasks possible
SemaphoreHandle_t SemaphoreDataControl;
SemaphoreHandle_t SemaphoreDataTests;
SemaphoreHandle_t SemaphoreDataStatus;
SemaphoreHandle_t SemaphoreDataCommand;
SemaphoreHandle_t SemaphoreDataPID;

LT8722 peltierDriver;                                     //create a LT8722 object with FSPI

SPIClass fspi(FSPI);
Adafruit_MAX31865 pt1000 = Adafruit_MAX31865(9, &fspi);   //create a Adafruit_MAX31865 object with FSPI

float Kp = 0.09, Ki = 0.25, Kd = 0;                       //PID tuning parameters 
float Setpoint = 10, Input, Output;                       //variables for PID      
QuickPID peltierPID(&Input, &Output, &Setpoint);          //specify PID links

ControlTFT display;                                       //create a TFT_eSPI object

const float refResistance = 4300.0;                       //the value of the Rref resistor
float nominalResistance   = 898;                          //the 'nominal' 0-degrees-C resistance of the sensor
uint16_t RTDraw           = 0;                            //raw RTD value of the PT1000
double currentResistance  = 0;                            //calculates PT1000 resistance

float startValue   = 0.0;                                 //start temperature for the tests
float endValue     = 60.0;                                //end temperature for the tests
float riseTime     = 30.0;                                //rise time for the tests
float fallTime     = 30.0;                                //fall time for the tests
float riseStepSize = 0.5;                                 //rise step size for the tests
float fallStepSize = 0.5;                                 //fall step size for the tests
float numberCycles = 5.0;                                 //number of tests cycles

bool controlRunning       = false;                        //indicates if control is running
bool testRunning          = false;                        //indicates if a test is running
bool voltageControlActive = false;                        //status to indicate if the temperature of voltage can be controlled
double startTime          = 0;                            //stores the start time of control or test for data logging

bool buttonPressed       = false;                         //stores if any button is pressed
double timeButtonPressed = 0;                             //stores the timestamp if any button is pressed for the first time

struct controlData dataControl = {};                      //store all information of the control tab that can be updated
struct testsData dataTests     = {};                      //store all information of the tests tab that can be updated
struct statusData dataStatus   = {};                      //store all information of the status tab that can be updated
struct commandData dataCommand = {};                      //store all information of the control tab that can be updated

//used functions, describe in more detail at the end of the code
void changeValue(bool increase, bool decrease, double step, float* value, float maxValue, float minValue);
void checkButtonPressed();
float calculateTemperature(uint16_t RTDraw, float RTDnominal, float refResistor);
void printData();
void printHeader();

/**************************************************************************/
/*!
    @brief Compute task, used to update all values and button presses. 
    (running on core 1)
*/
/**************************************************************************/
void taskcompute_loop(void * parameter) {
  TickType_t xLastWakeTime;                   //last wake up time of the task
  TickType_t xDelayTime = pdMS_TO_TICKS(100); //delay time of the task

  xLastWakeTime = xTaskGetTickCount();        //get the current wakeup time

  bool MeasurementH2Sensor1Running = false; //indicates if the H2 Sensor 1 measurement is on
  bool MeasurementH2Sensor2Running = false; //indicates if the H2 Sensor 2 measurement is on
  uint8_t tokenControl = 0;                 //round token to not update specific value every round in control tab   
  uint8_t tokenGlobal  = 0;                 //round token to not update specific value every round while data logging                       

  while(true) {
    vTaskDelayUntil(&xLastWakeTime, xDelayTime);  //delay the task regarding its last wake up dime and total delay time

    checkButtonPressed();                         //check if any button is pressed for the first time

    //turn on data logging if control or test is running and update needed values
    if (controlRunning || testRunning) {

      //read the RTD and calculate the current resistance
      RTDraw = pt1000.readRTD();
      currentResistance = double(RTDraw) / 32768;
      currentResistance *= refResistance;

      //update the value only if it has the token
        if (tokenGlobal == 0) {
          xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
            dataControl.valueChipTemperature = peltierDriver.readAnalogOutput(ANALOG_OUTPUT::TEMPERATURE);
          xSemaphoreGive(SemaphoreDataControl);
          tokenGlobal = 1;
        } else if (tokenGlobal == 1) {
          xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
            dataControl.valueOutputVoltage = peltierDriver.readAnalogOutput(ANALOG_OUTPUT::VOLTAGE);
          xSemaphoreGive(SemaphoreDataControl);
          tokenGlobal = 2;
        } else if (tokenGlobal == 2) {
          xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
            dataControl.valueOutputCurrent = peltierDriver.readAnalogOutput(ANALOG_OUTPUT::CURRENT);
          xSemaphoreGive(SemaphoreDataControl);
          tokenGlobal = 0;
        } else {
          tokenGlobal = 0;
        }

        //Measure H2 Sensor 1
        if (MeasurementH2Sensor2Running) {
          double voltage = analogRead(H2SENSOR1);
          voltage /= 1000;
          xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
            dataControl.valueMeasureH2Sensor1 = voltage;
          xSemaphoreGive(SemaphoreDataControl);
        } else {
          xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
            dataControl.valueMeasureH2Sensor1 = 0.0;
          xSemaphoreGive(SemaphoreDataControl);
        }

        //Measure H2 Sensor 2
        if (MeasurementH2Sensor2Running) {
          double voltage = analogRead(H2SENSOR2);
          voltage /= 1000;
          xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
            dataControl.valueMeasureH2Sensor2 = voltage;
          xSemaphoreGive(SemaphoreDataControl);
        } else {
          xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
            dataControl.valueMeasureH2Sensor2 = 0.0;
          xSemaphoreGive(SemaphoreDataControl);
        }
    }

    //trun off the peltier driver an control or test running if an communication error occurred
    if (dataControl.eventCommunicationError) {
      display.resetControlRunning();
      display.resetTestRunning();
      controlRunning = false;
      testRunning = false;
      xSemaphoreTake(SemaphoreDataTests, portMAX_DELAY);
        dataTests.infoInformation = 4;
      xSemaphoreGive(SemaphoreDataTests);
      peltierDriver.powerOff();
    }
    
    //only update the buttons, values, events and infos for the active tab 
    if (display.getTab() == PRESSED_TAB::CONTROL) {
      //update the nominal resistance
      xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
        dataControl.valueNominalResistance  = nominalResistance;
      xSemaphoreGive(SemaphoreDataControl);

      // update the set value regarding voltage control active
      if (voltageControlActive) {
        xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
          dataControl.valueSetValue = Output;
        xSemaphoreGive(SemaphoreDataControl);
      } else {
        xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
          dataControl.valueSetValue = Setpoint;
        xSemaphoreGive(SemaphoreDataControl);
      }
        
      //update the values regarding the state of controlRunning and testRunning (only temperature needed if control or test is running)
      if (controlRunning || testRunning) {
        xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
          dataControl.valueCurrentTemperature = calculateTemperature(RTDraw, nominalResistance, refResistance);
        xSemaphoreGive(SemaphoreDataControl);
      } else {
        if (tokenControl == 0) {
          xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
            dataControl.valueChipTemperature = peltierDriver.readAnalogOutput(ANALOG_OUTPUT::TEMPERATURE);
          xSemaphoreGive(SemaphoreDataControl);
          tokenControl = 1;
        } else if (tokenControl == 1) {
          xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
            dataControl.valueOutputVoltage = peltierDriver.readAnalogOutput(ANALOG_OUTPUT::VOLTAGE);
          xSemaphoreGive(SemaphoreDataControl);
          tokenControl = 2;
        } else if (tokenControl == 2) {
          xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
            dataControl.valueOutputCurrent = peltierDriver.readAnalogOutput(ANALOG_OUTPUT::CURRENT);
          xSemaphoreGive(SemaphoreDataControl);
          tokenControl = 3;
        } else if (tokenControl == 3) {
          xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
            dataControl.valueCurrentTemperature = calculateTemperature(pt1000.readRTD(), nominalResistance, refResistance);
          xSemaphoreGive(SemaphoreDataControl);
          tokenControl = 0;
        } else {
          tokenControl = 0;
        }

        //Measure H2 Sensor 1
        if (MeasurementH2Sensor2Running) {
          double voltage = analogRead(H2SENSOR1);
          voltage /= 1000;
          xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
            dataControl.valueMeasureH2Sensor1 = voltage;
          xSemaphoreGive(SemaphoreDataControl);
        } else {
          xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
            dataControl.valueMeasureH2Sensor1 = 0.0;
          xSemaphoreGive(SemaphoreDataControl);
        }

        //Measure H2 Sensor 1
        if (MeasurementH2Sensor2Running) {
          double voltage = analogRead(H2SENSOR2);
          voltage /= 1000;
          xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
            dataControl.valueMeasureH2Sensor2 = voltage;
          xSemaphoreGive(SemaphoreDataControl);
        } else {
          xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
            dataControl.valueMeasureH2Sensor2 = 0.0;
          xSemaphoreGive(SemaphoreDataControl);
        }
      }

      //behaviour if the activate voltage control button is pressed
      if (dataControl.buttonVoltageControlPressed && !dataControl.buttonVoltageControlPreviouslyPressed) {
        voltageControlActive = !voltageControlActive;

        if (voltageControlActive) {
          startValue   = 3;
          endValue     = -2;
          riseTime     = 60;
          fallTime     = 60;
          riseStepSize = 0.1;
          fallStepSize = 0.1;
        } else {
          startValue   = 0;
          endValue     = 60;
          riseTime     = 30;
          fallTime     = 30;
          riseStepSize = 0.5;
          fallStepSize = 0.5;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
      }

      //update the H2 sensor 1 button state
      if(dataControl.buttonMeasureH2Sensor1StartPressed) {
        MeasurementH2Sensor1Running = true;
      } else if (dataControl.buttonMeasureH2Sensor1StopPressed) {
        MeasurementH2Sensor1Running = false;
      }

      //update the H2 sensor 2 button state
      if(dataControl.buttonMeasureH2Sensor2StartPressed) {
        MeasurementH2Sensor2Running = true;
      } else if (dataControl.buttonMeasureH2Sensor2StopPressed) {
        MeasurementH2Sensor2Running = false;
      }
    } else if (display.getTab() == PRESSED_TAB::TESTS) {

      //update test values
      xSemaphoreTake(SemaphoreDataTests, portMAX_DELAY);
        dataTests.valueStartValue   = startValue;
        dataTests.valueEndValue     = endValue;
        dataTests.valueRiseTime     = riseTime;
        dataTests.valueFallTime     = fallTime;
        dataTests.valueRiseStepSize = riseStepSize;
        dataTests.valueFallStepSize = fallStepSize;
        dataTests.valueNumberCycles = numberCycles;
      xSemaphoreGive(SemaphoreDataTests);
    } else if (display.getTab() == PRESSED_TAB::STATUS) {
      uint16_t status = peltierDriver.getStatus();

      //update the status values
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

      //update the command values
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

/**************************************************************************/
/*!
    @brief Display task, used to update the selected tab.
    (running on core 1)
*/
/**************************************************************************/
void taskdisplay_loop(void * parameter) {
  TickType_t xLastWakeTime;                   //last wake up time of the task
  TickType_t xDelayTime = pdMS_TO_TICKS(50); //delay time of the task

  xLastWakeTime = xTaskGetTickCount();        //get the current wakeup time

  while(true) {
    vTaskDelayUntil(&xLastWakeTime, xDelayTime); //delay the task regarding its last wake up dime and total delay time

    display.drawTabSelect();                      //redraw the tab background and tests if a new one is selected

    //only draw the tab that is active
    if (display.getTab() == PRESSED_TAB::CONTROL) {
      //change the set value on button press regarding the state of voltage control active
      if (voltageControlActive) {
        changeValue(dataControl.buttonIncreaseSetValuePressed, dataControl.buttonDecreaseSetValuePressed, 0.1, &Output, -4, 4);
      } else {
        xSemaphoreTake(SemaphoreDataPID, portMAX_DELAY);
          changeValue(dataControl.buttonIncreaseSetValuePressed, dataControl.buttonDecreaseSetValuePressed, 0.5, &Setpoint, -15, 100);
        xSemaphoreGive(SemaphoreDataPID);
      }

      //change R0 on button press
      changeValue(dataControl.buttonIncreaseNominalResistancePressed, dataControl.buttonDecreaseNominalResistancePressed, 0.1, &nominalResistance, 600, 1200);

      //behaviour for start, stop and reset button pressed
      if (dataControl.buttonStartPressed && !dataTests.buttonStartTestPreviouslyPressed && !controlRunning && !dataControl.eventCommunicationError) {
        controlRunning = true;
        startTime = millis();
        startTime /= 1000;
        peltierDriver.softStart();                                            //softstart of the LT8722 (resets all registers)
        peltierDriver.setPositiveVoltageLimit(VOLTAGE_LIMIT::LIMIT_5_00);     //set the positive voltage limit to 5V
        peltierDriver.setNegativeVoltageLimit(VOLTAGE_LIMIT::LIMIT_5_00);     //set the negative voltage limit to -5V
        peltierDriver.setPositiveCurrentLimit(4.3);                           //set the positive current limit to 4.5A
        peltierDriver.setNegativeCurrentLimit(4.3);                           //set the negative current limit to -4.5A
        printHeader();
      } else if (dataControl.buttonStopPressed) {
        controlRunning = false;
        peltierDriver.powerOff();
      } else if (dataControl.buttonResetPressed) {
        controlRunning = false;
        peltierDriver.reset();
        dataControl.eventCommunicationError = false;
        xSemaphoreTake(SemaphoreDataTests, portMAX_DELAY);
          dataTests.infoInformation = 0;
        xSemaphoreGive(SemaphoreDataTests);
      }

      //draw the control tab and update button presses
      xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
      dataControl = display.drawControlTab(dataControl);
      xSemaphoreGive(SemaphoreDataControl);
    } else if (display.getTab() == PRESSED_TAB::TESTS) {
      //change test values on button press
      if (voltageControlActive) {
        changeValue(dataTests.buttonIncreaseStartValuePressed  , dataTests.buttonDecreaseStartValuePressed  , 0.1, &startValue  , endValue, 4);
        changeValue(dataTests.buttonIncreaseEndValuePressed    , dataTests.buttonDecreaseEndValuePressed    , 0.1, &endValue    ,       -4, startValue);
        changeValue(dataTests.buttonIncreaseRiseTimePressed    , dataTests.buttonDecreaseRiseTimePressed    , 0.5, &riseTime    ,      0.5, 999999);
        changeValue(dataTests.buttonIncreaseFallTimePressed    , dataTests.buttonDecreaseFallTimePressed    , 0.5, &fallTime    ,      0.5, 999999);
        changeValue(dataTests.buttonIncreaseRiseStepSizePressed, dataTests.buttonDecreaseRiseStepSizePressed, 0.1, &riseStepSize,      0.1, (-endValue+startValue));
        changeValue(dataTests.buttonIncreaseFallStepSizePressed, dataTests.buttonDecreaseFallStepSizePressed, 0.1, &fallStepSize,      0.1, (-endValue+startValue));
        changeValue(dataTests.buttonIncreaseNumberCyclesPressed, dataTests.buttonDecreaseNumberCyclesPressed,   1, &numberCycles,        1, 999999);
      } else {
        changeValue(dataTests.buttonIncreaseStartValuePressed  , dataTests.buttonDecreaseStartValuePressed  , 0.5, &startValue  ,        -15, endValue);
        changeValue(dataTests.buttonIncreaseEndValuePressed    , dataTests.buttonDecreaseEndValuePressed    , 0.5, &endValue    , startValue, 100);
        changeValue(dataTests.buttonIncreaseRiseTimePressed    , dataTests.buttonDecreaseRiseTimePressed    , 0.5, &riseTime    ,        0.5, 999999);
        changeValue(dataTests.buttonIncreaseFallTimePressed    , dataTests.buttonDecreaseFallTimePressed    , 0.5, &fallTime    ,        0.5, 999999);
        changeValue(dataTests.buttonIncreaseRiseStepSizePressed, dataTests.buttonDecreaseRiseStepSizePressed, 0.1, &riseStepSize,        0.5, (endValue-startValue));
        changeValue(dataTests.buttonIncreaseFallStepSizePressed, dataTests.buttonDecreaseFallStepSizePressed, 0.1, &fallStepSize,        0.5, (endValue-startValue));
        changeValue(dataTests.buttonIncreaseNumberCyclesPressed, dataTests.buttonDecreaseNumberCyclesPressed,   1, &numberCycles,          1, 999999);
      }

      //behaviour for start, stop and reset button pressed
      if (dataTests.buttonStartTestPressed && !dataTests.buttonStartTestPreviouslyPressed && !testRunning) {
        testRunning = true;
        startTime = millis();
        startTime /= 1000;
        peltierDriver.softStart();                                            //softstart of the LT8722 (resets all registers)
        peltierDriver.setPositiveVoltageLimit(VOLTAGE_LIMIT::LIMIT_5_00);     //set the positive voltage limit to 5V
        peltierDriver.setNegativeVoltageLimit(VOLTAGE_LIMIT::LIMIT_5_00);     //set the negative voltage limit to -5V
        peltierDriver.setPositiveCurrentLimit(4.2);                           //set the positive current limit to 4.5A
        peltierDriver.setNegativeCurrentLimit(4.2);                           //set the negative current limit to -4.5A
        xSemaphoreTake(SemaphoreDataTests, portMAX_DELAY);
          dataTests.infoInformation = 1;
        xSemaphoreGive(SemaphoreDataTests);
        printHeader();
      } else if (dataTests.buttonStopTestPressed) {
        testRunning = false;
        peltierDriver.powerOff();                                             //turn output of LT8722 off
        xSemaphoreTake(SemaphoreDataTests, portMAX_DELAY);
          dataTests.infoInformation = 0;
        xSemaphoreGive(SemaphoreDataTests);
        xSemaphoreTake(SemaphoreDataPID, portMAX_DELAY);
          Setpoint = 10;
          Output = 0; 
        xSemaphoreGive(SemaphoreDataPID);
      } else if (dataTests.buttonResetTestPressed) {
        testRunning = false;                                                                                        
        peltierDriver.reset();                                                 //reset all registers of LT8722
        xSemaphoreTake(SemaphoreDataTests, portMAX_DELAY);
          dataTests.infoInformation = 0;
        xSemaphoreGive(SemaphoreDataTests);
        xSemaphoreTake(SemaphoreDataPID, portMAX_DELAY);
          Setpoint = 10;
          Output = 0;
        xSemaphoreGive(SemaphoreDataPID);
        dataControl.eventCommunicationError = false;

        if (voltageControlActive) {
          startValue   = 3;
          endValue     = -2;
          riseTime     = 60;
          fallTime     = 60;
          riseStepSize = 0.1;
          fallStepSize = 0.1;
        } else {
          startValue   = 0;
          endValue     = 60;
          riseTime     = 30;
          fallTime     = 30;
          riseStepSize = 0.5;
          fallStepSize = 0.5;
        }
      }

      //draw the tests tab and update button presses
      xSemaphoreTake(SemaphoreDataTests, portMAX_DELAY);
      dataTests = display.drawTestsTab(dataTests);
      xSemaphoreGive(SemaphoreDataTests);
    } else if (display.getTab() == PRESSED_TAB::STATUS) {
      //draw the status tab
      xSemaphoreTake(SemaphoreDataStatus, portMAX_DELAY);
      display.drawStatusTab(dataStatus);
      xSemaphoreGive(SemaphoreDataStatus);
    } else if (display.getTab() == PRESSED_TAB::COMMAND) {
      //draw the command tab
      xSemaphoreTake(SemaphoreDataCommand, portMAX_DELAY);
      display.drawCommandTab(dataCommand);
      xSemaphoreGive(SemaphoreDataCommand);
    }
  }
  vTaskDelete( NULL );
}

/**************************************************************************/
/*!
    @brief PIDy task, used to calculate the output value regarding 
    the changing input and setpoint.
    (running on core 1)
*/
/**************************************************************************/
void taskpid_loop(void * parameter) {
  TickType_t xLastWakeTime;                   //last wake up time of the task
  TickType_t xDelayTime = pdMS_TO_TICKS(100); //delay time of the task

  xLastWakeTime = xTaskGetTickCount();        //get the current wakeup time

  while(true) {
    vTaskDelayUntil(&xLastWakeTime, xDelayTime); //delay the task regarding its last wake up dime and total delay time

    //calculate the output value if control or task is running
    if (controlRunning || testRunning) {
      if (voltageControlActive) {
        xSemaphoreTake(SemaphoreDataPID, portMAX_DELAY);
          Input = calculateTemperature(RTDraw, nominalResistance, refResistance);
        xSemaphoreGive(SemaphoreDataPID);
      } else {
        xSemaphoreTake(SemaphoreDataPID, portMAX_DELAY);
          Input = calculateTemperature(RTDraw, nominalResistance, refResistance);
          peltierPID.Compute();
        xSemaphoreGive(SemaphoreDataPID);
      }

      //check for a communication error
      bool error = false;
      error = peltierDriver.setVoltage(Output);

      //mark error if communication with peltier driver or pt1000 it faulty
      if (error) {
        dataControl.eventCommunicationError = true;
      }

      xDelayTime = pdMS_TO_TICKS(100);
    } else {
      xDelayTime = pdMS_TO_TICKS(500);
    }
  }
  vTaskDelete( NULL );
}

/**************************************************************************/
/*!
    @brief Test task, used to update the selected tab.
    (running on core 0 as it is time critical)
*/
/**************************************************************************/
void tasktest_loop(void * parameter) {
  TickType_t xLastWakeTime;                   //last wake up time of the task
  TickType_t xDelayTime = pdMS_TO_TICKS(500); //delay time of the task

  xLastWakeTime = xTaskGetTickCount();        //get the current wakeup time

  uint16_t currentCycle    = 0;  //stores the current test cycle (test round)
  double currentSetValue   = 0;  //stores the current set temperature
  double increaseSetValue  = 0;  //stores the rise sep size
  double decreaseSetValue  = 0;  //stores the fall set size
  double delayTimeRise     = 0;  //store the rise delay time
  double delayTimeFall     = 0;  //Stores the fall delay time
  double TimeRise          = 0;  //stores the complete rise time for one cycle
  double TimeFall          = 0;  //stores the complete fall time for one cycle
  double numberStepsRise   = 0;  //stores the number of rise steps
  double numberStepsFall   = 0;  //stores the number of fall steps
  double counterStepsRise  = 0;  //counts the completed rise steps
  double counterStepsFall  = 0;  //counts the completed fall steps
  double counterTotalSteps = 0;  //counts the total number of steps

  bool testRunningFirst = true;  //indicates if the start test button is pressed for the first time
  while(true) {
    vTaskDelayUntil(&xLastWakeTime, xDelayTime); //delay the task regarding its last wake up dime and total delay time

    //execute if test is running
    if (testRunning) {

      //execute if test is started for the first time
      if (testRunningFirst) {

        //set all needed variables depending on which value is controlled regarding voltage control active
        if (voltageControlActive) {
          currentCycle      = 1;
          currentSetValue   = startValue;
          increaseSetValue  = riseStepSize;
          decreaseSetValue  = fallStepSize;
          delayTimeRise     = riseTime * 1000 / ((-endValue + startValue) / riseStepSize);
          delayTimeFall     = fallTime * 1000 / ((-endValue + startValue) / fallStepSize);
          TimeRise          = riseTime * 1000;
          TimeFall          = fallTime * 1000;
          numberStepsRise   = (-endValue + startValue) / riseStepSize;
          numberStepsFall   = (-endValue + startValue) / fallStepSize;
          counterStepsRise  = 0;
          counterStepsFall  = 0;
          counterTotalSteps = 0;

          Output = currentSetValue;
        } else {
          currentCycle      = 1;
          currentSetValue   = startValue;
          increaseSetValue  = riseStepSize;
          decreaseSetValue  = fallStepSize;
          delayTimeRise     = riseTime * 1000 / ((endValue - startValue) / riseStepSize);
          delayTimeFall     = fallTime * 1000 / ((endValue - startValue) / fallStepSize);
          TimeRise          = riseTime * 1000;
          TimeFall          = fallTime * 1000;
          numberStepsRise   = (endValue - startValue) / riseStepSize;
          numberStepsFall   = (endValue - startValue) / fallStepSize;
          counterStepsRise  = 0;
          counterStepsFall  = 0;
          counterTotalSteps = 0;

          Setpoint = currentSetValue;
        }
        
        //mark the test as already running
        testRunningFirst = false;

        //delay task for 20s to reach the setpoint
        xDelayTime = pdMS_TO_TICKS(20000);

        //check for an incorrect data input
        if((voltageControlActive && (endValue > startValue)) || (!voltageControlActive && (endValue < startValue))) {
          testRunning = false;
          testRunningFirst = true;
          peltierDriver.powerOff();
          xSemaphoreTake(SemaphoreDataTests, portMAX_DELAY);
            dataTests.infoInformation = 3;
          xSemaphoreGive(SemaphoreDataTests);
          xSemaphoreTake(SemaphoreDataPID, portMAX_DELAY);
            Setpoint = 10;
            Output = 0;
          xSemaphoreGive(SemaphoreDataPID);
          display.resetTestRunning();
          xDelayTime = pdMS_TO_TICKS(100);
        }
        
        //check for an incorrect data input
        if (delayTimeRise < 100 || delayTimeFall < 100) {
          testRunning = false;
          testRunningFirst = true;
          peltierDriver.powerOff();
          xSemaphoreTake(SemaphoreDataTests, portMAX_DELAY);
            dataTests.infoInformation = 2;
          xSemaphoreGive(SemaphoreDataTests);
          xSemaphoreTake(SemaphoreDataPID, portMAX_DELAY);
            Setpoint = 10;
            Output = 0;
          xSemaphoreGive(SemaphoreDataPID);
          xDelayTime = pdMS_TO_TICKS(100);
        }
      } else {
        if (counterStepsRise < numberStepsRise) {         //execute for rising setpoint
          counterStepsRise++;
          if (voltageControlActive) {
            Output -= increaseSetValue;
          } else {
            xSemaphoreTake(SemaphoreDataPID, portMAX_DELAY);
              Setpoint += increaseSetValue;
            xSemaphoreGive(SemaphoreDataPID);
          }
          xDelayTime = pdMS_TO_TICKS(delayTimeRise);
        } else if (counterStepsFall < numberStepsFall) {  //execute for falling setpoint
          counterStepsFall++;
          if (voltageControlActive) {
            Output += decreaseSetValue;
          } else {
            xSemaphoreTake(SemaphoreDataPID, portMAX_DELAY);
              Setpoint -= decreaseSetValue;
            xSemaphoreGive(SemaphoreDataPID);
          }
          xDelayTime = pdMS_TO_TICKS(delayTimeFall);
        } else {                                          //execute if all rising steps and falling step are done
          //check if test is finished (all test cycles executed)
          if (currentCycle == numberCycles) {
            xSemaphoreTake(SemaphoreDataTests, portMAX_DELAY);
            dataTests.infoInformation = 0;
            xSemaphoreGive(SemaphoreDataTests);
            testRunning = false;
            vTaskDelay(pdMS_TO_TICKS(500));
            peltierDriver.powerOff();
            xSemaphoreTake(SemaphoreDataPID, portMAX_DELAY);
              Setpoint = 10;
              Output = 0;
            xSemaphoreGive(SemaphoreDataPID);
            display.resetTestRunning();
          }
          currentCycle += 1;      //update the current cycle
          counterStepsRise = 0;   //reset the rise counter
          counterStepsFall = 0;   //reset the fall counter
          xDelayTime = pdMS_TO_TICKS(500);
        }
        counterTotalSteps++;
        xSemaphoreTake(SemaphoreDataTests, portMAX_DELAY);
        dataTests.valueProgress = (counterTotalSteps / ((numberStepsRise + numberStepsFall) * numberCycles)) * 100;
        xSemaphoreGive(SemaphoreDataTests);      
      }
    } else {          //reset all test values
      xDelayTime = pdMS_TO_TICKS(500);

      currentCycle      = 0;
      currentSetValue   = 0;
      increaseSetValue  = 0;
      decreaseSetValue  = 0;
      delayTimeRise     = 0;
      delayTimeFall     = 0;
      TimeRise          = 0;
      TimeFall          = 0;
      numberStepsRise   = 0;
      numberStepsFall   = 0;
      counterStepsRise  = 0;
      counterStepsFall  = 0;
      counterTotalSteps = 0;

      testRunningFirst = true;

      dataTests.valueProgress = 0;
    }
  }
  vTaskDelete( NULL );
}

/**************************************************************************/
/*!
    @brief Logging task, used to serial print the data to be logged.
    (running on core 0 as it is time critical)
*/
/**************************************************************************/
void tasklogging_loop(void * parameter) {
  TickType_t xLastWakeTime;                   //last wake up time of the task
  TickType_t xDelayTime = pdMS_TO_TICKS(250); //delay time of the task

  xLastWakeTime = xTaskGetTickCount();        //get the current wakeup time
  while(true) {
    vTaskDelayUntil(&xLastWakeTime, xDelayTime); //delay the task regarding its last wake up dime and total delay time

    //execute if test is running
    if (controlRunning || testRunning) {
      printData();  //print the measurement data
    }
  }
  vTaskDelete( NULL );
}

/**************************************************************************/
/*!
    @brief Setup: initialising the serial port, tft display, pt1000, 
    peltier driver and starting all tasks.
    (running on core 1)
*/
/**************************************************************************/
void setup() {
  Serial.begin(115200);
  delay(2000);

  display.begin();                                                      //initialize the tft display
  display.drawTabSelect(true);                                          //draw the tab for the fist time

  pt1000.begin(MAX31865_2WIRE);                                         //initialize pt1000 in 2 wire mode
  
  peltierDriver.begin();                                                //initialize the SPI interface with the standard pins
  
  peltierPID.SetTunings(Kp, Ki, Kd);                                    //apply PID gains
  peltierPID.SetOutputLimits(-4.3, 4.3);                                //set min and max output limits in V
  peltierPID.SetControllerDirection(peltierPID.Action::reverse);        //set PID to reverse mode
  peltierPID.SetMode(peltierPID.Control::automatic);                    //turn the PID on

  //semaphores to make the data exchange between the tasks possible
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
    3,
    &TaskCompute,
    1);

  xTaskCreatePinnedToCore(
    taskdisplay_loop,
    "TaskDisplay",
    50000,
    NULL,
    4,
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

  xTaskCreatePinnedToCore(
    tasktest_loop,
    "TaskTest",
    10000,
    NULL,
    50,
    &TaskTest,
    0);

  xTaskCreatePinnedToCore(
    tasklogging_loop,
    "TaskLogging",
    10000,
    NULL,
    2,
    &TaskLogging,
    0);
}

/**************************************************************************/
/*!
    @brief Loop: nothing to do as different tasks are used.
    (running on core 1)
*/
/**************************************************************************/
void loop() { 
}

/**************************************************************************/
/*!
    @brief Change a specified value with a button press.
    @param increase true if the increase button is pressed
    @param decrease true if the decrease button is pressed
    @param step stepsize for increase of decrease
    @param value value to be changed
    @param minValue minimal possible value
    @param maxValue maximal possible value
*/
/**************************************************************************/
void changeValue(bool increase, bool decrease, double step, float* value, float minValue, float maxValue) {
  if (buttonPressed) {
    if (increase) {
      if (*value < maxValue) {
        if ((millis() - timeButtonPressed) < 5000) {
          *value = *value + step;
        } else {
          *value = *value + step*10;
        }

        if (*value > maxValue) {
          *value = maxValue;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
      }
    } else if (decrease) {
      if (*value > minValue) {
        if ((millis() - timeButtonPressed) < 5000) {
          *value = *value - step;
        } else {
          *value = *value - step*10;
        }

        if (*value < minValue) {
          *value = minValue;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
      }
    }
  }
}

/**************************************************************************/
/*!
    @brief Check if any button is pressed for the first time and store the 
    time stamp.
*/
/**************************************************************************/
void checkButtonPressed() {
  if (!dataControl.buttonIncreaseSetValuePressed && !dataControl.buttonDecreaseSetValuePressed && !dataControl.buttonIncreaseNominalResistancePressed && !dataControl.buttonDecreaseNominalResistancePressed && !dataTests.buttonIncreaseStartValuePressed && !dataTests.buttonDecreaseStartValuePressed && !dataTests.buttonIncreaseEndValuePressed && !dataTests.buttonDecreaseEndValuePressed && !dataTests.buttonIncreaseRiseTimePressed && !dataTests.buttonDecreaseRiseTimePressed && !dataTests.buttonIncreaseFallTimePressed && !dataTests.buttonDecreaseFallTimePressed && !dataTests.buttonIncreaseRiseStepSizePressed && !dataTests.buttonDecreaseRiseStepSizePressed && !dataTests.buttonIncreaseFallStepSizePressed && !dataTests.buttonDecreaseFallStepSizePressed && !dataTests.buttonIncreaseNumberCyclesPressed && !dataTests.buttonDecreaseNumberCyclesPressed) {
    buttonPressed = false;
  } else {
    if (!buttonPressed) {
      timeButtonPressed = millis();
      buttonPressed = true;
    }
  }
}

/**************************************************************************/
/*!
    @brief This function is copied from the Adafruit MAX31865 library 
    to change the calculation as needed. 
    Calculate the temperature in C from the RTD through calculation of
    the resistance. 
    Uses http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf
    technique.
    @param RTDraw The raw 16-bit value from the RTD_REG
    @param RTDnominal The 'nominal' resistance of the RTD sensor, usually 100
    or 1000
    @param refResistor The value of the matching reference resistor, usually
    430 or 4300
    @returns Temperature in C
*/
/**************************************************************************/
float calculateTemperature(uint16_t RTDraw, float RTDnominal, float refResistor) {
  float Z1, Z2, Z3, Z4, Rt, temp;

  Rt = RTDraw;
  Rt /= 32768;
  Rt *= refResistor;

  // Serial.print("\nResistance: "); Serial.println(Rt, 8);

  Z1 = -RTD_A;
  Z2 = RTD_A * RTD_A - (4 * RTD_B);
  Z3 = (4 * RTD_B) / RTDnominal;
  Z4 = 2 * RTD_B;

  temp = Z2 + (Z3 * Rt);
  temp = (sqrt(temp) + Z1) / Z4;

  //if (temp >= 0)
  //  return temp;

  // ugh.
  //Rt /= RTDnominal;
  //Rt *= 100; // normalize to 100 ohm

  //float rpoly = Rt;

  //temp = -242.02;
  //temp += 2.2228 * rpoly;
  //rpoly *= Rt; // square
  //temp += 2.5859e-3 * rpoly;
  //rpoly *= Rt; // ^3
  //temp -= 4.8260e-6 * rpoly;
  //rpoly *= Rt; // ^4
  //temp -= 2.8183e-8 * rpoly;
  //rpoly *= Rt; // ^5
  //temp += 1.5243e-10 * rpoly;

  return temp;
}

/**************************************************************************/
/*!
    @brief Print all important data for data logging.
*/
/**************************************************************************/
void printData() {
  double currentTime = millis();
  currentTime /= 1000;

  if (voltageControlActive) {
    Serial.print(currentTime - startTime, 4);           Serial.print(", ");
    Serial.print(Input, 4);                             Serial.print(", ");
    Serial.print(currentResistance, 4);                 Serial.print(", ");
    Serial.print(Output, 4);                            Serial.print(", ");
    Serial.print(dataControl.valueMeasureH2Sensor1, 4); Serial.print(", ");
    Serial.print(dataControl.valueMeasureH2Sensor2, 4); Serial.print(", ");
    Serial.print(dataControl.valueOutputVoltage, 2);    Serial.print(", ");
    Serial.print(dataControl.valueOutputCurrent, 2);    Serial.print(", ");
    Serial.println(dataControl.valueChipTemperature, 2);
  } else {
    Serial.print(currentTime - startTime, 4);           Serial.print(", ");
    Serial.print(Input, 4);                             Serial.print(", ");
    Serial.print(currentResistance, 4);                        Serial.print(", ");
    Serial.print(Setpoint, 1);                          Serial.print(", ");
    Serial.print(Output, 4);                            Serial.print(", ");
    Serial.print(dataControl.valueMeasureH2Sensor1, 4); Serial.print(", ");
    Serial.print(dataControl.valueMeasureH2Sensor2, 4); Serial.print(", ");
    Serial.print(dataControl.valueOutputVoltage, 2);    Serial.print(", ");
    Serial.print(dataControl.valueOutputCurrent, 2);    Serial.print(", ");
    Serial.println(dataControl.valueChipTemperature, 2);
  }
}

/**************************************************************************/
/*!
    @brief Print the header for data logging
*/
/**************************************************************************/
void printHeader() {
  Serial.println("----------------------------------------------------");
  Serial.println("");

  if (controlRunning) {
    Serial.println("----------------------------------------------------");
    Serial.println("Control Running:");
    Serial.println("----------------------------------------------------");
    Serial.println("");
    Serial.print("Voltage Control Active:"); Serial.println(voltageControlActive);
    Serial.println("");
    Serial.println("----------------------------------------------------");
  } else if (testRunning) {
    Serial.println("----------------------------------------------------");
    Serial.println("Test Running:");
    Serial.println("----------------------------------------------------");
    Serial.println("");
    Serial.print("Voltage Control Active:"); Serial.println(voltageControlActive);
    Serial.println("");
    if (voltageControlActive) {
      Serial.print("Start Voltage:     "); Serial.println(startValue, 2);
      Serial.print("End Voltage:       "); Serial.println(endValue, 2);
    } else {
      Serial.print("Start Temperature: "); Serial.println(startValue, 2);
      Serial.print("End Temperature:   "); Serial.println(endValue, 2);
    }
    Serial.print("Rise Time:         "); Serial.println(riseTime, 2);
    Serial.print("Fall Time:         "); Serial.println(fallTime, 2);
    Serial.print("Rise Step Size:    "); Serial.println(riseStepSize, 2);
    Serial.print("Fall Step Size:    "); Serial.println(fallStepSize, 2);
    Serial.print("Number Cycles:     "); Serial.println(numberCycles, 2);
    Serial.println("----------------------------------------------------");
  } else {
    Serial.println("----------------------------------------------------");
    Serial.println("ERROR:");
    Serial.println("----------------------------------------------------");
  }

  if (voltageControlActive) {
    Serial.print("Time (sec), ");
    Serial.print("Current Temperature (°C), ");
    Serial.print("Resistance PT1000 (Ohm), ");
    Serial.print("Set Output Voltage (V), ");
    Serial.print("H2 Concentration Sensor 1 (%), ");
    Serial.print("H2 Concentration Sensor 2 (%), ");
    Serial.print("Analog Voltage Output (V), ");
    Serial.print("Analog Current Output (A), ");
    Serial.println("Analog Chip Temperature (°C)");
  } else {
    Serial.print("Time (sec), ");
    Serial.print("Current Temperature (°C), ");
    Serial.print("Resistance PT1000 (Ohm), ");
    Serial.print("Set Temperature (°C), ");
    Serial.print("PID Output Voltage (V), ");
    Serial.print("H2 Concentration Sensor 1 (%), ");
    Serial.print("H2 Concentration Sensor 2 (%), ");
    Serial.print("Analog Voltage Output (V), ");
    Serial.print("Analog Current Output (A), ");
    Serial.println("Analog Chip Temperature (°C)");
  }
}
