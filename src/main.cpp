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
 * Date: 2025-01-22 Author: Jan kleine Piening Comments: Added ControlTFT class to control the display with own widgets
 * Date: 2025-01-24 Author: Jan kleine Piening Comments: added control, tests, status and command tab
 * Date: 2025-01-27 Author: Jan kleine Piening Comments: Added finalized tests tab
 * Date: 2025-01-28 Author: Jan kleine Piening Comments: Added added data logging and improved test behavior
 * Date: 2025-01-28 Author: Jan kleine Piening Comments: Added docs: added more comments
 * Date: 2025-01-29 Author: Jan kleine Piening Comments: Added docs: improved comments
 * Date: 2025-01-29 Author: Jan kleine Piening Comments: Added docs: added README
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

//taskhandle for the different tasks
TaskHandle_t TaskCompute;
TaskHandle_t TaskDisplay;
TaskHandle_t TaskPID;
TaskHandle_t TaskTest;

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
float nominalResistance   = 940;                          //the 'nominal' 0-degrees-C resistance of the sensor

float startTemperature = 0.0;                             //start temperature for the tests
float endTemperature   = 60.0;                            //end temperature for the tests
float riseTime         = 30.0;                            //rise time for the tests
float fallTime         = 30.0;                            //fall time for the tests
float riseStepSize     = 0.5;                             //rise step size for the tests
float fallStepSize     = 0.5;                             //fall step size for the tests
float numberCycles     = 5.0;                             //number of tests cycles

bool controlRunning = false;                              //indicates if control is running
bool testRunning    = false;                              //indicates if a test is running
double startTime    = 0;                                  //stores the start time of control or test for data logging

bool buttonPressed = false;                               //stores if any button is pressed
double timeButtonPressed = 0;                             //stores the timestamp if any button is pressed for the first time

struct controlData dataControl = {};                      //store all information of the control tab that can be updated
struct testsData dataTests = {};                          //store all information of the tests tab that can be updated
struct statusData dataStatus = {};                        //store all information of the status tab that can be updated
struct commandData dataCommand = {};                      //store all information of the control tab that can be updated

//used functions, describe in more detail at the end of the code
void changeValue(bool increase, bool decrease, double step, float* value, float maxValue, float minValue);
void checkButtonPressed();
void printData();
void printHeader();

/**************************************************************************/
/*!
    @brief Compute task, used to update all values and button presses. 
    (running on core 1)
*/
/**************************************************************************/
void taskcompute_loop(void * parameter) {
  bool MeasurementH2Sensor1Running = false; //indicates if the H2 Sensor 1 measurement is on
  bool MeasurementH2Sensor2Running = false; //indicates if the H2 Sensor 2 measurement is on
  uint8_t tokenControl = 0;                 //round token to not update specific value every round in control tab   
  uint8_t tokenGlobal = 0;                  //round token to not update specific value every round while data logging                       

  for( ;; ) {
    checkButtonPressed(); //check if any button is pressed for the first time

    //turn on data logging if control or test is running
    if (controlRunning || testRunning) {

      //update the value only if it has the token
      if (tokenGlobal == 0) {
        dataControl.valueChipTemperature = peltierDriver.readAnalogOutput(ANALOG_OUTPUT::TEMPERATURE);
        tokenGlobal = 1;
      } else if (tokenGlobal == 1) {
        dataControl.valueOutputVoltage = peltierDriver.readAnalogOutput(ANALOG_OUTPUT::VOLTAGE);
        tokenGlobal = 2;
      } else if (tokenGlobal == 2) {
        dataControl.valueOutputCurrent = peltierDriver.readAnalogOutput(ANALOG_OUTPUT::CURRENT);
        tokenGlobal = 0;
      } else {
        tokenGlobal = 0;
      }

      //Measure H2 Sensor 1
      if (MeasurementH2Sensor2Running) {
        double voltage = analogRead(H2SENSOR1);
        voltage /= 1000;
        dataControl.valueMeasureH2Sensor1 = voltage;
      } else {
        dataControl.valueMeasureH2Sensor1 = 0.0;
      }

       //Measure H2 Sensor 2
      if (MeasurementH2Sensor2Running) {
        double voltage = analogRead(H2SENSOR2);
        voltage /= 1000;
        dataControl.valueMeasureH2Sensor2 = voltage;
      } else {
        dataControl.valueMeasureH2Sensor2 = 0.0;
      }
      
      printData();
    }

    //trun off the peltier driver an control or test running if an communication error occurred
    if (dataControl.eventCommunicationError) {
      display.resetControlRunning();
      display.resetTestRunning();
      controlRunning = false;
      testRunning = false;
      peltierDriver.powerOff();
    }
    
    //only update the buttons, values, events and infos for the active tab 
    if (display.getTab() == PRESSED_TAB::CONTROL) {
      xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
        dataControl.valueNominalResistance  = nominalResistance;
        dataControl.valueSetTemperature     = Setpoint;

        //update the value only if it has the token
        if (tokenControl == 0) {
          dataControl.valueChipTemperature = peltierDriver.readAnalogOutput(ANALOG_OUTPUT::TEMPERATURE);
          tokenControl = 1;
        } else if (tokenControl == 1) {
          dataControl.valueOutputVoltage = peltierDriver.readAnalogOutput(ANALOG_OUTPUT::VOLTAGE);
          tokenControl = 2;
        } else if (tokenControl == 2) {
          dataControl.valueOutputCurrent = peltierDriver.readAnalogOutput(ANALOG_OUTPUT::CURRENT);
          tokenControl = 3;
        } else if (tokenControl == 3) {
          dataControl.valueCurrentTemperature = pt1000.temperature(nominalResistance, refResistance);
          tokenControl = 0;
        } else {
          tokenControl = 0;
        }
        
        if(dataControl.buttonMeasureH2Sensor1StartPressed) {
          MeasurementH2Sensor1Running = true;
        } else if (dataControl.buttonMeasureH2Sensor1StopPressed) {
          MeasurementH2Sensor1Running = false;
        }

        if(dataControl.buttonMeasureH2Sensor2StartPressed) {
          MeasurementH2Sensor2Running = true;
        } else if (dataControl.buttonMeasureH2Sensor2StopPressed) {
          MeasurementH2Sensor2Running = false;
        }

        if (MeasurementH2Sensor2Running) {
          double voltage = analogRead(H2SENSOR1);
          voltage /= 1000;
          dataControl.valueMeasureH2Sensor1 = voltage;
        } else {
          dataControl.valueMeasureH2Sensor1 = 0.0;
        }

        if (MeasurementH2Sensor2Running) {
          double voltage = analogRead(H2SENSOR2);
          voltage /= 1000;
          dataControl.valueMeasureH2Sensor2 = voltage;
        } else {
          dataControl.valueMeasureH2Sensor2 = 0.0;
        }
      xSemaphoreGive(SemaphoreDataControl);
      
      xSemaphoreTake(SemaphoreDataPID, portMAX_DELAY);
        changeValue(dataControl.buttonIncreaseSetTemperaturePressed, dataControl.buttonDecreaseSetTemperaturePressed, 0.5, &Setpoint, -5, 150);
      xSemaphoreGive(SemaphoreDataPID);

      changeValue(dataControl.buttonIncreaseNominalResistancePressed, dataControl.buttonDecreaseNominalResistancePressed, 0.1, &nominalResistance, 900, 1100);

      //behaviour for start, stop and reset button pressed
      if (dataControl.buttonStartPressed && !dataTests.buttonStartTestPreviouslyPressed && !controlRunning && !dataControl.eventCommunicationError) {
        controlRunning = true;
        startTime = millis();
        startTime /= 1000;
        peltierDriver.softStart();                                            //softstart of the LT8722 (resets all registers)
        peltierDriver.setPositiveVoltageLimit(VOLTAGE_LIMIT::LIMIT_5_00);     //set the positive voltage limit to 5V
        peltierDriver.setNegativeVoltageLimit(VOLTAGE_LIMIT::LIMIT_5_00);     //set the negative voltage limit to -5V
        peltierDriver.setPositiveCurrentLimit(4.5);                           //set the positive current limit to 4.5A
        peltierDriver.setNegativeCurrentLimit(4.5);                           //set the negative current limit to -4.5A
        printHeader();
      } else if (dataControl.buttonStopPressed) {
        controlRunning = false;
        peltierDriver.powerOff();
      } else if (dataControl.buttonResetPressed) {
        controlRunning = false;
        peltierDriver.reset();
        dataControl.eventCommunicationError = false;
      }
    } else if (display.getTab() == PRESSED_TAB::TESTS) {
      xSemaphoreTake(SemaphoreDataTests, portMAX_DELAY);
        dataTests.valueStartTemperature  = startTemperature;
        dataTests.valueEndTemperature    = endTemperature;
        dataTests.valueRiseTime          = riseTime;
        dataTests.valueFallTime          = fallTime;
        dataTests.valueRiseStepSize      = riseStepSize;
        dataTests.valueFallStepSize      = fallStepSize;
        dataTests.valueNumberCycles      = numberCycles;
      xSemaphoreGive(SemaphoreDataTests);

      changeValue(dataTests.buttonIncreaseStartTemperaturePressed, dataTests.buttonDecreaseStartTemperaturePressed, 0.5, &startTemperature,  -5, endTemperature);
      changeValue(dataTests.buttonIncreaseEndTemperaturePressed  , dataTests.buttonDecreaseEndTemperaturePressed  , 0.5, &endTemperature  ,  startTemperature, 150);
      changeValue(dataTests.buttonIncreaseRiseTimePressed        , dataTests.buttonDecreaseRiseTimePressed        , 0.5, &riseTime        , 0.5, 999999);
      changeValue(dataTests.buttonIncreaseFallTimePressed        , dataTests.buttonDecreaseFallTimePressed        , 0.5, &fallTime        , 0.5, 999999);
      changeValue(dataTests.buttonIncreaseRiseStepSizePressed    , dataTests.buttonDecreaseRiseStepSizePressed    , 0.1, &riseStepSize    , 0.5, (endTemperature-startTemperature));
      changeValue(dataTests.buttonIncreaseFallStepSizePressed    , dataTests.buttonDecreaseFallStepSizePressed    , 0.1, &fallStepSize    , 0.5, (endTemperature-startTemperature));
      changeValue(dataTests.buttonIncreaseNumberCyclesPressed    , dataTests.buttonDecreaseNumberCyclesPressed    ,   1, &numberCycles    ,   1, 999999);

      //behaviour for start, stop and reset button pressed
      if (dataTests.buttonStartTestPressed && !dataTests.buttonStartTestPreviouslyPressed && !testRunning) {
        testRunning = true;
        startTime = millis();
        startTime /= 1000;
        peltierDriver.softStart();                                            //softstart of the LT8722 (resets all registers)
        peltierDriver.setPositiveVoltageLimit(VOLTAGE_LIMIT::LIMIT_5_00);     //set the positive voltage limit to 5V
        peltierDriver.setNegativeVoltageLimit(VOLTAGE_LIMIT::LIMIT_5_00);     //set the negative voltage limit to -5V
        peltierDriver.setPositiveCurrentLimit(4.5);                           //set the positive current limit to 4.5A
        peltierDriver.setNegativeCurrentLimit(4.5);                           //set the negative current limit to -4.5A
        dataTests.infoInformation = 1;
        printHeader();
      } else if (dataTests.buttonStopTestPressed) {
        testRunning = false;
        peltierDriver.powerOff();
        dataTests.infoInformation = 0;
        xSemaphoreTake(SemaphoreDataPID, portMAX_DELAY);
          Setpoint = 10;
        xSemaphoreGive(SemaphoreDataPID);
      } else if (dataTests.buttonResetTestPressed) {
        testRunning = false;
        peltierDriver.reset();
        dataTests.infoInformation = 0;
        xSemaphoreTake(SemaphoreDataPID, portMAX_DELAY);
          Setpoint = 10;
        xSemaphoreGive(SemaphoreDataPID);
        dataControl.eventCommunicationError = false;
        startTemperature = 0.0;
        endTemperature   = 60.0;
        riseTime         = 30.0;
        fallTime         = 30.0;
        riseStepSize     = 0.5;
        fallStepSize     = 0.5;
        numberCycles     = 5.0;
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

/**************************************************************************/
/*!
    @brief Display task, used to update the selected tab.
    (running on core 1)
*/
/**************************************************************************/
void taskdisplay_loop(void * parameter) {
  for( ;; ) {
    display.drawTabSelect();

    //only draw the tab that is active
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

/**************************************************************************/
/*!
    @brief PIDy task, used to calculate the output value regarding 
    the changing input and setpoint.
    (running on core 1)
*/
/**************************************************************************/
void taskpid_loop(void * parameter) {
  for( ;; ) {

    //calculate the output value if control or task is running
    if (controlRunning || testRunning) {
      xSemaphoreTake(SemaphoreDataPID, portMAX_DELAY);
        Input = pt1000.temperature(nominalResistance, refResistance);
        peltierPID.Compute();
      xSemaphoreGive(SemaphoreDataPID);

      //check for a communication error
      bool error = false;
      error = peltierDriver.setVoltage(Output);

      //mark error if communication with peltier driver or pt1000 it faulty
      if (error || pt1000.readFault()) {
        dataControl.eventCommunicationError = true;
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(500));
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
  uint16_t currentCycle         = 0;  //stores the current test cycle (test round)
  double currentSetTemperature  = 0;  //stores the current set temperature
  double increaseSetTemperature = 0;  //stores the rise sep size
  double decreaseSetTemperature = 0;  //stores the fall set size
  double delayTimeRise          = 0;  //store the rise delay time
  double delayTimeFall          = 0;  //Stores the fall delay time
  double TimeRise               = 0;  //stores the complete rise time for one cycle
  double TimeFall               = 0;  //stores the complete fall time for one cycle
  double numberStepsRise        = 0;  //stores the number of rise steps
  double numberStepsFall        = 0;  //stores the number of fall steps
  double counterStepsRise       = 0;  //counts the completed rise steps
  double counterStepsFall       = 0;  //counts the completed fall steps
  double counterTotalSteps      = 0;  //counts the total number of steps

  bool testRunningFirst = true;       //indicates if the start test button is pressed for the first time

  TickType_t xLastWakeTime;                   //last wake up time of the task
  TickType_t xDelayTime = pdMS_TO_TICKS(500); //delay time of the task

  xLastWakeTime = xTaskGetTickCount();        //get the current wakeup time

  for( ;; ) {
    vTaskDelayUntil(&xLastWakeTime, xDelayTime);  //delay the task regarding its last wake up dime and total delay time

    //execute if test is running
    if (testRunning) {

      //execute if test is started for the first time
      if (testRunningFirst) {

        //set all needed variables
        currentCycle           = 1;
        currentSetTemperature  = startTemperature;
        increaseSetTemperature = riseStepSize;
        decreaseSetTemperature = fallStepSize;
        delayTimeRise          = riseTime * 1000 / ((endTemperature - startTemperature) / riseStepSize);
        delayTimeFall          = fallTime * 1000 / ((endTemperature - startTemperature) / fallStepSize);
        TimeRise               = riseTime * 1000;
        TimeFall               = fallTime * 1000;
        numberStepsRise        = (endTemperature - startTemperature) / riseStepSize;
        numberStepsFall        = (endTemperature - startTemperature) / fallStepSize;
        counterStepsRise       = 0;
        counterStepsFall       = 0;
        counterTotalSteps      = 0;

        Setpoint         = currentSetTemperature;

        //mark the test as already running
        testRunningFirst = false;

        //check for an correct data input
        if(endTemperature < startTemperature) {
          testRunning = false;
          peltierDriver.powerOff();
          dataTests.infoInformation = 3;
          xSemaphoreTake(SemaphoreDataPID, portMAX_DELAY);
            Setpoint = 10;
          xSemaphoreGive(SemaphoreDataPID);
          display.resetTestRunning();
        } else if (delayTimeRise < 100 || delayTimeFall < 100) {
          testRunning = false;
          peltierDriver.powerOff();
          dataTests.infoInformation = 2;
          xSemaphoreTake(SemaphoreDataPID, portMAX_DELAY);
            Setpoint = 10;
          xSemaphoreGive(SemaphoreDataPID);
          display.resetTestRunning();
        }

        //delay task for 20s to reach the setpoint
        xDelayTime = pdMS_TO_TICKS(20000);
      } else {
        if (counterStepsRise < numberStepsRise) {         //execute for rising setpoint
          counterStepsRise++;
          xSemaphoreTake(SemaphoreDataPID, portMAX_DELAY);
          Setpoint += increaseSetTemperature;
          xSemaphoreGive(SemaphoreDataPID);
          xDelayTime = pdMS_TO_TICKS(delayTimeRise);
        } else if (counterStepsFall < numberStepsFall) {  //execute for falling setpoint
          counterStepsFall++;
          xSemaphoreTake(SemaphoreDataPID, portMAX_DELAY);
          Setpoint -= decreaseSetTemperature;
          xSemaphoreGive(SemaphoreDataPID);
          xDelayTime = pdMS_TO_TICKS(delayTimeRise);
        } else {                                          //execute is all rising steps and falling step are done
          
          //check if test is finished (all test cycles executed)
          if (currentCycle == numberCycles-1) {
            testRunning = false;
            peltierDriver.powerOff();
            dataTests.infoInformation = 0;
            xSemaphoreTake(SemaphoreDataPID, portMAX_DELAY);
            Setpoint = 10;
            xSemaphoreGive(SemaphoreDataPID);
            display.resetTestRunning();
          }

          currentCycle += 1;
          counterStepsRise = 0;
          counterStepsFall = 0;
          xDelayTime = pdMS_TO_TICKS(100);
        }
        counterTotalSteps++;
        dataTests.valueProgress = (counterTotalSteps / ((numberStepsRise + numberStepsFall) * numberCycles)) * 100;
      }
    } else {
      xDelayTime = pdMS_TO_TICKS(500);

      currentCycle           = 0;
      currentSetTemperature  = 0;
      increaseSetTemperature = 0;
      decreaseSetTemperature = 0;
      delayTimeRise          = 0;
      delayTimeFall          = 0;
      TimeRise               = 0;
      TimeFall               = 0;
      numberStepsRise        = 0;
      numberStepsFall        = 0;
      counterStepsRise       = 0;
      counterStepsFall       = 0;
      counterTotalSteps      = 0;

      testRunningFirst = true;

      dataTests.valueProgress = 0;
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
  peltierPID.SetOutputLimits(-3.5, 3.5);                                //set min and max output limits in V
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
    9,
    &TaskCompute,
    1);

  xTaskCreatePinnedToCore(
    taskdisplay_loop,
    "TaskDisplay",
    50000,
    NULL,
    10,
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
    "Test",
    10000,
    NULL,
    50,
    &TaskTest,
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
  if (!dataControl.buttonIncreaseSetTemperaturePressed && !dataControl.buttonDecreaseSetTemperaturePressed && !dataControl.buttonIncreaseNominalResistancePressed && !dataControl.buttonDecreaseNominalResistancePressed && !dataTests.buttonIncreaseStartTemperaturePressed && !dataTests.buttonDecreaseStartTemperaturePressed && !dataTests.buttonIncreaseEndTemperaturePressed && !dataTests.buttonDecreaseEndTemperaturePressed && !dataTests.buttonIncreaseRiseTimePressed && !dataTests.buttonDecreaseRiseTimePressed && !dataTests.buttonIncreaseFallTimePressed && !dataTests.buttonDecreaseFallTimePressed && !dataTests.buttonIncreaseRiseStepSizePressed && !dataTests.buttonDecreaseRiseStepSizePressed && !dataTests.buttonIncreaseFallStepSizePressed && !dataTests.buttonDecreaseFallStepSizePressed && !dataTests.buttonIncreaseNumberCyclesPressed && !dataTests.buttonDecreaseNumberCyclesPressed) {
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
    @brief Print all important data for data logging.
*/
/**************************************************************************/
void printData() {
  double currentTime = millis();
  currentTime /= 1000;
  Serial.print(currentTime - startTime, 4);           Serial.print(", ");
  Serial.print(Input, 4);                             Serial.print(", ");
  Serial.print(Setpoint, 1);                          Serial.print(", ");
  Serial.print(Output, 4);                            Serial.print(", ");
  Serial.print(dataControl.valueMeasureH2Sensor1, 4); Serial.print(", ");
  Serial.print(dataControl.valueMeasureH2Sensor2, 4); Serial.print(", ");
  Serial.print(dataControl.valueOutputVoltage, 2);    Serial.print(", ");
  Serial.print(dataControl.valueOutputCurrent, 2);    Serial.print(", ");
  Serial.println(dataControl.valueChipTemperature, 2);
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
  } else if (testRunning) {
    Serial.println("----------------------------------------------------");
    Serial.println("Test Running:");
    Serial.println("----------------------------------------------------");
    Serial.print("Start Temperature:"); Serial.println(startTemperature, 2);
    Serial.print("End Temperature:  "); Serial.println(endTemperature, 2);
    Serial.print("Rise Time:        "); Serial.println(riseTime, 2);
    Serial.print("Fall Time:        "); Serial.println(fallTime, 2);
    Serial.print("Rise Step Size:   "); Serial.println(riseStepSize, 2);
    Serial.print("Fall Step Size:   "); Serial.println(fallStepSize, 2);
    Serial.print("Number Cycles:    "); Serial.println(numberCycles, 2);
    Serial.println("----------------------------------------------------");
  } else {
    Serial.println("----------------------------------------------------");
    Serial.println("ERROR:");
    Serial.println("----------------------------------------------------");
  }

  Serial.print("Time (sec), ");
  Serial.print("Current Temperature (°C), ");
  Serial.print("Set Temperature (°C), ");
  Serial.print("PID Output Voltage (V), ");
  Serial.print("H2 Concentration Sensor 1 (%), ");
  Serial.print("H2 Concentration Sensor 2 (%), ");
  Serial.print("Analog Voltage Output (V), ");
  Serial.print("Analog Current Output (A), ");
  Serial.println("Analog Chip Temperature (°C)");
}