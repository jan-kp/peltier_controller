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
TaskHandle_t TaskTest;

SemaphoreHandle_t SemaphoreDataControl;
SemaphoreHandle_t SemaphoreDataTests;
SemaphoreHandle_t SemaphoreDataStatus;
SemaphoreHandle_t SemaphoreDataCommand;
SemaphoreHandle_t SemaphoreDataPID;

LT8722 peltierDriver;                                           //create a LT8722 object with FSPI

SPIClass fspi(FSPI);
Adafruit_MAX31865 pt1000 = Adafruit_MAX31865(9, &fspi);         //create a Adafruit_MAX31865 object with FSPI

float Kp = 0.09, Ki = 0.25, Kd = 0;                             //PID tuning parameters 
float Setpoint = 10, Input, Output;                             //variables for PID      
QuickPID peltierPID(&Input, &Output, &Setpoint);                //specify PID links

ControlTFT display;

const float refResistance = 4300.0;                             //the value of the Rref resistor
float nominalResistance   = 940;                                //the 'nominal' 0-degrees-C resistance of the sensor

float startTemperature = 0.0;
float endTemperature   = 60.0;
float riseTime         = 30.0;
float fallTime         = 30.0;
float riseStepSize     = 0.5;
float fallStepSize     = 0.5;
float numberCycles     = 5.0;

bool controlRunning = false;
bool testRunning    = false;
double startTime    = 0;

bool buttonPressed = false;
double timeButtonPressed = 0;

struct controlData dataControl = {};
struct testsData dataTests = {};
struct statusData dataStatus = {};
struct commandData dataCommand = {};

void changeValue(bool increase, bool decrease, double step, float* value, float maxValue, float minValue);
void checkButtonPressed();
void printData();
void printHeader();

void taskcompute_loop(void * parameter) {
  bool MeasurementH2Sensor1Running = false;
  bool MeasurementH2Sensor2Running = false;
  uint8_t token = 0;

  for( ;; ) {
    checkButtonPressed();

    if (controlRunning || testRunning) {
      printData();
    }

    if (dataControl.eventCommunicationError) {
      display.resetControlRunning();
      display.resetTestRunning();
      controlRunning = false;
      testRunning = false;
      peltierDriver.powerOff();
    }
    

    if (display.getTab() == PRESSED_TAB::CONTROL) {
      xSemaphoreTake(SemaphoreDataControl, portMAX_DELAY);
        dataControl.valueNominalResistance  = nominalResistance;
        dataControl.valueSetTemperature     = Setpoint;

        if (token == 0) {
          dataControl.valueChipTemperature = peltierDriver.readAnalogOutput(ANALOG_OUTPUT::TEMPERATURE);
          token = 1;
        } else if (token == 1) {
          dataControl.valueOutputVoltage = peltierDriver.readAnalogOutput(ANALOG_OUTPUT::VOLTAGE);
          token = 2;
        } else if (token == 2) {
          dataControl.valueOutputCurrent = peltierDriver.readAnalogOutput(ANALOG_OUTPUT::CURRENT);
          token = 3;
        } else if (token == 3) {
          dataControl.valueCurrentTemperature = pt1000.temperature(nominalResistance, refResistance);
          token = 0;
        } else {
          token = 0;
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
    if (controlRunning || testRunning) {
      xSemaphoreTake(SemaphoreDataPID, portMAX_DELAY);
        Input = pt1000.temperature(nominalResistance, refResistance);
        peltierPID.Compute();
      xSemaphoreGive(SemaphoreDataPID);

      bool error = false;
      error = peltierDriver.setVoltage(Output);

      if (error || pt1000.readFault()) {
        dataControl.eventCommunicationError = true;
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
  vTaskDelete( NULL );
}

void tasktest_loop(void * parameter) {
  uint16_t currentCycle         = 0;
  double currentSetTemperature  = 0;
  double increaseSetTemperature = 0;
  double decreaseSetTemperature = 0;
  double delayTimeRise          = 0;
  double delayTimeFall          = 0;
  double TimeRise               = 0;
  double TimeFall               = 0;
  double numberStepsRise        = 0;
  double numberStepsFall        = 0;
  double counterStepsRise       = 0;
  double counterStepsFall       = 0;
  double counterTotalSteps      = 0;

  bool testRunningFirst = true;

  TickType_t xLastWakeTime;
  TickType_t xDelayTime = pdMS_TO_TICKS(500);

  xLastWakeTime = xTaskGetTickCount();

  for( ;; ) {
    vTaskDelayUntil(&xLastWakeTime, xDelayTime);

    if (testRunning) {
      if (testRunningFirst) {
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
        testRunningFirst = false;

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

        xDelayTime = pdMS_TO_TICKS(20000);
      } else {
        if (counterStepsRise < numberStepsRise) {
          counterStepsRise++;
          xSemaphoreTake(SemaphoreDataPID, portMAX_DELAY);
          Setpoint += increaseSetTemperature;
          xSemaphoreGive(SemaphoreDataPID);
          xDelayTime = pdMS_TO_TICKS(delayTimeRise);
        } else if (counterStepsFall < numberStepsFall) {
          counterStepsFall++;
          xSemaphoreTake(SemaphoreDataPID, portMAX_DELAY);
          Setpoint -= decreaseSetTemperature;
          xSemaphoreGive(SemaphoreDataPID);
          xDelayTime = pdMS_TO_TICKS(delayTimeRise);
        } else {
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

void setup() {
  Serial.begin(115200);
  delay(2000);

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

void loop() { 
}

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

void printData() {
  double currentTime = millis();
  currentTime /= 1000;
  Serial.print(currentTime - startTime, 4);         Serial.print(", ");
  Serial.print(controlRunning);                       Serial.print(", ");
  Serial.print(testRunning);                          Serial.print(", ");
  Serial.print(Input, 4);                             Serial.print(", ");
  Serial.print(Setpoint, 1);                          Serial.print(", ");
  Serial.print(Output, 4);                            Serial.print(", ");
  Serial.print(dataControl.valueMeasureH2Sensor1, 4); Serial.print(", ");
  Serial.print(dataControl.valueMeasureH2Sensor2, 4); Serial.println(", ");
}

void printHeader() {
  Serial.print("Time (sec), ");
  Serial.print("Control Running, ");
  Serial.print("Test Running, ");
  Serial.print("Current Temperature (°C), ");
  Serial.print("Set Temperature (°C), ");
  Serial.print("Output Voltage (V), ");
  Serial.print("H2 Concentration Sensor 1 (%), ");
  Serial.println("H2 Concentration Sensor 2 (%), ");
}