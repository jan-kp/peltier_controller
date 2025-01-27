/*
 * File Name: TFT_Control.h
 * Description: Functions to control the TFT Display.
 *
 * Notes: This code was written as part of my master's thesis at the 
 *        Institute for Microsensors, -actuators and -systems (IMSAS) 
 *        at the University of Bremen.
 */

#ifndef ControlTFT_H
#define ControlTFT_H

#include "FS.h"

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>

// This is the file name used to store the calibration data
// You can change this to create new calibration files.
// The SPIFFS file name must start with "/".
#define CALIBRATION_FILE "/TouchCalData1"

// Set REPEAT_CAL to true instead of false to run calibration
// again, otherwise it will only be done once.
#define REPEAT_CAL false

#define LIGHT_GREY 0xDEFB
#define GREY 0xC5F7
#define BLUE 0x343C
#define RED 0xE987
#define GREEN 0x2E0F
#define BLACK 0x1000

struct tab {
    int32_t startX;
    int32_t startY;
    int32_t withX;
    int32_t hightY;
    int32_t radius;
    uint32_t textColor;
    uint32_t backgroundColor;
    uint32_t selectColor;
    bool pressed;
};

struct button {
    char* name;
    int32_t startX;
    int32_t startY;
    int32_t withX;
    int32_t hightY;
    int32_t radius;
    uint32_t textColor;
    uint32_t backgroundColor;
    uint32_t selectColor;
    bool pressed;
    bool previouslyPressed;
};

struct text {
    char* name;
    int32_t startX;
    int32_t startY;
    int32_t withX;
    int32_t hightY;
    uint32_t textColor;
    uint32_t backgroundColor;
};

struct value {
    double value;
    int32_t startX;
    int32_t startY;
    int32_t withX;
    int32_t hightY;
    uint32_t textColor;
    uint32_t backgroundColor;
};

struct event {
    bool bit;
    char* noEvent;
    char* event;
    int32_t startX;
    int32_t startY;
    int32_t withX;
    int32_t hightY;
    uint32_t textColor;
    uint32_t eventColor;
    uint32_t backgroundColor;
};

struct info {
    uint8_t bits;
    int32_t startX;
    int32_t startY;
    int32_t withX;
    int32_t hightY;
    uint32_t textColor;
    uint32_t backgroundColor;
    char* info0;
    char* info1;
    char* info2;
    char* info3;
    char* info4;
    char* info5;
    char* info6;
    char* info7;
};

struct controlData {
    double valueChipTemperature;
    double valueOutputVoltage;
    double valueOutputCurrent;
    double valueMeasureH2Sensor1;
    double valueMeasureH2Sensor2;
    double valueNominalResistance;
    double valueSetTemperature;
    double valueCurrentTemperature;
    bool   buttonMeasureH2Sensor1StopPressed;
    bool   buttonMeasureH2Sensor1StartPressed;
    bool   buttonMeasureH2Sensor2StopPressed;
    bool   buttonMeasureH2Sensor2StartPressed;
    bool   buttonDecreaseNominalResistancePressed;
    bool   buttonIncreaseNominalResistancePressed;
    bool   buttonDecreaseSetTemperaturePressed;
    bool   buttonIncreaseSetTemperaturePressed;
    bool   eventCommunicationError;
    bool   buttonResetPressed;
    bool   buttonStopPressed;
    bool   buttonStartPressed;
};

struct testsData {
    double  valueStartTemperature;
    double  valueEndTemperature;
    double  valueRiseTime;
    double  valueFallTime;
    double  valueRiseStepSize;
    double  valueFallStepSize;
    double  valueNumberCycles;
    double  valueProgress;
    bool    buttonDecreaseStartTemperaturePressed;
    bool    buttonIncreaseStartTemperaturePressed;
    bool    buttonDecreaseEndTemperaturePressed;
    bool    buttonIncreaseEndTemperaturePressed;
    bool    buttonDecreaseRiseTimePressed;
    bool    buttonIncreaseRiseTimePressed;
    bool    buttonDecreaseFallTimePressed;
    bool    buttonIncreaseFallTimePressed;
    bool    buttonDecreaseRiseStepSizePressed;
    bool    buttonIncreaseRiseStepSizePressed;
    bool    buttonDecreaseFallStepSizePressed;
    bool    buttonIncreaseFallStepSizePressed;
    bool    buttonDecreaseNumberCyclesPressed;
    bool    buttonIncreaseNumberCyclesPressed;
    bool    buttonResetTestPressed;
    bool    buttonStopTestPressed;
    bool    buttonStartTestPressed;
    bool    buttonStartTestPreviouslyPressed;
    uint8_t infoInformation;
};

struct statusData {
    bool eventPWMSwitchingEvent;
    bool eventCurrentLimitEvent;
    bool eventPowerLimitEvent;
    bool eventSoftResetEvent;
    bool eventOverCurrentEvent;
    bool eventOverttemperatureEvent;
    bool eventVCCUnderVoltageEvent;
    bool eventVDDIOUnderVOltageEvent;
};

struct commandData {
    bool    eventPowerStage;
    bool    eventPWMSwitching;
    uint8_t infoPWMFrequency;
    uint8_t infoPWMAdjust;
    uint8_t infoPWMDutyCycle;
    bool    eventLDORegulation;
    uint8_t infoPeakInductorCurrent;
    uint8_t infoPowerLimit;
};

enum class PRESSED_TAB : uint8_t{
    CONTROL,
    TESTS,
    STATUS,
    COMMAND
};

class ControlTFT {
public:
    ControlTFT();
    void begin();

    void drawTabSelect(bool init = false);
    controlData drawControlTab(controlData data);
    testsData drawTestsTab(testsData data);
    void drawStatusTab(statusData data);
    void drawCommandTab(commandData data);

    PRESSED_TAB getTab();

    void resetControlRunning();
    void resetTestRunning();

private:
    void touchCalibrate();

    bool checkTouchTabs();
    bool inArea(int16_t startX, int16_t startY, int16_t withX, int16_t hightY, int16_t touchX, int16_t touchY);
    button buttonPressed(button button, uint16_t t_x, uint16_t t_y, bool pressed);
    void lockButton(button* lock, bool* task);
    void releaseButton(button action, button* release, bool* task);

    void drawButton(button button);
    void drawText(text text, uint8_t position);
    void drawValue(value value, uint8_t position);
    void drawEvent(event event);
    void drawInfo(info info);

    bool _testRunning;
    bool _controlRunning;
    bool _measurementH2Sensor1Running;
    bool _measurementH2Sensor2Running;

    struct tab _controlTab;
    struct tab _testsTab;
    struct tab _statusTab;
    struct tab _commandTab;

    //texts for control tab
    struct text _textChipTemperature;
    struct text _textOutputVoltage;
    struct text _textOutputCurrent;
    struct text _textMeasureH2Sensor1;
    struct text _textMeasureH2Sensor2;
    struct text _textNominalResistance;
    struct text _textSetTemperature;
    struct text _textCurrentTemperature;
    struct text _textOutput;

    //buttons for control tab
    struct button _buttonMeasureH2Sensor1Stop;
    struct button _buttonMeasureH2Sensor1Start;
    struct button _buttonMeasureH2Sensor2Stop;
    struct button _buttonMeasureH2Sensor2Start;
    struct button _buttonDecreaseNominalResistance;
    struct button _buttonIncreaseNominalResistance;
    struct button _buttonDecreaseSetTemperature;
    struct button _buttonIncreaseSetTemperature;
    struct button _buttonReset;
    struct button _buttonStop;
    struct button _buttonStart; 
    
    //values for control tab
    struct value _valueChipTemperature;
    struct value _valueOutputVoltage;
    struct value _valueOutputCurrent;
    struct value _valueMeasureH2Sensor1;
    struct value _valueMeasureH2Sensor2;
    struct value _valueNominalResistance;
    struct value _valueSetTemperature;
    struct value _valueCurrentTemperature;
    
    //event for control tab
    struct event _eventCommunicationError;

    //texts for tests tab
    struct text _textStartTemperature;
    struct text _textEndTemperature;
    struct text _textRiseTime;
    struct text _textFallTime;
    struct text _textRiseStepSize;
    struct text _textFallStepSize;
    struct text _textNumberCycles;
    struct text _textInformation;
    struct text _textTest;

    //buttons for tests tab
    struct button _buttonDecreaseStartTemperature;
    struct button _buttonIncreaseStartTemperature;
    struct button _buttonDecreaseEndTemperature;
    struct button _buttonIncreaseEndTemperature;
    struct button _buttonDecreaseRiseTime;
    struct button _buttonIncreaseRiseTime;
    struct button _buttonDecreaseFallTime;
    struct button _buttonIncreaseFallTime;
    struct button _buttonDecreaseRiseStepSize;
    struct button _buttonIncreaseRiseStepSize;
    struct button _buttonDecreaseFallStepSize;
    struct button _buttonIncreaseFallStepSize;
    struct button _buttonDecreaseNumberCycles;
    struct button _buttonIncreaseNumberCycles;
    struct button _buttonResetTest;
    struct button _buttonStopTest;
    struct button _buttonStartTest;

    //values for tests tab
    struct value _valueStartTemperature;
    struct value _valueEndTemperature;
    struct value _valueRiseTime;
    struct value _valueFallTime;
    struct value _valueRiseStepSize;
    struct value _valueFallStepSize;
    struct value _valueNumberCycles;
    struct value _valueProgress;

    //info for tests tab
    struct info _infoInformation;

    //texts for status tab
    struct text _textPWMSwitchingEvent;
    struct text _textCurrentLimitEvent;
    struct text _textPowerLimitEvent;
    struct text _textSoftResetEvent;
    struct text _textOverCurrentEvent;
    struct text _textOverttemperatureEvent;
    struct text _textVCCUnderVoltageEvent;
    struct text _textVDDIOUnderVOltageEvent;

    //events for status tab
    struct event _eventPWMSwitchingEvent;
    struct event _eventCurrentLimitEvent;
    struct event _eventPowerLimitEvent;
    struct event _eventSoftResetEvent;
    struct event _eventOverCurrentEvent;
    struct event _eventOverttemperatureEvent;
    struct event _eventVCCUnderVoltageEvent;
    struct event _eventVDDIOUnderVOltageEvent;

    //texts for command tab
    struct text _textPowerStage;
    struct text _textPWMSwitching;
    struct text _textPWMFrequency;
    struct text _textPWMAdjust;
    struct text _textPWMDutyCycle;
    struct text _textLDORegulation;
    struct text _textPeakInductorCurrent;
    struct text _textPowerLimit;

    //events for command tab
    struct event _eventPowerStage;
    struct event _eventPWMSwitching;
    struct event _eventLDORegulation;
    
    //infos for command tab
    struct info _infoPWMFrequency;
    struct info _infoPWMAdjust;
    struct info _infoPWMDutyCycle;
    struct info _infoPeakInductorCurrent;
    struct info _infoPowerLimit;

    TFT_eSPI _tft;
    TFT_eSprite _tabSelect;
    TFT_eSprite _background;
    TFT_eSprite _button;
    TFT_eSprite _text;
    TFT_eSprite _value;
    TFT_eSprite _event;
    TFT_eSprite _info;
};

#endif