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

//store all information regarding each tab
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

//store all information regarding each button
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

//store all information regarding each text
struct text {
    char* name;
    int32_t startX;
    int32_t startY;
    int32_t withX;
    int32_t hightY;
    uint32_t textColor;
    uint32_t backgroundColor;
};

//store all information regarding each value
struct value {
    double value;
    int32_t startX;
    int32_t startY;
    int32_t withX;
    int32_t hightY;
    uint32_t textColor;
    uint32_t backgroundColor;
};

//store all information regarding each event (1 bit info)
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

//store all information regarding each info
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

//store all information of the control tab that can be updated
struct controlData {
    double valueChipTemperature;
    double valueOutputVoltage;
    double valueOutputCurrent;
    double valueMeasureH2Sensor1;
    double valueMeasureH2Sensor2;
    double valueNominalResistance;
    double valueSetValue;
    double valueCurrentTemperature;
    bool   buttonMeasureH2Sensor1StopPressed;
    bool   buttonMeasureH2Sensor1StartPressed;
    bool   buttonMeasureH2Sensor2StopPressed;
    bool   buttonMeasureH2Sensor2StartPressed;
    bool   buttonDecreaseNominalResistancePressed;
    bool   buttonIncreaseNominalResistancePressed;
    bool   buttonDecreaseSetValuePressed;
    bool   buttonIncreaseSetValuePressed;
    bool   eventCommunicationError;
    bool   voltageControlActive;
    bool   buttonHeaderPressed;
    bool   buttonResetPressed;
    bool   buttonStopPressed;
    bool   buttonStartPressed;
    bool   buttonStartPreviouslyPressed;
};

//store all information of the tests tab that can be updated
struct testsData {
    double  valueStartValue;
    double  valueEndValue;
    double  valueRiseTime;
    double  valueFallTime;
    double  valueRiseStepSize;
    double  valueFallStepSize;
    double  valueNumberCycles;
    double  valueProgress;
    bool    buttonDecreaseStartValuePressed;
    bool    buttonIncreaseStartValuePressed;
    bool    buttonDecreaseEndValuePressed;
    bool    buttonIncreaseEndValuePressed;
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

//store all information of the status tab that can be updated
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

//store all information of the control tab that can be updated
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

//store which tab is pressed
enum class PRESSED_TAB : uint8_t{
    CONTROL,
    TESTS,
    STATUS,
    COMMAND
};

class ControlTFT {
public:
    //constructor and begin function

    /**************************************************************************/
    /*!
        @brief Constructor for the ControlTFT class using the TFT_eSPI library
    */
    /**************************************************************************/
    ControlTFT();

    /**************************************************************************/
    /*!
        @brief Begin function for the the ControlTFT class initialising all 
        sprites used with the TFT_eSPI library. Additionally the positions, etc. 
        of the texts, buttons, values, events and infos to be drawn are defined. 
    */
    /**************************************************************************/
    void begin();

    //draw functions

    /**************************************************************************/
    /*!
        @brief Draw the selected tab with its backgrounds, buttons and texts.
        @param initial Defines if the selected tab has to be redrawn.
    */
    /**************************************************************************/
    void drawTabSelect(bool initial = false);

    /**************************************************************************/
    /*!
        @brief Update and Draw the buttons and texts for the control tab.
        @param data Stores all updated sensor data.
        @return data Retruns the information about pressed buttons
    */
    /**************************************************************************/
    controlData drawControlTab(controlData data);

    /**************************************************************************/
    /*!
        @brief Update and Draw the buttons and texts for the test tab.
        @param data Stores all updated sensor data.
        @return data Retruns the information about pressed buttons
    */
    /**************************************************************************/
    testsData drawTestsTab(testsData data);

    /**************************************************************************/
    /*!
        @brief Update and Draw the texts for the status tab.
        @param data Stores all updated status data.
    */
    /**************************************************************************/
    void drawStatusTab(statusData data);

    /**************************************************************************/
    /*!
        @brief Update and Draw the texts for the control tab.
        @param data Stores all updated control data.
    */
    /**************************************************************************/
    void drawCommandTab(commandData data);

    //return function

    /**************************************************************************/
    /*!
        @brief Returns the currently active tab
        @return currently active tab
    */
    /**************************************************************************/
    PRESSED_TAB getTab();

    //reset functions

    /**************************************************************************/
    /*!
        @brief Resets the controlRunning task by stopping it and releasing 
        changing the start buttons colour to its released state
    */
    /**************************************************************************/
    void resetControlRunning();

    /**************************************************************************/
    /*!
        @brief Resets the testRunning task by stopping it and releasing 
        changing the start test buttons colour to its released state
    */
    /**************************************************************************/
    void resetTestRunning();

private:
    //touch and button functions

    /**************************************************************************/
    /*!
        @brief Check and update which tab is pressed.
    */
    /**************************************************************************/
    bool checkTouchTabs();

    /**************************************************************************/
    /*!
        @brief Check if button is pressed and draw button
        @param button Stores all information about the button
        @param t_x X-Coordinate of the touch press
        @param t_y Y-Coordinate of the touch press
        @param pressed True if touchscreen pressed
        @return button Stores all information about the button
    */
    /**************************************************************************/
    button buttonPressed(button button, uint16_t t_x, uint16_t t_y, bool pressed);

    /**************************************************************************/
    /*!
        @brief Check if the touch press is in the area of the button
        @param startX button start position in x direction
        @param startY button start position in y direction
        @param withX with of the button
        @param hightY hight of the button
        @param touchX X-Coordinate of the touch press
        @param touchY Y-Coordinate of the touch press
        @return true if button is pressed
    */
    /**************************************************************************/
    bool inArea(int16_t startX, int16_t startY, int16_t withX, int16_t hightY, int16_t touchX, int16_t touchY);

    /**************************************************************************/
    /*!
        @brief Changes the colour of the button to its pressed state and starts 
        _controlRunning or _testRunning
        @param button button to be locked
        @param task task to be started
    */
    /**************************************************************************/
    void lockButton(button* lock, bool* task);

    /**************************************************************************/
    /*!
        @brief Changes the colour of the button to its released state and stops 
        _controlRunning or _testRunning
        @param button button to be released
        @param task task to be stopped
    */
    /**************************************************************************/
    void releaseButton(button action, button* release, bool* task);

    //draw functions

    /**************************************************************************/
    /*!
        @brief Draws the button
        @param button button to br drawn
    */
    /**************************************************************************/
    void drawButton(button button);

    /**************************************************************************/
    /*!
        @brief Draws the text
        @param text text to be drawn
        @param position position of the text in its field
    */
    /**************************************************************************/
    void drawText(text text, uint8_t position);

    /**************************************************************************/
    /*!
        @brief Draws the value
        @param value value to be drawn
        @param position position of the text in its field
    */
    /**************************************************************************/
    void drawValue(value value, uint8_t position);

    /**************************************************************************/
    /*!
        @brief Draws the event
        @param event event to be drawn
    */
    /**************************************************************************/
    void drawEvent(event event);

    /**************************************************************************/
    /*!
        @brief Draws the info
        @param info info to be drawn
    */
    /**************************************************************************/
    void drawInfo(info info);

    //calibrate function

    /**************************************************************************/
    /*!
        @brief Calibrate the orientation of the touchscreen
    */
    /**************************************************************************/
    void touchCalibrate();

    bool _testRunning;                  //status to indicate if test is running
    bool _controlRunning;               //status to indicate if control is running
    bool _measurementH2Sensor1Running;  //status to indicate if H2 Sensor 1 is used
    bool _measurementH2Sensor2Running;  //status to indicate if H2 Sensor 2 is used
    bool _voltageControlActive;         //status to indicate if the temperature of voltage can be controlled

    bool _redrawStartButton;
    bool _redrawStartTestButton;

    //tab struct for the different tabs
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
    struct text _textSetValue;
    struct text _textCurrentTemperature;
    struct text _textOutput;

    //buttons for control tab
    struct button _buttonMeasureH2Sensor1Stop;
    struct button _buttonMeasureH2Sensor1Start;
    struct button _buttonMeasureH2Sensor2Stop;
    struct button _buttonMeasureH2Sensor2Start;
    struct button _buttonDecreaseNominalResistance;
    struct button _buttonIncreaseNominalResistance;
    struct button _buttonDecreaseSetValue;
    struct button _buttonIncreaseSetValue;
    struct button _buttonVoltageControl;
    struct button _buttonHeader;
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
    struct value _valueSetValue;
    struct value _valueCurrentTemperature;
    
    //event for control tab
    struct event _eventCommunicationError;

    //texts for tests tab
    struct text _textStartValue;
    struct text _textEndValue;
    struct text _textRiseTime;
    struct text _textFallTime;
    struct text _textRiseStepSize;
    struct text _textFallStepSize;
    struct text _textNumberCycles;
    struct text _textInformation;
    struct text _textTest;

    //buttons for tests tab
    struct button _buttonDecreaseStartValue;
    struct button _buttonIncreaseStartValue;
    struct button _buttonDecreaseEndValue;
    struct button _buttonIncreaseEndValue;
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
    struct value _valueStartValue;
    struct value _valueEndValue;
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

    //sprites for the tft display
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