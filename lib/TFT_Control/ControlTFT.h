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

struct controlData {
    double chipTemperature;
    double outputVoltage;
    double outputCurrent;
    double setTemperature;
    double currentTemperature;
    bool higherPressed;
    bool lowerPressed;
    bool resetPressed;
    bool stopPressed;
    bool startPressed;
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
    void drawControlTab();
    void drawTestsTab();
    void drawStatusTab();
    void drawCommandTab();

    void touchCalibrate();

    bool checkTouchTabs();
    bool checkButtonPressed(button button, uint16_t t_x, uint16_t t_y, bool pressed);
    bool inArea(int16_t startX, int16_t startY, int16_t withX, int16_t hightY, int16_t touchX, int16_t touchY);

    void drawButton(button button);
    void drawText(text text, uint8_t position);

    PRESSED_TAB getTab();

private:
    struct tab _controlTab;
    struct tab _testsTab;
    struct tab _statusTab;
    struct tab _commandTab;

    struct button _startButton;
    struct button _stopButton;
    struct button _resetButton;
    struct button _increaseTemperatureButton;
    struct button _decreaseTemperatureButton;

    struct text _LT8722Monitor;
    struct text _chipTemperature;
    struct text _outputVoltage;
    struct text _outputCurrent;
    struct text _setTemperature;
    struct text _output;
    
    struct text _valueChipTemperature;
    struct text _valueOutputVoltage;
    struct text _valueOutputCurrent;
    struct text _valueSetTemperature;
    struct text _valueCurrentTemperature;

    TFT_eSPI _tft;
    TFT_eSprite _tabSelect;
    TFT_eSprite _background;
    TFT_eSprite _display;
    TFT_eSprite _button;
    TFT_eSprite _text;
};

#endif