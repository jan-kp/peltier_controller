/*
 * File Name: TFT_Control.cpp
 * Description: Functions to control the TFT Display.
 *
 * Notes: This code was written as part of my master's thesis at the 
 *        Institute for Microsensors, -actuators and -systems (IMSAS) 
 *        at the University of Bremen.
 */

#include "ControlTFT.h"

/**************************************************************************/
/*!
    @brief Constructor for the ControlTFT class using the TFT_eSPI library
*/
/**************************************************************************/
ControlTFT::ControlTFT() : _tabSelect(&_tft), _background(&_tft), _button(&_tft), _text(&_tft), _value(&_tft), _event(&_tft),  _info(&_tft) {
  _tft.init();
  _tft.setRotation(1); 
}

/**************************************************************************/
/*!
    @brief Begin function for the the ControlTFT class initialising all 
    sprites used with the TFT_eSPI library. Additionally the positions, etc. 
    of the texts, buttons, values, events and infos to be drawn are defined. 
*/
/**************************************************************************/
void ControlTFT::begin(){
  _testRunning = false;
  _controlRunning = false;

  touchCalibrate();
  _tft.fillScreen(TFT_BLACK);
  _tft.fillRoundRect(0, 0, 480, 320, 15, LIGHT_GREY);

  //set the text position of the sprites to MC_DATUM
  _tabSelect.setTextDatum(4);
  _background.setTextDatum(4);
  _button.setTextDatum(4);
  _text.setTextDatum(4);
  _value.setTextDatum(4);
  _event.setTextDatum(4);
  _info.setTextDatum(4);

  //set the text size of the prites to 2
  _tabSelect.setTextSize(2);
  _background.setTextSize(2);
  _button.setTextSize(2);
  _text.setTextSize(2);
  _value.setTextSize(2);
  _event.setTextSize(2);
  _info.setTextSize(2);

  //information about the diffentent tabs
  _controlTab = {5,   5, 113, 35, 10, BLACK, LIGHT_GREY, GREY, true};
  _testsTab   = {124, 5, 113, 35, 10, BLACK, LIGHT_GREY, GREY, false};
  _statusTab  = {243, 5, 113, 35, 10, BLACK, LIGHT_GREY, GREY, false};
  _commandTab = {362, 5, 113, 35, 10, BLACK, LIGHT_GREY, GREY, false};

  //texts for control tab
  _textChipTemperature    = {"Temperature",    20,  60, 142, 27, BLACK, TFT_WHITE};
  _textOutputVoltage      = {"Voltage",       169,  60, 142, 27, BLACK, TFT_WHITE};
  _textOutputCurrent      = {"Current",       318,  60, 142, 27, BLACK, TFT_WHITE};
  _textMeasureH2Sensor1   = {"H2 Sensor 1:",   26, 120, 160, 27, BLACK, TFT_WHITE};
  _textMeasureH2Sensor2   = {"H2 Sensor 2:",   26, 147, 160, 27, BLACK, TFT_WHITE};
  _textNominalResistance  = {"R_0:",           26, 180, 150, 27, BLACK, TFT_WHITE};
  _textSetValue           = {"Temp. Set:",     26, 213, 150, 27, BLACK, TFT_WHITE};
  _textCurrentTemperature = {"Temp. Current:", 26, 240, 170, 27, BLACK, TFT_WHITE};
  _textOutput             = {"Output:",        26, 273,  80, 27, BLACK, TFT_WHITE};

  //buttons for control tab
  _buttonMeasureH2Sensor1Stop      = {"Stop",    311, 123,  70,  21, 10, BLACK, LIGHT_GREY, GREY, false, false};
  _buttonMeasureH2Sensor1Start     = {"Start",   387, 123,  70,  21, 10, BLACK, LIGHT_GREY, BLUE, false, false};
  _buttonMeasureH2Sensor2Stop      = {"Stop",    311, 150,  70,  21, 10, BLACK, LIGHT_GREY, GREY, false, false};
  _buttonMeasureH2Sensor2Start     = {"Start",   387, 150,  70,  21, 10, BLACK, LIGHT_GREY, BLUE, false, false};
  _buttonIncreaseNominalResistance = {"Up",      387, 183,  70,  21, 10, BLACK, LIGHT_GREY, GREY, false, false};
  _buttonDecreaseNominalResistance = {"Down",    311, 183,  70,  21, 10, BLACK, LIGHT_GREY, GREY, false, false};
  _buttonDecreaseSetValue          = {"Down",    311, 216,  70,  21, 10, BLACK, LIGHT_GREY, GREY, false, false};
  _buttonIncreaseSetValue          = {"Up",      387, 216,  70,  21, 10, BLACK, LIGHT_GREY, GREY, false, false};
  _buttonVoltageControl            = {"Temp.",   311, 243, 146,  21, 10, BLACK, LIGHT_GREY, GREY, false, false};
  _buttonReset                     = {"Reset",   235, 276,  70,  21, 10, BLACK, LIGHT_GREY, GREY, false, false};
  _buttonStop                      = {"Stop",    311, 276,  70,  21, 10, BLACK, LIGHT_GREY, GREY, false, false};
  _buttonStart                     = {"Start",   387, 276,  70,  21, 10, BLACK, LIGHT_GREY, BLUE, false, false};

  //values for control tab
  _valueChipTemperature    = {0.0,  20,  87, 142, 27, BLACK, TFT_WHITE};
  _valueOutputVoltage      = {0.0, 169,  87, 142, 27, BLACK, TFT_WHITE};
  _valueOutputCurrent      = {0.0, 318,  87, 142, 27, BLACK, TFT_WHITE};
  _valueMeasureH2Sensor1   = {0.0, 190, 120, 100, 27, BLACK, TFT_WHITE};
  _valueMeasureH2Sensor2   = {0.0, 190, 147, 100, 27, BLACK, TFT_WHITE};
  _valueNominalResistance  = {0.0, 190, 180, 100, 27, BLACK, TFT_WHITE};
  _valueSetValue           = {0.0, 190, 213, 100, 27, BLACK, TFT_WHITE};
  _valueCurrentTemperature = {0.0, 190, 240, 100, 27, BLACK, TFT_WHITE};

  //event for control tab
  _eventCommunicationError = {0, "-", "ERROR",  159, 276, 70, 21, BLACK, RED, GREEN};

  //texts for tests tab
  _textStartValue   = {"Start Temp.:",    26,   60, 170, 26, BLACK, TFT_WHITE};
  _textEndValue     = {"End Temp.:",      26,   86, 160, 26, BLACK, TFT_WHITE};
  _textRiseTime     = {"Rise Time:",      26,  112, 140, 26, BLACK, TFT_WHITE};
  _textFallTime     = {"Fall Time:",      26,  138, 140, 26, BLACK, TFT_WHITE};
  _textRiseStepSize = {"Rise Step Size:", 26,  164, 200, 26, BLACK, TFT_WHITE};
  _textFallStepSize = {"Fall Step Size:", 26,  190, 200, 26, BLACK, TFT_WHITE};
  _textNumberCycles = {"Numb. Cycles:",   26,  216, 150, 26, BLACK, TFT_WHITE};
  _textInformation  = {"Information:",    26,  245, 150, 26, BLACK, TFT_WHITE};
  _textTest         = {"Progress:",       26,  274, 110, 26, BLACK, TFT_WHITE};

  //buttons for tests tab
  _buttonDecreaseStartValue   = {"Down",  311,  62, 70,  22, 10, BLACK, LIGHT_GREY, GREY, false, false};
  _buttonIncreaseStartValue   = {"Up",    387,  62, 70,  22, 10, BLACK, LIGHT_GREY, GREY, false, false};
  _buttonDecreaseEndValue     = {"Down",  311,  88, 70,  22, 10, BLACK, LIGHT_GREY, GREY, false, false};
  _buttonIncreaseEndValue     = {"Up",    387,  88, 70,  22, 10, BLACK, LIGHT_GREY, GREY, false, false};
  _buttonDecreaseRiseTime     = {"Down",  311, 114, 70,  22, 10, BLACK, LIGHT_GREY, GREY, false, false};
  _buttonIncreaseRiseTime     = {"Up",    387, 114, 70,  22, 10, BLACK, LIGHT_GREY, GREY, false, false};
  _buttonDecreaseFallTime     = {"Down",  311, 140, 70,  22, 10, BLACK, LIGHT_GREY, GREY, false, false};
  _buttonIncreaseFallTime     = {"Up",    387, 140, 70,  22, 10, BLACK, LIGHT_GREY, GREY, false, false};
  _buttonDecreaseRiseStepSize = {"Down",  311, 166, 70,  22, 10, BLACK, LIGHT_GREY, GREY, false, false};
  _buttonIncreaseRiseStepSize = {"Up",    387, 166, 70,  22, 10, BLACK, LIGHT_GREY, GREY, false, false};
  _buttonDecreaseFallStepSize = {"Down",  311, 192, 70,  22, 10, BLACK, LIGHT_GREY, GREY, false, false};
  _buttonIncreaseFallStepSize = {"Up",    387, 192, 70,  22, 10, BLACK, LIGHT_GREY, GREY, false, false};
  _buttonDecreaseNumberCycles = {"Down",  311, 218, 70,  22, 10, BLACK, LIGHT_GREY, GREY, false, false};
  _buttonIncreaseNumberCycles = {"Up",    387, 218, 70,  22, 10, BLACK, LIGHT_GREY, GREY, false, false};
  _buttonResetTest            = {"Reset", 235, 276, 70,  22, 10, BLACK, LIGHT_GREY, GREY, false, false};
  _buttonStopTest             = {"Stop",  311, 276, 70,  22, 10, BLACK, LIGHT_GREY, GREY, false, false};
  _buttonStartTest            = {"Start", 387, 276, 70,  22, 10, BLACK, LIGHT_GREY, BLUE, false, false};
  
  //values for tests tab
  _valueStartValue   = {0.0, 200,  60, 80, 26, BLACK, TFT_WHITE};
  _valueEndValue     = {0.0, 200,  86, 80, 26, BLACK, TFT_WHITE};
  _valueRiseTime     = {0.0, 200, 112, 80, 26, BLACK, TFT_WHITE};
  _valueFallTime     = {0.0, 200, 138, 80, 26, BLACK, TFT_WHITE};
  _valueRiseStepSize = {0.0, 200, 164, 80, 26, BLACK, TFT_WHITE};
  _valueFallStepSize = {0.0, 200, 190, 80, 26, BLACK, TFT_WHITE};
  _valueNumberCycles = {0.0, 200, 216, 80, 26, BLACK, TFT_WHITE};
  _valueProgress     = {0.0, 140, 274, 80, 26, BLACK, TFT_WHITE};

  //infos for tests tab
  _infoInformation = {0x0, 170, 245, 290, 26, BLACK, TFT_WHITE, "Ready", "Running ...", "Step Wait Time to Short", "Invalid Input", "Communication ERROR"};

  //texts for status tab
  _textPWMSwitchingEvent      = {"PWM Switching:",      26, 60,  170, 30, BLACK, TFT_WHITE};
  _textCurrentLimitEvent      = {"Current Limit Loop:", 26, 90,  230, 30, BLACK, TFT_WHITE};
  _textPowerLimitEvent        = {"Power Limit Loop:",   26, 120, 200, 30, BLACK, TFT_WHITE};
  _textSoftResetEvent         = {"Soft Reset:",         26, 150, 140, 30, BLACK, TFT_WHITE};
  _textOverCurrentEvent       = {"Over Current:",       26, 180, 160, 30, BLACK, TFT_WHITE};
  _textOverttemperatureEvent  = {"Overtemperature:",    26, 210, 190, 30, BLACK, TFT_WHITE};
  _textVCCUnderVoltageEvent   = {"VCC Undervoltage:",   26, 240, 200, 30, BLACK, TFT_WHITE};
  _textVDDIOUnderVOltageEvent = {"VDDIO Undervoltage:", 26, 270, 230, 30, BLACK, TFT_WHITE};

  //events for status tab
  _eventPWMSwitchingEvent      = {0, "OFF", "ON",  320, 65,  80, 20, BLACK, BLUE, TFT_WHITE};
  _eventCurrentLimitEvent      = {0, "-", "Event", 320, 95,  80, 20, BLACK, RED,  TFT_WHITE};
  _eventPowerLimitEvent        = {0, "-", "Event", 320, 125, 80, 20, BLACK, RED,  TFT_WHITE};
  _eventSoftResetEvent         = {0, "-", "Event", 320, 155, 80, 20, BLACK, RED,  TFT_WHITE};
  _eventOverCurrentEvent       = {0, "-", "Event", 320, 185, 80, 20, BLACK, RED,  TFT_WHITE};
  _eventOverttemperatureEvent  = {0, "-", "Event", 320, 215, 80, 20, BLACK, RED,  TFT_WHITE};
  _eventVCCUnderVoltageEvent   = {0, "-", "Event", 320, 245, 80, 20, BLACK, RED,  TFT_WHITE};
  _eventVDDIOUnderVOltageEvent = {0, "-", "Event", 320, 275, 80, 20, BLACK, RED,  TFT_WHITE};

  //texts for command tab
  _textPowerStage          = {"Power Stage:",           26, 60,  160, 30, BLACK, TFT_WHITE};
  _textPWMSwitching        = {"PWM Switching:",         26, 90,  170, 30, BLACK, TFT_WHITE};
  _textPWMFrequency        = {"PWM Frequency:",         26, 120, 170, 30, BLACK, TFT_WHITE};
  _textPWMAdjust           = {"PWM Adjust:",            26, 150, 160, 30, BLACK, TFT_WHITE};
  _textPWMDutyCycle        = {"PWM Duty Cycle:",        26, 180, 180, 30, BLACK, TFT_WHITE};
  _textLDORegulation       = {"LDO Regulation:",        26, 210, 180, 30, BLACK, TFT_WHITE};
  _textPeakInductorCurrent = {"Peak Inductor Current:", 26, 240, 260, 30, BLACK, TFT_WHITE};
  _textPowerLimit          = {"Power Limit:",           26, 270, 160, 30, BLACK, TFT_WHITE};

  //events for command tab
  _eventPowerStage    = {0, "OFF", "ON"   , 300, 65,  120, 20, BLACK, BLUE,       TFT_WHITE};
  _eventPWMSwitching  = {0, "OFF", "ON"   , 300, 95,  120, 20, BLACK, BLUE,       TFT_WHITE}; 
  _eventLDORegulation = {0, "3.1V", "3.4V", 300, 215, 120, 20, BLACK, TFT_WHITE,  TFT_WHITE};

  //infos for command tab
  _infoPWMFrequency        = {0x0, 300, 125, 120, 20, BLACK, TFT_WHITE, "0.5MHZ", "1MHZ", "1.5MHZ", "2MHZ", "2.5MHZ", "3MHZ"};
  _infoPWMAdjust           = {0x0, 300, 155, 120, 20, BLACK, TFT_WHITE, "0%", "15%", "-15%", "0%"};
  _infoPWMDutyCycle        = {0x0, 300, 185, 120, 20, BLACK, TFT_WHITE, "20% - 80%", "15% - 85%", "10% - 90%"};
  _infoPeakInductorCurrent = {0x0, 300, 245, 120, 20, BLACK, TFT_WHITE, "0.252A", "0.594A", "0.936A", "1.278A", "1.620A", "1.962A", "2.304A", "2.646A"};
  _infoPowerLimit          = {0x0, 300, 275, 120, 20, BLACK, TFT_WHITE, "2W", "No Limit", "3W", "3.5W"};
}

/**************************************************************************/
/*!
    @brief Draw the selected tab with its backgrounds, buttons and texts.
    @param initial Defines if the selected tab has to be redrawn.
*/
/**************************************************************************/
void ControlTFT::drawTabSelect(bool initial) {
  bool changedTab = checkTouchTabs();

  if (changedTab || initial) {
    _tabSelect.createSprite(480, 47);
    _tabSelect.fillRoundRect(0, 0, 480, 47, 15, LIGHT_GREY);

    if (_controlTab.pressed) {
      //draw tab select
      _tabSelect.setTextColor(_controlTab.textColor, _controlTab.selectColor);
      _tabSelect.fillRoundRect(_controlTab.startX, _controlTab.startY, _controlTab.withX, _controlTab.hightY, _controlTab.radius, _controlTab.selectColor);
      _tabSelect.drawString("Control", 62, 23);
      _tabSelect.setTextColor(_controlTab.textColor, _controlTab.backgroundColor);
      _tabSelect.drawString("Tests", 181, 23);
      _tabSelect.drawString("Status", 300, 23);
      _tabSelect.drawString("Command", 419, 23);
      _tabSelect.pushSprite(0, 0, TFT_BLACK);
      _tabSelect.deleteSprite();

      //draw background
      _background.createSprite(480, 260);
      _background.setTextColor(_controlTab.textColor, TFT_WHITE);
      _background.fillRoundRect(  0,   0, 480, 260, 15, LIGHT_GREY);
      _background.fillRoundRect( 20,   0, 142,  54, 10, TFT_WHITE);
      _background.fillRoundRect(169,   0, 142,  54, 10, TFT_WHITE);
      _background.fillRoundRect(318,   0, 142,  54, 10, TFT_WHITE);
      _background.fillRoundRect( 20,  60, 440,  54, 10, TFT_WHITE);
      _background.fillRoundRect( 20, 120, 440,  27, 10, TFT_WHITE);
      _background.fillRoundRect( 20, 153, 440,  54, 10, TFT_WHITE);
      _background.fillRoundRect( 20, 213, 440,  27, 10, TFT_WHITE);
      _background.pushSprite(0, 60, TFT_BLACK);
      _background.deleteSprite();

      //draw texts, values, buttons, events and infos
      drawText(_textChipTemperature, MC_DATUM);
      drawText(_textOutputVoltage, MC_DATUM);
      drawText(_textOutputCurrent, MC_DATUM);

      drawValue(_valueChipTemperature, MC_DATUM);
      drawValue(_valueOutputVoltage, MC_DATUM);
      drawValue(_valueOutputCurrent, MC_DATUM);

      drawText(_textMeasureH2Sensor1, ML_DATUM);
      drawValue(_valueMeasureH2Sensor1, MC_DATUM);
      drawButton(_buttonMeasureH2Sensor1Stop);
      drawButton(_buttonMeasureH2Sensor1Start);

      drawText(_textMeasureH2Sensor2, ML_DATUM);
      drawValue(_valueMeasureH2Sensor2, MC_DATUM);
      drawButton(_buttonMeasureH2Sensor2Stop);
      drawButton(_buttonMeasureH2Sensor2Start);

      drawText(_textNominalResistance, ML_DATUM);
      drawValue(_valueNominalResistance, MC_DATUM);
      drawButton(_buttonDecreaseNominalResistance);
      drawButton(_buttonIncreaseNominalResistance);

      drawText(_textSetValue, ML_DATUM);
      drawValue(_valueSetValue, MC_DATUM);
      drawButton(_buttonDecreaseSetValue);
      drawButton(_buttonIncreaseSetValue);

      drawText(_textCurrentTemperature, ML_DATUM);
      drawValue(_valueCurrentTemperature, MC_DATUM);
      drawButton(_buttonVoltageControl);

      drawText(_textOutput, ML_DATUM);
      drawEvent(_eventCommunicationError);
      drawButton(_buttonReset);
      drawButton(_buttonStop);
      drawButton(_buttonStart);
    } else if (_testsTab.pressed) {
      //draw tab select
      _tabSelect.setTextColor(_testsTab.textColor, _testsTab.selectColor);
      _tabSelect.fillRoundRect(_testsTab.startX, _testsTab.startY, _testsTab.withX, _testsTab.hightY, _testsTab.radius, _testsTab.selectColor);
      _tabSelect.drawString("Tests", 181, 23);
      _tabSelect.setTextColor(_testsTab.textColor, _testsTab.backgroundColor);
      _tabSelect.drawString("Control", 62, 23);
      _tabSelect.drawString("Status", 300, 23);
      _tabSelect.drawString("Command", 419, 23);
      _tabSelect.pushSprite(0, 0, TFT_BLACK);
      _tabSelect.deleteSprite();

      //draw background
      _background.createSprite(480, 260);
      _background.fillRoundRect( 0,   0, 480, 260, 15, LIGHT_GREY);
      _background.fillRoundRect(20,   0, 440, 182, 10, TFT_WHITE);
      _background.fillRoundRect(20, 185, 440,  26, 10, TFT_WHITE);
      _background.fillRoundRect(20, 214, 440,  26, 10, TFT_WHITE);
      _background.pushSprite(0, 60, TFT_BLACK);
      _background.deleteSprite();

      //draw texts, values, buttons, events and infos
      drawText(_textStartValue, ML_DATUM);
      drawValue(_valueStartValue, MC_DATUM);
      drawButton(_buttonDecreaseStartValue);
      drawButton(_buttonIncreaseStartValue);

      drawText(_textEndValue, ML_DATUM);
      drawValue(_valueEndValue, MC_DATUM);
      drawButton(_buttonDecreaseEndValue);
      drawButton(_buttonIncreaseEndValue);

      drawText(_textRiseTime, ML_DATUM);
      drawValue(_valueRiseTime, MC_DATUM);
      drawButton(_buttonDecreaseRiseTime);
      drawButton(_buttonIncreaseRiseTime);

      drawText(_textFallTime, ML_DATUM);
      drawValue(_valueFallTime, MC_DATUM);
      drawButton(_buttonDecreaseFallTime);
      drawButton(_buttonIncreaseFallTime);

      drawText(_textRiseStepSize, ML_DATUM);
      drawValue(_valueRiseStepSize, MC_DATUM);
      drawButton(_buttonDecreaseRiseStepSize);
      drawButton(_buttonIncreaseRiseStepSize);

      drawText(_textFallStepSize, ML_DATUM);
      drawValue(_valueFallStepSize, MC_DATUM);
      drawButton(_buttonDecreaseFallStepSize);
      drawButton(_buttonIncreaseFallStepSize);

      drawText(_textNumberCycles, ML_DATUM);
      drawValue(_valueNumberCycles, MC_DATUM);
      drawButton(_buttonDecreaseNumberCycles);
      drawButton(_buttonIncreaseNumberCycles);

      drawText(_textInformation, ML_DATUM);
      drawInfo(_infoInformation);

      drawText(_textTest, ML_DATUM);
      drawValue(_valueProgress, MC_DATUM);
      drawButton(_buttonResetTest);
      drawButton(_buttonStopTest);
      drawButton(_buttonStartTest);
    } else if (_statusTab.pressed) {
      //draw tab select
      _tabSelect.setTextColor(_statusTab.textColor, _statusTab.selectColor);
      _tabSelect.fillRoundRect(_statusTab.startX, _statusTab.startY, _statusTab.withX, _statusTab.hightY, _statusTab.radius, _statusTab.selectColor);
      _tabSelect.drawString("Status", 300, 23);
      _tabSelect.setTextColor(_statusTab.textColor, _statusTab.backgroundColor);
      _tabSelect.drawString("Control", 62, 23);
      _tabSelect.drawString("Tests", 181, 23);
      _tabSelect.drawString("Command", 419, 23);
      _tabSelect.pushSprite(0, 0, TFT_BLACK);
      _tabSelect.deleteSprite();

      //draw background
      _background.createSprite(480, 260);
      _background.fillRoundRect( 0, 0, 480, 260, 15, LIGHT_GREY);
      _background.fillRoundRect(20, 0, 440, 240, 10, TFT_WHITE);
      _background.pushSprite(0, 60, TFT_BLACK);
      _background.deleteSprite();

      //draw texts, values, buttons, events and infos
      drawText(_textPWMSwitchingEvent, ML_DATUM);
      drawText(_textCurrentLimitEvent, ML_DATUM);
      drawText(_textPowerLimitEvent, ML_DATUM);
      drawText(_textSoftResetEvent, ML_DATUM);
      drawText(_textOverCurrentEvent, ML_DATUM);
      drawText(_textOverttemperatureEvent, ML_DATUM);
      drawText(_textVCCUnderVoltageEvent, ML_DATUM);
      drawText(_textVDDIOUnderVOltageEvent, ML_DATUM);

      drawEvent(_eventPWMSwitchingEvent);
      drawEvent(_eventCurrentLimitEvent);
      drawEvent(_eventPowerLimitEvent);
      drawEvent(_eventSoftResetEvent);
      drawEvent(_eventOverCurrentEvent);
      drawEvent(_eventOverttemperatureEvent);
      drawEvent(_eventVCCUnderVoltageEvent);
      drawEvent(_eventVDDIOUnderVOltageEvent);
    } else if (_commandTab.pressed) {
      //draw tab select
      _tabSelect.setTextColor(_commandTab.textColor, _commandTab.selectColor);
      _tabSelect.fillRoundRect(_commandTab.startX, _commandTab.startY, _commandTab.withX, _commandTab.hightY, _commandTab.radius, _commandTab.selectColor);
      _tabSelect.drawString("Command", 419, 23);
      _tabSelect.setTextColor(_commandTab.textColor, _commandTab.backgroundColor);
      _tabSelect.drawString("Control", 62, 23);
      _tabSelect.drawString("Tests", 181, 23);
      _tabSelect.drawString("Status", 300, 23);
      _tabSelect.pushSprite(0, 0, TFT_BLACK);
      _tabSelect.deleteSprite();

      //draw background
      _background.createSprite(480, 260);
      _background.fillRoundRect( 0, 0, 480, 260, 15, LIGHT_GREY);
      _background.fillRoundRect(20, 0, 440, 240, 10, TFT_WHITE);
      _background.pushSprite(0, 60, TFT_BLACK);
      _background.deleteSprite();

      //draw texts, values, buttons, events and infos
      drawText(_textPowerStage, ML_DATUM);
      drawText(_textPWMSwitching, ML_DATUM);
      drawText(_textPWMFrequency, ML_DATUM);
      drawText(_textPWMAdjust, ML_DATUM);
      drawText(_textPWMDutyCycle, ML_DATUM);
      drawText(_textLDORegulation, ML_DATUM);
      drawText(_textPeakInductorCurrent, ML_DATUM);
      drawText(_textPowerLimit, ML_DATUM);

      drawEvent(_eventPowerStage);
      drawEvent(_eventPWMSwitching);
      drawEvent(_eventLDORegulation);

      drawInfo(_infoPWMFrequency);
      drawInfo(_infoPWMAdjust);
      drawInfo(_infoPWMDutyCycle);
      drawInfo(_infoPeakInductorCurrent);
      drawInfo(_infoPowerLimit);
    }
  }
}


/**************************************************************************/
/*!
    @brief Update and Draw the buttons and texts for the control tab.
    @param data Stores all updated sensor data.
    @return data Retruns the information about pressed buttons
*/
/**************************************************************************/
controlData ControlTFT::drawControlTab(controlData data) {
  uint16_t t_x = 0, t_y = 0;                 // To store the touch coordinates
  bool pressed = _tft.getTouch(&t_x, &t_y);  // Pressed will be set true is there is a valid touch on the screen

  //update values
  _valueChipTemperature.value    = data.valueChipTemperature;
  _valueOutputVoltage.value      = data.valueOutputVoltage;
  _valueOutputCurrent.value      = data.valueOutputCurrent;
  _valueMeasureH2Sensor1.value   = data.valueMeasureH2Sensor1;
  _valueMeasureH2Sensor2.value   = data.valueMeasureH2Sensor2;
  _valueNominalResistance.value  = data.valueNominalResistance;
  _valueSetValue.value           = data.valueSetValue;
  _valueCurrentTemperature.value = data.valueCurrentTemperature;

  //draw values
  drawValue(_valueChipTemperature, MC_DATUM);
  drawValue(_valueOutputVoltage, MC_DATUM);
  drawValue(_valueOutputCurrent, MC_DATUM);
  drawValue(_valueMeasureH2Sensor1, MC_DATUM);
  drawValue(_valueMeasureH2Sensor2, MC_DATUM);
  drawValue(_valueNominalResistance, MC_DATUM);
  drawValue(_valueSetValue, MC_DATUM);
  drawValue(_valueCurrentTemperature, MC_DATUM);

  //update event
  _eventCommunicationError.bit = data.eventCommunicationError;

  //draw event
  drawEvent(_eventCommunicationError);

  //check and update button press
  _buttonMeasureH2Sensor1Stop = buttonPressed(_buttonMeasureH2Sensor1Stop, t_x, t_y, pressed);
  data.buttonMeasureH2Sensor1StopPressed = _buttonMeasureH2Sensor1Stop.pressed;
  releaseButton(_buttonMeasureH2Sensor1Stop, &_buttonMeasureH2Sensor1Start, &_measurementH2Sensor1Running);

  _buttonMeasureH2Sensor1Start = buttonPressed(_buttonMeasureH2Sensor1Start, t_x, t_y, pressed);
  data.buttonMeasureH2Sensor1StartPressed = _buttonMeasureH2Sensor1Start.pressed;
  lockButton(&_buttonMeasureH2Sensor1Start, &_measurementH2Sensor1Running);

  _buttonMeasureH2Sensor2Stop = buttonPressed(_buttonMeasureH2Sensor2Stop, t_x, t_y, pressed);
  data.buttonMeasureH2Sensor2StopPressed = _buttonMeasureH2Sensor2Stop.pressed;
  releaseButton(_buttonMeasureH2Sensor2Stop, &_buttonMeasureH2Sensor2Start, &_measurementH2Sensor2Running);

  _buttonMeasureH2Sensor2Start = buttonPressed(_buttonMeasureH2Sensor2Start, t_x, t_y, pressed);
  data.buttonMeasureH2Sensor2StartPressed = _buttonMeasureH2Sensor2Start.pressed;
  lockButton(&_buttonMeasureH2Sensor2Start, &_measurementH2Sensor2Running);

  //buttons can only be pressed if no test is running
  if (!_testRunning) {
    _buttonDecreaseNominalResistance = buttonPressed(_buttonDecreaseNominalResistance, t_x, t_y, pressed);
    data.buttonDecreaseNominalResistancePressed = _buttonDecreaseNominalResistance.pressed;

    _buttonIncreaseNominalResistance = buttonPressed(_buttonIncreaseNominalResistance, t_x, t_y, pressed);
    data.buttonIncreaseNominalResistancePressed = _buttonIncreaseNominalResistance.pressed;

    _buttonDecreaseSetValue = buttonPressed(_buttonDecreaseSetValue, t_x, t_y, pressed);
    data.buttonDecreaseSetValuePressed = _buttonDecreaseSetValue.pressed;

    _buttonIncreaseSetValue = buttonPressed(_buttonIncreaseSetValue, t_x, t_y, pressed);
    data.buttonIncreaseSetValuePressed = _buttonIncreaseSetValue.pressed;

    if (!_controlRunning) {
      _buttonVoltageControl = buttonPressed(_buttonVoltageControl, t_x, t_y, pressed);

      if (_buttonVoltageControl.pressed && !_buttonVoltageControl.previouslyPressed) {
        if (_buttonVoltageControl.pressed && _voltageControlActive) {
          _voltageControlActive = false;
          data.voltageControlActive = false;
          _buttonVoltageControl.name = "Temp.";
          _textSetValue.name = "Temp. Set:";
          _textStartValue.name = "Start Temp.:";
          _textEndValue.name = "End Temp.:";
          _textRiseTime.name = "Rise Time:";
          _textFallTime.name = "Fall Time:";
          _textRiseStepSize.name = "Rise Step Size:";
          _textFallStepSize.name = "Fall Step Size:";

          drawButton(_buttonVoltageControl);
          drawText(_textSetValue, ML_DATUM);
        } else if (_buttonVoltageControl.pressed && !_voltageControlActive) {
          _voltageControlActive = true;
          data.voltageControlActive = true;
          _buttonVoltageControl.name = "Voltage";
          _textSetValue.name = "Voltage Set:";
          _textStartValue.name = "Start Voltage:";
          _textEndValue.name = "End Voltage:";
          _textRiseTime.name = "Fall Time:";
          _textFallTime.name = "Rise Time:";
          _textRiseStepSize.name = "Fall Step Size:";
          _textFallStepSize.name = "Rise Step Size:";

          drawButton(_buttonVoltageControl);
          drawText(_textSetValue, ML_DATUM);
        }
      }
    }

    _buttonReset = buttonPressed(_buttonReset, t_x, t_y, pressed);
    data.buttonResetPressed = _buttonReset.pressed;
    releaseButton(_buttonReset, &_buttonStart, &_controlRunning);

    _buttonStop = buttonPressed(_buttonStop, t_x, t_y, pressed);
    data.buttonStopPressed = _buttonStop.pressed;
    releaseButton(_buttonStop, &_buttonStart, &_controlRunning);

    _buttonStart = buttonPressed(_buttonStart, t_x, t_y, pressed);
    data.buttonStartPreviouslyPressed = data.buttonStartPressed;
    data.buttonStartPressed = _buttonStart.pressed;
    lockButton(&_buttonStart, &_controlRunning);
    if (_redrawStartButton){
      drawButton(_buttonStart);
      _redrawStartButton = false;
    }
    
  }  

  return data;
}

/**************************************************************************/
/*!
    @brief Update and Draw the buttons and texts for the test tab.
    @param data Stores all updated sensor data.
    @return data Retruns the information about pressed buttons
*/
/**************************************************************************/
testsData ControlTFT::drawTestsTab(testsData data) {
  uint16_t t_x = 0, t_y = 0;                 // To store the touch coordinates
  bool pressed = _tft.getTouch(&t_x, &t_y);  // Pressed will be set true is there is a valid touch on the screen

  //update values
  _valueStartValue.value   = data.valueStartValue; 
  _valueEndValue.value     = data.valueEndValue;   
  _valueRiseTime.value     = data.valueRiseTime;
  _valueFallTime.value     = data.valueFallTime;
  _valueRiseStepSize.value = data.valueRiseStepSize;
  _valueFallStepSize.value = data.valueFallStepSize;
  _valueNumberCycles.value = data.valueNumberCycles;
  _valueProgress.value     = data.valueProgress;

  //draw values
  drawValue(_valueStartValue  , MC_DATUM);
  drawValue(_valueEndValue    , MC_DATUM);
  drawValue(_valueRiseTime    , MC_DATUM);
  drawValue(_valueFallTime    , MC_DATUM);
  drawValue(_valueRiseStepSize, MC_DATUM);
  drawValue(_valueFallStepSize, MC_DATUM);
  drawValue(_valueNumberCycles, MC_DATUM);
  drawValue(_valueProgress    , MC_DATUM);

  //update info
  _infoInformation.bits = data.infoInformation;

  //draw info
  drawInfo(_infoInformation);

  //check and update button press if control is not running
  if (!_controlRunning) {
    
    //buttons can only be pressed if no test is running
    if (!_testRunning){
      _buttonDecreaseStartValue = buttonPressed(_buttonDecreaseStartValue, t_x, t_y, pressed);
      data.buttonDecreaseStartValuePressed = _buttonDecreaseStartValue.pressed;

      _buttonIncreaseStartValue = buttonPressed(_buttonIncreaseStartValue, t_x, t_y, pressed);
      data.buttonIncreaseStartValuePressed = _buttonIncreaseStartValue.pressed;

      _buttonDecreaseEndValue = buttonPressed(_buttonDecreaseEndValue, t_x, t_y, pressed);
      data.buttonDecreaseEndValuePressed = _buttonDecreaseEndValue.pressed;

      _buttonIncreaseEndValue = buttonPressed(_buttonIncreaseEndValue, t_x, t_y, pressed);
      data.buttonIncreaseEndValuePressed = _buttonIncreaseEndValue.pressed;

      _buttonDecreaseRiseTime = buttonPressed(_buttonDecreaseRiseTime, t_x, t_y, pressed);
      data.buttonDecreaseRiseTimePressed = _buttonDecreaseRiseTime.pressed;

      _buttonIncreaseRiseTime = buttonPressed(_buttonIncreaseRiseTime, t_x, t_y, pressed);
      data.buttonIncreaseRiseTimePressed = _buttonIncreaseRiseTime.pressed;

      _buttonDecreaseFallTime = buttonPressed(_buttonDecreaseFallTime, t_x, t_y, pressed);
      data.buttonDecreaseFallTimePressed = _buttonDecreaseFallTime.pressed;

      _buttonIncreaseFallTime = buttonPressed(_buttonIncreaseFallTime, t_x, t_y, pressed);
      data.buttonIncreaseFallTimePressed = _buttonIncreaseFallTime.pressed;

      _buttonDecreaseRiseStepSize = buttonPressed(_buttonDecreaseRiseStepSize, t_x, t_y, pressed);
      data.buttonDecreaseRiseStepSizePressed = _buttonDecreaseRiseStepSize.pressed;

      _buttonIncreaseRiseStepSize = buttonPressed(_buttonIncreaseRiseStepSize, t_x, t_y, pressed);
      data.buttonIncreaseRiseStepSizePressed = _buttonIncreaseRiseStepSize.pressed;

      _buttonDecreaseFallStepSize = buttonPressed(_buttonDecreaseFallStepSize, t_x, t_y, pressed);
      data.buttonDecreaseFallStepSizePressed = _buttonDecreaseFallStepSize.pressed;

      _buttonIncreaseFallStepSize = buttonPressed(_buttonIncreaseFallStepSize, t_x, t_y, pressed);
      data.buttonIncreaseFallStepSizePressed = _buttonIncreaseFallStepSize.pressed;

      _buttonDecreaseNumberCycles = buttonPressed(_buttonDecreaseNumberCycles, t_x, t_y, pressed);
      data.buttonDecreaseNumberCyclesPressed = _buttonDecreaseNumberCycles.pressed;

      _buttonIncreaseNumberCycles = buttonPressed(_buttonIncreaseNumberCycles, t_x, t_y, pressed);
      data.buttonIncreaseNumberCyclesPressed = _buttonIncreaseNumberCycles.pressed;
    }
                     
    _buttonResetTest = buttonPressed(_buttonResetTest, t_x, t_y, pressed);
    data.buttonResetTestPressed = _buttonResetTest.pressed;
    releaseButton(_buttonResetTest, &_buttonStartTest, &_testRunning);

    _buttonStopTest = buttonPressed(_buttonStopTest, t_x, t_y, pressed);
    data.buttonStopTestPressed = _buttonStopTest.pressed;
    releaseButton(_buttonStopTest, &_buttonStartTest, &_testRunning);

    _buttonStartTest = buttonPressed(_buttonStartTest, t_x, t_y, pressed);
    data.buttonStartTestPreviouslyPressed = data.buttonStartTestPressed;
    data.buttonStartTestPressed = _buttonStartTest.pressed;
    lockButton(&_buttonStartTest, &_testRunning);
    if (_redrawStartTestButton){
      drawButton(_buttonStartTest);
      _redrawStartTestButton = false;
    }
  }

  return data;      
}

/**************************************************************************/
/*!
    @brief Update and Draw the texts for the status tab.
    @param data Stores all updated status data.
*/
/**************************************************************************/
void ControlTFT::drawStatusTab(statusData data) {

  //update values
  _eventPWMSwitchingEvent.bit      = data.eventPWMSwitchingEvent;
  _eventCurrentLimitEvent.bit      = data.eventCurrentLimitEvent;
  _eventPowerLimitEvent.bit        = data.eventPowerLimitEvent;
  _eventSoftResetEvent.bit         = data.eventSoftResetEvent;
  _eventOverCurrentEvent.bit       = data.eventOverCurrentEvent;
  _eventOverttemperatureEvent.bit  = data.eventOverttemperatureEvent;
  _eventVCCUnderVoltageEvent.bit   = data.eventVCCUnderVoltageEvent;
  _eventVDDIOUnderVOltageEvent.bit = data.eventVDDIOUnderVOltageEvent;

  //draw values
  drawEvent(_eventPWMSwitchingEvent);
  drawEvent(_eventCurrentLimitEvent);
  drawEvent(_eventPowerLimitEvent);
  drawEvent(_eventSoftResetEvent);
  drawEvent(_eventOverCurrentEvent);
  drawEvent(_eventOverttemperatureEvent);
  drawEvent(_eventVCCUnderVoltageEvent);
  drawEvent(_eventVDDIOUnderVOltageEvent);
}

/**************************************************************************/
/*!
    @brief Update and Draw the texts for the control tab.
    @param data Stores all updated control data.
*/
/**************************************************************************/
void ControlTFT::drawCommandTab(commandData data) {

  //update events
  _eventPowerStage.bit    = data.eventPowerStage;
  _eventPWMSwitching.bit  = data.eventPWMSwitching;
  _eventLDORegulation.bit = data.eventLDORegulation;

  //draw events
  drawEvent(_eventPowerStage);
  drawEvent(_eventPWMSwitching);
  drawEvent(_eventLDORegulation);

  //update info
  _infoPWMFrequency.bits        = data.infoPWMFrequency;
  _infoPWMAdjust.bits           = data.infoPWMAdjust;
  _infoPWMDutyCycle.bits        = data.infoPWMDutyCycle;
  _infoPeakInductorCurrent.bits = data.infoPeakInductorCurrent;
  _infoPowerLimit.bits          = data.infoPowerLimit;

  //draw info
  drawInfo(_infoPWMFrequency);
  drawInfo(_infoPWMAdjust);
  drawInfo(_infoPWMDutyCycle);
  drawInfo(_infoPeakInductorCurrent);
  drawInfo(_infoPowerLimit);
}

/**************************************************************************/
/*!
    @brief Returns the currently active tab
    @return currently active tab
*/
/**************************************************************************/
PRESSED_TAB ControlTFT::getTab() {
  if (_testsTab.pressed) {
    return PRESSED_TAB::TESTS;
  } else if (_statusTab.pressed) {
    return PRESSED_TAB::STATUS;
  } else if (_commandTab.pressed) {
    return PRESSED_TAB::COMMAND;
  } else {
    return PRESSED_TAB::CONTROL;
  }
}

/**************************************************************************/
/*!
    @brief Resets the controlRunning task by stopping it and releasing 
    changing the start buttons colour to its released state
*/
/**************************************************************************/
void ControlTFT::resetControlRunning() {
  _controlRunning = false;                    //turn control running off
  _buttonStart.backgroundColor = LIGHT_GREY;  //reset the colour of the start button (blue to lightgrey)
  _redrawStartButton = true;                  //set start button pressed to true to redraw th button
}

/**************************************************************************/
/*!
    @brief Resets the testRunning task by stopping it and releasing 
    changing the start test buttons colour to its released state
*/
/**************************************************************************/
void ControlTFT::resetTestRunning() {
  _testRunning = false;                           //turn test running off
  _buttonStartTest.backgroundColor = LIGHT_GREY;  //reset the colour of the start test button (blue to lightgrey)
  _redrawStartTestButton = true;                  //set start test button pressed to true to redraw the button
}

/**************************************************************************/
/*!
    @brief Check and update which tab is pressed.
*/
/**************************************************************************/
bool ControlTFT::checkTouchTabs() {
  uint16_t t_x = 0, t_y = 0;                 // store the touch coordinates
  bool pressed = _tft.getTouch(&t_x, &t_y);  // pressed will be set true is there is a valid touch on the screen

  //check if any tab area is pressed and if tab is already drawn with previousPRedd
  if (pressed & inArea(_controlTab.startX, _controlTab.startY, _controlTab.withX, _controlTab.hightY, t_x, t_y)) {
    bool previousPress = _controlTab.pressed;

    _controlTab.pressed = true;
    _testsTab.pressed =   false;
    _statusTab.pressed =  false;
    _commandTab.pressed = false;

    return previousPress != _controlTab.pressed;
  } else if (pressed & inArea(_testsTab.startX, _testsTab.startY, _testsTab.withX, _testsTab.hightY, t_x, t_y)) {
    bool previousPress = _testsTab.pressed;

    _controlTab.pressed = false;
    _testsTab.pressed =   true;
    _statusTab.pressed =  false;
    _commandTab.pressed = false;

    return previousPress != _testsTab.pressed;
  } else if (pressed & inArea(_statusTab.startX, _statusTab.startY, _statusTab.withX, _statusTab.hightY, t_x, t_y)) {
    bool previousPress = _statusTab.pressed;

    _controlTab.pressed = false;
    _testsTab.pressed =   false;
    _statusTab.pressed =  true;
    _commandTab.pressed = false;

    return previousPress != _statusTab.pressed;
  } else if (pressed & inArea(_commandTab.startX, _commandTab.startY, _commandTab.withX, _commandTab.hightY, t_x, t_y)) {
    bool previousPress = _commandTab.pressed;

    _controlTab.pressed = false;
    _testsTab.pressed =   false;
    _statusTab.pressed =  false;
    _commandTab.pressed = true;

    return previousPress != _commandTab.pressed;
  }

  return false;
}

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
button ControlTFT::buttonPressed(button button, uint16_t t_x, uint16_t t_y, bool pressed) {
  button.previouslyPressed = button.pressed; 

  //if button is pressed mark button as pressed and draw its select colour
  if (pressed & inArea(button.startX, button.startY, button.withX, button.hightY, t_x, t_y)) {
      button.pressed = true;
      drawButton(button);
  } else {

      //only mark button as not pressed and redraw it if the button was previously pressed
      if (button.pressed == true) {
        button.pressed = false;
        drawButton(button);
      }
  }

  return button;
}

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
bool ControlTFT::inArea(int16_t startX, int16_t startY, int16_t withX, int16_t hightY, int16_t touchX, int16_t touchY) {
  return ((touchX >= startX) && (touchX < (startX + withX)) &&
          (touchY >= startY) && (touchY < (startY + hightY)));
}

/**************************************************************************/
/*!
    @brief Changes the colour of the button to its pressed state and starts 
    _controlRunning or _testRunning
    @param button button to be locked
    @param task task to be started
*/
/**************************************************************************/
void ControlTFT::lockButton(button* lock, bool* task) {
  if ((*lock).pressed) {
    *task = true;                   //set the task running to true
    (*lock).backgroundColor = BLUE; //change the background colour of the tasks start button to blue
    drawButton(*lock);              //redraw button
  }
}

/**************************************************************************/
/*!
    @brief Changes the colour of the button to its released state and stops 
    _controlRunning or _testRunning
    @param button button to be released
    @param task task to be stopped
*/
/**************************************************************************/
void ControlTFT::releaseButton(button action, button* release, bool* task) {
  if (action.pressed) {
    *task =false;                             //set the task running to false
    (*release).backgroundColor = LIGHT_GREY;  //change the background colour of the tasks start button to lightgrey
  drawButton(*release);                       //redraw button
  }
}

/**************************************************************************/
/*!
    @brief Draws the button
    @param button button to br drawn
*/
/**************************************************************************/
void ControlTFT::drawButton(button button) {
  if (button.pressed) {
    _button.createSprite(button.withX, button.hightY);
    _button.fillRoundRect(0, 0, button.withX, button.hightY, button.radius, button.selectColor);
    _button.setTextColor(button.textColor, button.selectColor);
    _button.drawString(button.name, button.withX/2, button.hightY/2);
    _button.pushSprite(button.startX, button.startY, TFT_BLACK);
    _button.deleteSprite();
  } else {
    _button.createSprite(button.withX, button.hightY);
    _button.fillRoundRect(0, 0, button.withX, button.hightY, button.radius, button.backgroundColor);
    _button.setTextColor(button.textColor, button.backgroundColor);
    _button.drawString(button.name, button.withX/2, button.hightY/2);
    _button.pushSprite(button.startX, button.startY , TFT_BLACK);
    _button.deleteSprite();
  }
}

/**************************************************************************/
/*!
    @brief Draws the text
    @param text text to be drawn
    @param position position of the text in its field
*/
/**************************************************************************/
void ControlTFT::drawText(text text, uint8_t position) {
  _text.setTextDatum(position);
  _text.createSprite(text.withX, text.hightY);
  _text.fillRoundRect(0, 0, text.withX, text.hightY, 10, text.backgroundColor);
  _text.setTextColor(text.textColor, text.backgroundColor);

  switch (position) {
  case TL_DATUM:
    _text.drawString(text.name, 0, 0);
    break;
  case TC_DATUM:
    _text.drawString(text.name, text.withX/2, 0);
    break;
  case TR_DATUM:
    _text.drawString(text.name, text.withX, 0);
    break;
  case ML_DATUM:
    _text.drawString(text.name, 0, text.hightY/2);
    break;
  case MC_DATUM:
    _text.drawString(text.name, text.withX/2, text.hightY/2);
    break;
  case MR_DATUM:
    _text.drawString(text.name, text.withX, text.hightY/2);
    break;
  case BL_DATUM:
    _text.drawString(text.name, 0, text.hightY);
    break;
  case BC_DATUM:
    _text.drawString(text.name, text.withX/2, text.hightY);
    break;
  case BR_DATUM:
    _text.drawString(text.name, text.withX, text.hightY);
    break;
  default:
    break;
  }

  _text.pushSprite(text.startX, text.startY , TFT_BLACK);
  _text.deleteSprite();
}

/**************************************************************************/
/*!
    @brief Draws the value
    @param value value to be drawn
    @param position position of the text in its field
*/
/**************************************************************************/
void ControlTFT::drawValue(value value, uint8_t position) {
  _value.setTextDatum(position);
  _value.createSprite(value.withX, value.hightY);
  _value.fillRoundRect(0, 0, value.withX, value.hightY, 10, value.backgroundColor);
  _value.setTextColor(value.textColor, value.backgroundColor);

  switch (position) {
  case TL_DATUM:
    _value.drawFloat((float)value.value, 2, 0, 0);
    break;
  case TC_DATUM:
    _value.drawFloat((float)value.value, 2, value.withX/2, 0);
    break;
  case TR_DATUM:
    _value.drawFloat((float)value.value, 2, value.withX, 0);
    break;
  case ML_DATUM:
    _value.drawFloat((float)value.value, 2, 0, value.hightY/2);
    break;
  case MC_DATUM:
    _value.drawFloat((float)value.value, 2, value.withX/2, value.hightY/2);
    break;
  case MR_DATUM:
    _value.drawFloat((float)value.value, 2, value.withX, value.hightY/2);
    break;
  case BL_DATUM:
    _value.drawFloat((float)value.value, 2, 0, value.hightY);
    break;
  case BC_DATUM:
    _value.drawFloat((float)value.value, 2, value.withX/2, value.hightY);
    break;
  case BR_DATUM:
    _value.drawFloat((float)value.value, 2, value.withX, value.hightY);
    break;
  default:
    break;
  }

  _value.pushSprite(value.startX, value.startY , TFT_BLACK);
  _value.deleteSprite();
}

/**************************************************************************/
/*!
    @brief Draws the event
    @param event event to be drawn
*/
/**************************************************************************/
void ControlTFT::drawEvent(event event) {
  _event.createSprite(event.withX, event.hightY);
  
  if (event.bit == 0) {
    _event.fillRoundRect(0, 0, event.withX, event.hightY, 10, event.backgroundColor);
    _event.setTextColor(event.textColor, event.backgroundColor);
    _event.drawString(event.noEvent, event.withX/2, event.hightY/2);
  } else {
    _event.fillRoundRect(0, 0, event.withX, event.hightY, 10, event.eventColor);
    _event.setTextColor(event.textColor, event.eventColor);
    _event.drawString(event.event, event.withX/2, event.hightY/2);
  }

  _event.pushSprite(event.startX, event.startY , TFT_BLACK);
  _event.deleteSprite();
}

/**************************************************************************/
/*!
    @brief Draws the info
    @param info info to be drawn
*/
/**************************************************************************/
void ControlTFT::drawInfo(info info) {
  _info.createSprite(info.withX, info.hightY);
  _info.fillRoundRect(0, 0, info.withX, info.hightY, 10, info.backgroundColor);
  _info.setTextColor(info.textColor, info.backgroundColor);

  switch (info.bits) {
  case 0x0:
    _info.drawString(info.info0, info.withX/2, info.hightY/2);
    break;
  case 0x1:
    _info.drawString(info.info1, info.withX/2, info.hightY/2);
    break;
  case 0x2:
    _info.drawString(info.info2, info.withX/2, info.hightY/2);
    break;
  case 0x3:
    _info.drawString(info.info3, info.withX/2, info.hightY/2);
    break;
  case 0x4:
    _info.drawString(info.info4, info.withX/2, info.hightY/2);
    break;
  case 0x5:
    _info.drawString(info.info5, info.withX/2, info.hightY/2);
    break;
  case 0x6:
    _info.drawString(info.info6, info.withX/2, info.hightY/2);
    break;
  case 0x7:
    _info.drawString(info.info7, info.withX/2, info.hightY/2);
    break;  
  default:
    break;
  }
  
  _info.pushSprite(info.startX, info.startY , TFT_BLACK);
  _info.deleteSprite();
}

/**************************************************************************/
/*!
    @brief Calibrate the orientation of the touchscreen
*/
/**************************************************************************/
void ControlTFT::touchCalibrate()
{
  uint16_t calData[5];
  uint8_t calDataOK = 0;

  // check file system exists
  if (!SPIFFS.begin()) {
    Serial.println("formatting file system");
    SPIFFS.format();
    SPIFFS.begin();
  }

  // check if calibration file exists and size is correct
  if (SPIFFS.exists(CALIBRATION_FILE)) {
    if (REPEAT_CAL)
    {
      // Delete if we want to re-calibrate
      SPIFFS.remove(CALIBRATION_FILE);
    }
    else
    {
      File f = SPIFFS.open(CALIBRATION_FILE, "r");
      if (f) {
        if (f.readBytes((char *)calData, 14) == 14)
          calDataOK = 1;
        f.close();
      }
    }
  }

  if (calDataOK && !REPEAT_CAL) {
    // calibration data valid
    _tft.setTouch(calData);
  } else {
    // data not valid so recalibrate
    _tft.fillScreen(TFT_BLACK);
    _tft.setCursor(20, 0);
    _tft.setTextFont(2);
    _tft.setTextSize(1);
    _tft.setTextColor(TFT_WHITE, TFT_BLACK);

    _tft.println("Touch corners as indicated");

    _tft.setTextFont(1);
    _tft.println();

    if (REPEAT_CAL) {
      _tft.setTextColor(TFT_RED, TFT_BLACK);
      _tft.println("Set REPEAT_CAL to false to stop this running again!");
    }

    _tft.calibrateTouch(calData, TFT_BLUE, TFT_BLACK, 15);

    _tft.setTextColor(TFT_GREEN, TFT_BLACK);
    _tft.println("Calibration complete!");

    // store data
    File f = SPIFFS.open(CALIBRATION_FILE, "w");
    if (f) {
      f.write((const unsigned char *)calData, 14);
      f.close();
    }
  }
}