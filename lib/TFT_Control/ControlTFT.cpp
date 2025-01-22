/*
 * File Name: TFT_Control.cpp
 * Description: Functions to control the TFT Display.
 *
 * Notes: This code was written as part of my master's thesis at the 
 *        Institute for Microsensors, -actuators and -systems (IMSAS) 
 *        at the University of Bremen.
 */

#include "ControlTFT.h"

ControlTFT::ControlTFT() : _tabSelect(&_tft), _background(&_tft), _button(&_tft), _display(&_tft), _text(&_tft) {
  _tft.init();
  _tft.setRotation(1); 
}


void ControlTFT::begin(){
  touchCalibrate();
  _tft.fillScreen(TFT_BLACK);
  _tft.fillRoundRect(0, 0, 480, 320, 15, LIGHT_GREY);

  _tabSelect.setTextDatum(4);
  _background.setTextDatum(4);
  _button.setTextDatum(4);
  _display.setTextDatum(4);
  _text.setTextDatum(4);

  _tabSelect.setTextSize(2);
  _background.setTextSize(2);
  _button.setTextSize(2);
  _display.setTextSize(2);
  _text.setTextSize(2);

  _controlTab = {5, 5, 113, 35, 10, BLACK, LIGHT_GREY, GREY, true};
  _testsTab   = {124, 5, 113, 35, 10, BLACK, LIGHT_GREY, GREY, false};
  _statusTab  = {243, 5, 113, 35, 10, BLACK, LIGHT_GREY, GREY, false};
  _commandTab = {362, 5, 113, 35, 10, BLACK, LIGHT_GREY, GREY, false};

  _startButton               = {"Start", 384, 249, 70, 45, 10, BLACK, LIGHT_GREY, GREY, false};
  _stopButton                = {"Stop", 308, 249, 70, 45, 10, BLACK, LIGHT_GREY, GREY, false};
  _resetButton               = {"Reset", 232, 249, 70, 45, 10, BLACK, LIGHT_GREY, GREY, false};
  _increaseTemperatureButton = {"Lower", 57, 51, 45, 60, 10, BLACK, LIGHT_GREY, GREY, false};
  _decreaseTemperatureButton = {"Higher", 0, 51, 45, 60, 10, BLACK, LIGHT_GREY, GREY, false};

  _LT8722Monitor   = {"Start", 384, 249, 70, 45, BLACK, TFT_WHITE};
  _chipTemperature = {"Start", 384, 249, 70, 45, BLACK, TFT_WHITE};
  _outputVoltage   = {"Start", 384, 249, 70, 45, BLACK, TFT_WHITE};
  _outputCurrent   = {"Start", 384, 249, 70, 45, BLACK, TFT_WHITE};
  _setTemperature  = {"Start", 384, 249, 70, 45, BLACK, TFT_WHITE};
  _output          = {"Output", 26, 243, 80, 57, BLACK, TFT_WHITE};

  _valueChipTemperature    = {"Start", 384, 249, 70, 45, BLACK, TFT_WHITE};
  _valueOutputVoltage      = {"Start", 384, 249, 70, 45, BLACK, TFT_WHITE};
  _valueOutputCurrent      = {"Start", 384, 249, 70, 45, BLACK, TFT_WHITE};
  _valueSetTemperature     = {"Start", 384, 249, 70, 45, BLACK, TFT_WHITE};
  _valueCurrentTemperature = {"Start", 384, 249, 70, 45, BLACK, TFT_WHITE};
}

void ControlTFT::drawTabSelect(bool init) {
  bool changedTab = checkTouchTabs();

  if (changedTab || init) {
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
      _background.fillRoundRect(0, 0, 480, 260, 15, LIGHT_GREY);
      _background.fillRoundRect(20, 0, 440, 114, 10, TFT_WHITE);
      _background.fillRoundRect(20, 120, 440, 57, 10, TFT_WHITE);
      _background.fillRoundRect(20, 183, 440, 57, 10, TFT_WHITE);
      _background.drawString("LT8722 Monitor:", 118, 20);
      _background.drawString("Temperature", 93, 55);
      _background.drawString("Voltage", 240, 55);
      _background.drawString("Current", 386, 55);
      _background.drawString("Temperature:", 98, 148);
      _background.pushSprite(0, 60, TFT_BLACK);
      _background.deleteSprite();

      drawText(_output, ML_DATUM);

      drawButton(_resetButton);
      drawButton(_stopButton);
      drawButton(_startButton);
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
      _background.fillRoundRect(0, 0, 480, 260, 15, LIGHT_GREY);
      _background.pushSprite(0, 60, TFT_BLACK);
      _background.deleteSprite();
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
      _background.fillRoundRect(0, 0, 480, 260, 15, LIGHT_GREY);
      _background.pushSprite(0, 60, TFT_BLACK);
      _background.deleteSprite();
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
      _background.fillRoundRect(0, 0, 480, 260, 15, LIGHT_GREY);
      _background.pushSprite(0, 60, TFT_BLACK);
      _background.deleteSprite();
    }
  }
}

/**************************************************************************/
/*!
    @brief Calculate the CRC for two bytes
    @param data Data for the CRC calculation
    @return CRC value
*/
/**************************************************************************/
void ControlTFT::drawControlTab() {
  double temperature = 10.0;
  double voltage = 2.1;
  double current = 1.2;

  //draw temperature, voltage and current to LT8722 Monitor field
  _display.createSprite(440, 57);
  _display.fillRoundRect(0, 5, 440, 38, 10, TFT_WHITE);
  _display.setTextColor(_startButton.textColor, TFT_WHITE);
  _display.drawFloat((float)temperature, 2, 73, 29);
  _display.drawFloat((float)voltage, 2, 220, 29);
  _display.drawFloat((float)current, 2, 366, 29);
  _display.pushSprite(20, 120, TFT_BLACK);
  _display.deleteSprite();

  //draw ste temperature and current temperature to temperature field
  _display.createSprite(150, 57);
  _display.fillRoundRect(0, 0, 140, 57, 10, TFT_WHITE);
  _display.setTextColor(_startButton.textColor, TFT_WHITE);
  _display.drawFloat((float)temperature, 2, 30, 29);
  _display.drawFloat((float)temperature, 2, 120, 29);
  _display.pushSprite(180, 180, TFT_BLACK);
  _display.deleteSprite();

  _button.createSprite(102, 57);
  _button.fillRoundRect(0, 0, 140, 57, 10, TFT_WHITE);
  _button.setTextColor(_startButton.textColor, TFT_WHITE);
  _button.drawFloat((float)temperature, 2, 30, 29);
  _button.drawFloat((float)temperature, 2, 120, 29);
  _button.pushSprite(180, 180, TFT_BLACK);
  _button.deleteSprite();

  uint16_t t_x = 0, t_y = 0;                 // To store the touch coordinates
  bool pressed = _tft.getTouch(&t_x, &t_y);  // Pressed will be set true is there is a valid touch on the screen

}

void ControlTFT::drawTestsTab() {

}

void ControlTFT::drawStatusTab() {

}

void ControlTFT::drawCommandTab() {

}

bool ControlTFT::checkTouchTabs() {
  uint16_t t_x = 0, t_y = 0;                 // To store the touch coordinates
  bool pressed = _tft.getTouch(&t_x, &t_y);  // Pressed will be set true is there is a valid touch on the screen

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

bool ControlTFT::checkButtonPressed(button button, uint16_t t_x, uint16_t t_y, bool pressed) { 
  if (pressed & inArea(button.startX, button.startY, button.withX, button.hightY, t_x, t_y)) {
    return true;
  } else {
    return false;
  }
}

bool ControlTFT::inArea(int16_t startX, int16_t startY, int16_t withX, int16_t hightY, int16_t touchX, int16_t touchY) {
  return ((touchX >= startX) && (touchX < (startX + withX)) &&
          (touchY >= startY) && (touchY < (startY + hightY)));
}

void ControlTFT::drawButton(button button) {
  _button.createSprite(button.withX, button.hightY);

  if (button.pressed) {
    _button.fillRoundRect(0, 0, button.withX, button.hightY, button.radius, button.selectColor);
    _button.setTextColor(_startButton.textColor, button.selectColor);
    _button.drawString(button.name, button.withX/2, button.hightY/2);
    _button.pushSprite(button.startX, button.startY, TFT_BLACK);
    _button.fillRoundRect(0, 0, button.withX, button.hightY, button.radius, button.backgroundColor);
    _button.setTextColor(_startButton.textColor, button.backgroundColor);
    _button.drawString(button.name, button.withX/2, button.hightY/2);
    _button.pushSprite(button.startX, button.startY, TFT_BLACK);
    _button.deleteSprite();
  } else {
    _button.fillRoundRect(0, 0, button.withX, button.hightY, button.radius, button.backgroundColor);
    _button.setTextColor(_startButton.textColor, button.backgroundColor);
    _button.drawString(button.name, button.withX/2, button.hightY/2);
    _button.pushSprite(button.startX, button.startY , TFT_BLACK);
    _button.deleteSprite();
  }
}

void ControlTFT::drawText(text text, uint8_t position) {
  _text.setTextDatum(position);
  _text.createSprite(text.withX, text.hightY);
  _text.fillRoundRect(0, 0, text.withX, text.hightY, 10, text.backgroundColor);
  _text.setTextColor(_startButton.textColor, text.backgroundColor);

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