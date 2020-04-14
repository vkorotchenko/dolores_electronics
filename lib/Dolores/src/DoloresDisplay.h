#ifndef DOLORES_DISPLAY_H
#define DOLORES_DISPLAY_H

#include <Arduino.h>
#include <Fingerprint.h>
#include <TM1637Display.h>
#include <Adafruit_LEDBackpack.h>
#include <Adafruit_GFX.h>
#include <DoloresDatabase.h>
#include <DoloresTimer.h>

class DoloresDisplay {
  private:
    byte pin_CLK_SDA;
    byte pin_DIO_SCL;
    boolean isAlphanumeric;
    Adafruit_AlphaNum4 alpha4;
    TM1637Display* numeric4;


    void displayOdometerAlphanumeric(float input);
    void scrollAlphaNumericDisaplay(String input);
    void digitalDisplay(boolean isOil, boolean isRunning, boolean isLeftTurn, boolean isRightTurn, int displaySpeed, boolean isLowVolt);
    void alphaNumericDisplay(boolean isOil, boolean isRunning, boolean isLeftTurn, boolean isRightTurn, int displaySpeed, boolean isLowVolt);
    void displayOdometerDigital (float input);
    int getDisplayValue(int digits, int offset , long reading);
    int extractDigit(long v, int p) ;
    void displayAlphaChars(char a, char b, char c, char d);
  public:
    DoloresDisplay(byte pin_CLK_SDA, byte pin_DIO_SCL, boolean isAlphanumeric);
    DoloresDisplay(byte pin_CLK_SDA, byte pin_DIO_SCL);
    void scrollText(String value);
    void scrollOdometer(float odometer);
    void setDisplay(boolean isOil, boolean isRunning, boolean isLeftTurn, boolean isRightTurn, int displaySpeed, boolean isLowVolt);
};
#endif