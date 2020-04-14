#ifndef DOLORES_VOLT_H
#define DOLORES_VOLT_H
#include <Arduino.h>

class DoloresVoltage {
  private:
    byte pin;
    float initialValue;
    unsigned long millisWhenStarted = 0;
    boolean isRunning;
  public:
    DoloresVoltage(byte pin) ;
    bool isTriggered();
    float getCurrentVoltage();
    boolean isLowVoltage();
};
#endif