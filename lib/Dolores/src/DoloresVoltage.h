#ifndef DOLORES_VOLT_H
#define DOLORES_VOLT_H
#include <Arduino.h>

class DoloresVoltage {
  private:
    byte pin;
    int initialValue;
    unsigned long millisWhenStarted = 0;
  public:
    DoloresVoltage(byte pin) ;
    bool isTriggered();
    int getCurrentVoltage();
};
#endif