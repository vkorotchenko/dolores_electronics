#ifndef DOLORES_OIL_H
#define DOLORES_OIL_H
#include <Arduino.h>

class DoloresOil {
  private:
    byte pin;
  public:
    DoloresOil(byte pin) ;
    bool isTriggered() ;
};
#endif