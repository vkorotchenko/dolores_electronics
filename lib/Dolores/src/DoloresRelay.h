#ifndef DOLORES_RELAY_H
#define DOLORES_RELAY_H
#include <Arduino.h>
class DoloresRelay {
  private:
    byte pin;
  public:
    DoloresRelay(byte pin);
    void turnOn();
    void turnOff();
    void toggle();
    byte getState();
};
#endif