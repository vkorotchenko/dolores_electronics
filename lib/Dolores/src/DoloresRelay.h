#ifndef DOLORES_RELAY_H
#define DOLORES_RELAY_H
#include <Arduino.h>
class DoloresRelay {
  private:
    byte pin;
    byte logic;
    byte getState();
  public:
    DoloresRelay(byte pin);
    DoloresRelay(byte pin, byte logic);
    void turnOn();
    void turnOff();
    void toggle();
};
#endif