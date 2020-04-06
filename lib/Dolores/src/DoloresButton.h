#ifndef DOLORES_BUTTON_H
#define DOLORES_BUTTON_H
#include <Arduino.h>
#include <DoloresRelay.h>
#include <Bounce2.h>

class DoloresButton {
  private:
    byte pin;
    Bounce* button;
    DoloresRelay* relay;

  public:
    DoloresButton(byte pin, byte relayPin) ;
    void turnOn();
    void turnOff();
    boolean isTriggered();
    boolean isReleased();
    virtual void check();
    boolean isOn() ;
};
#endif