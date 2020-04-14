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
    byte state;
  public:
    boolean update();
    DoloresButton(byte pin, byte relayPin);
    void turnOn();
    void turnOff();
    void turnRelayOn();
    void turnRelayOff();
    virtual void check();
    boolean isOn();
    void checkRelay();
    byte readButton();
    void setRelayState( byte state);
};
#endif