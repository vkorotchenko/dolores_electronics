#ifndef DOLORES_BUTTON_TURN_H
#define DOLORES_BUTTON_TURN_H

#include <Arduino.h>
#include <DoloresButton.h>
#include <DoloresTimer.h>

class DoloresTurnButton : public DoloresButton {
  public:
    DoloresTurnButton (byte pin, byte relayPin) ;
    void checkBlink();
    void check();
};
#endif