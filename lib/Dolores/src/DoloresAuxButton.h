#ifndef DOLORES_BUTTON_AUX_H
#define DOLORES_BUTTON_AUX_H

#include <Arduino.h>
#include <DoloresButton.h>

class DoloresAuxButton : public DoloresButton {
  private:
    DoloresRelay* relayAlt;

    void turnAltOn();
    void turnAltOff() ;
  public:
    DoloresAuxButton (byte pin, byte relayPin, byte relayPinAlt);
    void check(bool useAlt);
    boolean isAltOn();
};
#endif