#ifndef DOLORES_FINGERPRINT_H
#define DOLORES_FINGERPRINT_H

#include <Arduino.h>
#include <Fingerprint.h>
#include <DoloresRelay.h>

class DoloresFingerprint {
  private:
    byte pin_SLA;
    byte pin_SLK;
    DoloresRelay* relay;
    Fingerprint* finger;
    boolean loggedIn;

    int getFingerprintIDez();
  public:
    DoloresFingerprint(byte pin_SLA, byte pin_SLK, byte relay);
    boolean logIn();

};
#endif