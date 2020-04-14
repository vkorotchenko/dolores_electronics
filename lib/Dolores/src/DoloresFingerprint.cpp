#include "DoloresFingerprint.h"

int DoloresFingerprint::getFingerprintIDez() {
    uint8_t p = finger->getImage();
    if (p != FINGERPRINT_OK)  return -1;

    p = finger->image2Tz();
    if (p != FINGERPRINT_OK)  return -1;

    p = finger->fingerFastSearch();
    if (p != FINGERPRINT_OK)  return -1;

    return finger->fingerID;
}

DoloresFingerprint::DoloresFingerprint(byte pin_SLA, byte pin_SLK, byte relay) {
    this->pin_SLA = pin_SLA;
    this->pin_SLK = pin_SLK;
    this->relay = new DoloresRelay(relay, LOW);
    this->finger = new Fingerprint(&Serial);
    this->loggedIn = false;

    finger->begin(57600);
}

boolean DoloresFingerprint::logIn() {
    while (!loggedIn) {
        int id = getFingerprintIDez();
        if (id > 0 && id < 11) {
            relay->turnOn();
            loggedIn = true;
            return true;
        }
    }
    return false;
}