
#include "DoloresAuxButton.h"

void DoloresAuxButton::turnAltOn() {
    relayAlt->turnOn();
}
void DoloresAuxButton::turnAltOff() {
    relayAlt->turnOff();
}

DoloresAuxButton::DoloresAuxButton (byte pin, byte relayPin, byte relayPinAlt) : DoloresButton(pin, relayPin){

    this->relayAlt = new DoloresRelay(relayPinAlt);
}

void DoloresAuxButton::check(bool useAlt) {
    boolean isChanged = update();
    if (isChanged) {
        checkRelay(useAlt);
    }
}

void DoloresAuxButton::checkRelay(bool useAlt) {
        if (readButton() == LOW) {
            turnOn();
            if(useAlt) {
                turnAltOn();
                turnRelayOff();
            } else {
                turnRelayOn();
                turnAltOff();
            }
        } else {
            turnOff();
            turnAltOff();
        }
    
}