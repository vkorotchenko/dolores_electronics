
#include "DoloresAuxButton.h"

void DoloresAuxButton::turnAltOn() {
    relayAlt->turnOn();
}
void DoloresAuxButton::turnAltOff() {
    relayAlt->turnOff();
}

DoloresAuxButton::DoloresAuxButton (byte pin, byte relayPin, byte relayPinAlt) : DoloresButton(pin, relayPin){
    this->relayAlt = new DoloresRelay(relayPinAlt);
    checkRelay(false);
}

void DoloresAuxButton::check(bool useAlt) {
    boolean isChanged = update();
    if (isChanged) {
        checkRelay(useAlt);
    }
}

void DoloresAuxButton::checkRelay(bool useAlt) {
        if (readButton() == LOW) {
            setRelayState(HIGH);
            if(useAlt) {
                turnAltOn();
                turnRelayOff();
            } else {
                turnAltOff();
                turnOn();
            }
        } else {
            setRelayState(LOW);
            turnOff();
            turnAltOff();
        }
    
}