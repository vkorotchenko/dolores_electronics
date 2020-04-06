
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
    if (isTriggered()) {
        if (useAlt) {
            turnAltOn();
        } else {
            turnOn();
        }
    } else if (isReleased()) {
        turnOff();
        turnAltOff();
    }
}

boolean DoloresAuxButton::isAltOn() {
    return relayAlt->getState() == HIGH;
}