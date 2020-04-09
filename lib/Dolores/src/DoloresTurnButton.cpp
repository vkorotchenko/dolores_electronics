#include "DoloresTurnButton.h"

DoloresTurnButton::DoloresTurnButton (byte pin, byte relayPin) : DoloresButton(pin, relayPin) {
}


void DoloresTurnButton::check() {
    boolean isChanged = update();
    if (isChanged) {
        checkRelay();
    }
    checkBlink();
}

void DoloresTurnButton::checkBlink() {
    if (readButton() == LOW) {
        if (DoloresTimer::isTimeForBlink()) {
            turnRelayOn();
        } else {
            turnRelayOff();
        }
    }
}