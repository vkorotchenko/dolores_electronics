#include "DoloresTurnButton.h"

DoloresTurnButton::DoloresTurnButton (byte pin, byte relayPin) : DoloresButton(pin, relayPin) {
}

void DoloresTurnButton::check() {
    if (isTriggered()) {
    if (DoloresTimer::isTimeForBlink()) {
        turnOn();
    } else {
        turnOff();
    }
    } else if (isReleased()) {
    turnOff();
    }
}