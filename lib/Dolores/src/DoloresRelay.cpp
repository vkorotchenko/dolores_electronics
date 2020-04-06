#include "DoloresRelay.h"

DoloresRelay::DoloresRelay(byte pin) {
    this->pin = pin;
    pinMode(pin, OUTPUT);
}

void DoloresRelay::turnOn() {
    digitalWrite(pin, HIGH);
}

void DoloresRelay::turnOff() {
    digitalWrite(pin, LOW);
}

void DoloresRelay::toggle() {
    digitalWrite(pin, !getState());
}

byte DoloresRelay::getState() {
    return digitalRead(pin);
}
