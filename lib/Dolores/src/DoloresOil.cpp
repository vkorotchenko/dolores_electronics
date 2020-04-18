#include "DoloresOil.h"
DoloresOil::DoloresOil(byte pin) {
    this->pin = pin;
    pinMode(pin, INPUT_PULLUP);
}

bool DoloresOil::isTriggered() {
    return (digitalRead(pin) == HIGH);
}