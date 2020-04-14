#include "DoloresRelay.h"

DoloresRelay::DoloresRelay(byte pin) {
    this->pin = pin;
    this->logic = HIGH;
    pinMode(pin, OUTPUT);
    turnOff();
}
DoloresRelay::DoloresRelay(byte pin, byte logic) {
    this->pin = pin;
    this->logic = logic;
    pinMode(pin, OUTPUT);
    turnOff();
}

void DoloresRelay::turnOn() {
    if( logic == HIGH) {
        digitalWrite(pin, LOW);
    } else {
        digitalWrite(pin, HIGH);
    }
}

void DoloresRelay::turnOff() {
    if( logic == HIGH) {
        digitalWrite(pin, HIGH);
    } else {
        digitalWrite(pin, LOW);
    }

}

void DoloresRelay::toggle() {
    digitalWrite(pin, !getState());
}

byte DoloresRelay::getState() {
    return digitalRead(pin);
}
