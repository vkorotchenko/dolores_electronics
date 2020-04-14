#include "DoloresButton.h"

#define DEBOUNCE_DELAY 50

DoloresButton::DoloresButton(byte pin, byte relayPin) {
    this->pin = pin;
    this->relay = new DoloresRelay(relayPin);
    this->button = new Bounce();

    pinMode(pin, INPUT_PULLUP);
    button->attach(pin);
    button->interval(DEBOUNCE_DELAY);
    button->update();
    checkRelay();
}

byte DoloresButton::readButton() {
    return button->read();
}

void DoloresButton::checkRelay() {
    if (readButton() == LOW) {
        turnOn();
    } else {
        turnOff();
    }
}

boolean DoloresButton::update() {
    return button->update();
}

void DoloresButton::setRelayState( byte state) { 
    state = state;
}

void DoloresButton::turnOn() {
    setRelayState(HIGH);
    relay->turnOn();
}

void DoloresButton::turnOff() {
    setRelayState(LOW);
    relay->turnOff();
}

void DoloresButton::turnRelayOn() {
    relay->turnOn();
}

void DoloresButton::turnRelayOff() {
    relay->turnOff();
}

void DoloresButton::check() {
    boolean isChanged = update();
    if (isChanged) {
        checkRelay();
    }
}

boolean DoloresButton::isOn() {
    return state == HIGH;
}