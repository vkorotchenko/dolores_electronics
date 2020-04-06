
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
}

void DoloresButton::turnOn() {
    relay->turnOn();
}

void DoloresButton::turnOff() {
    relay->turnOff();
}

boolean DoloresButton::isTriggered() {
    bool changed = button->update();
    return (changed && button->read() == LOW);
}

boolean DoloresButton::isReleased() {
    bool changed = button->update();
    return (changed && button->read() == HIGH);
}

void DoloresButton::check() {
    if (isTriggered()) {
        turnOn();
    } else if (isReleased()) {
        turnOff();
    }
}

boolean DoloresButton::isOn() {
    return relay->getState() == HIGH;
}
