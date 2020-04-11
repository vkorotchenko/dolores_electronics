
#include "DoloresVoltage.h"

#define TIME_SINCE_STARTED_THRESHOLD 1000
#define THRESHOLD .8
#define RESISTOR1 30000.0
#define RESISTOR2 7500.0
DoloresVoltage::DoloresVoltage(byte pin) {
    this->pin = pin;
    this->initialValue = analogRead(pin);
    pinMode(pin, INPUT);
}

bool DoloresVoltage::isTriggered() {
    if ( getCurrentVoltage() > THRESHOLD + initialValue) {
        if (millisWhenStarted == 0) {
            millisWhenStarted = millis();
            return false;
        } else if (millisWhenStarted < millis() - TIME_SINCE_STARTED_THRESHOLD) {
            return false;
        } else {
            return true;
        }
    }
    millisWhenStarted = 0;
    return false;
}

int DoloresVoltage::getCurrentVoltage() {
    int value = analogRead(pin);
    float vOUT = (value * 5.0) / 1024.0;
    float vIN = vOUT / (RESISTOR2 / (RESISTOR1 + RESISTOR2));
    return vIN;
}