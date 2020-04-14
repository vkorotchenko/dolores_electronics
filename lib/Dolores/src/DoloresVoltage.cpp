
#include "DoloresVoltage.h"

#define TIME_SINCE_STARTED_THRESHOLD 1000
#define THRESHOLD .3
#define RESISTOR1 30000.0
#define RESISTOR2 7500.0
#define LOW_VOLT_THRESHOLD 11.8


DoloresVoltage::DoloresVoltage(byte pin) {
    this->pin = pin;
    this->isRunning = false;
    pinMode(pin, INPUT);
    this->initialValue = getCurrentVoltage();
}

bool DoloresVoltage::isTriggered() {
    if (isRunning == true) {
        return isRunning;
    }
    if ( getCurrentVoltage() > (THRESHOLD + initialValue)) {
        if (millisWhenStarted == 0) {
            millisWhenStarted = millis();
            isRunning =  false;
        } else if (millisWhenStarted < millis() - TIME_SINCE_STARTED_THRESHOLD) {
            isRunning =  false;
        } else {
            isRunning = true;
        }
    }
    millisWhenStarted = 0;
    return isRunning;
}

float DoloresVoltage::getCurrentVoltage() {
    int value = analogRead(pin);
    float vOUT = (value * 5.0) / 1024.0;
    float vIN = vOUT / (RESISTOR2 / (RESISTOR1 + RESISTOR2));
    return vIN;
}

boolean DoloresVoltage::isLowVoltage() {
    return getCurrentVoltage() < LOW_VOLT_THRESHOLD;
}