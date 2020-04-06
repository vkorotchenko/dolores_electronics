
#include <Arduino.h>
#include "DoloresTimer.h"

#define LED_PIN 13

void setup() {
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
    if(DoloresTimer::isTimeForBlink()) {
        digitalWrite(LED_PIN, HIGH);
    } else {
        digitalWrite(LED_PIN, HIGH);
    }
}