
#include <Arduino.h>
#include "DoloresTimer.h"

#define ONBOARD_LED 13

void setup() {
  pinMode(ONBOARD_LED, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if(DoloresTimer::isTimeForBlink()) {
      digitalWrite(ONBOARD_LED, HIGH);
      Serial.println("ON");
  } else {
      digitalWrite(ONBOARD_LED, LOW);
      Serial.println("OFF");
  }
}