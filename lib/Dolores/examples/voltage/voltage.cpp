#include <Arduino.h>
#include <DoloresVoltage.h>

#define VOLT_SENSOR A3
#define ONBOARD_LED 13

DoloresVoltage* volt;

void setup() {
  Serial.begin(9600);
  volt = new DoloresVoltage(VOLT_SENSOR);
}

void loop() {
  if(volt->isTriggered()) {
    digitalWrite(ONBOARD_LED, HIGH);
  }
}