#include <Arduino.h>
#include <DoloresVoltage.h>

#define VOLT_SENSOR A3

DoloresVoltage* volt;

void setup() {
  Serial.begin(9600);
  volt = new DoloresVoltage(VOLT_SENSOR);
}

void loop() {
  Serial.println("CURRENT VOLTAGE: " + volt->getCurrentVoltage());
  Serial.println("IS TRIGGERED: " + volt->isTriggered());
}