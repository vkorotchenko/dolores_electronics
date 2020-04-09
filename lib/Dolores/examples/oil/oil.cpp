#include <Arduino.h>
#include <DoloresOil.h>

#define OIL_SENSOR A0

DoloresOil* oil;

void setup() {
  Serial.begin(9600);
  oil = new DoloresOil(OIL_SENSOR);
}

void loop() {
  Serial.println("IS OIL TRIGGERED: " + (String)oil->isTriggered());
}