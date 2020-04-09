#include <Arduino.h>
#include <DoloresAuxButton.h>
#include <DoloresTimer.h>

#
#define LED_PIN 13
#define LED_PIN2 12
#define BUTTON 11

DoloresAuxButton* button;

void setup() {
  Serial.begin(9600);
  button = new DoloresAuxButton(BUTTON, LED_PIN, LED_PIN2);
}

void loop() {
  button->check(DoloresTimer::isTimeForBlink());
}
