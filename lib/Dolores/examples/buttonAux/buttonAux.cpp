#include <Arduino.h>
#include <DoloresAuxButton.h>
#include <DoloresTimer.h>

#define LED_PIN 13
#define LED_PIN2 12
#define BUTTON 4

DoloresAuxButton* button;

void setup() {
  Serial.begin(9600);
  button = new DoloresAuxButton(BUTTON, LED_PIN, LED_PIN2);
}

void loop() {
  Serial.println("BEGIN: " + (String)(DoloresTimer::isTimeForBlink()? " ALT " : "REG"));
  button->check(DoloresTimer::isTimeForBlink());
  boolean isOn = button->isOn();
  boolean isAltOn = button->isAltOn();

  Serial.println("RELAY IS" + (String)(isOn ? "ON":"OFF"));
  Serial.println("ALT RELAY IS" + (String)(isAltOn ? "ON":"OFF"));
}