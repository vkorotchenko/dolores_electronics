#include <Arduino.h>
#include <DoloresButton.h>

#define LED_PIN 13
#define BUTTON 4

DoloresButton* button;

void setup() {
  Serial.begin(9600);
  button = new DoloresButton(BUTTON, LED_PIN);
}

void loop() {
  button->check();
  Serial.println("RELAY IS" + (String)(button->isOn() ? "ON":"OFF"));
}