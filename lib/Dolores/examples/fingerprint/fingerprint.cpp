#include <Arduino.h>
#include <DoloresFingerprint.h>

// LED pin will turn on when fingerprint tis logged on

#define FINGERPRINT_SLA 0
#define FINGERPRINT_SLK 1
#define LED_PIN 13

DoloresFingerprint* fingerprint;

void setup() {
  fingerprint = new DoloresFingerprint(FINGERPRINT_SLA, FINGERPRINT_SLK, LED_PIN);
}

void loop() {
    boolean isLoggedIn = fingerprint->logIn();
}