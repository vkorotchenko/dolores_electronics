#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>


// comment out TEST_VAL to upload prod
// #define TESTING 1

//IMPORT MODULES
#include <DoloresTimer.h>
#include <DoloresDatabase.h>
#include <DoloresRelay.h>
#include <DoloresOil.h>
#include <DoloresVoltage.h>
#include <DoloresButton.h>
#include <DoloresAuxButton.h>
#include <DoloresTurnButton.h>
#include <DoloresDisplay.h>
#include <DoloresFingerprint.h>
#include <DoloresGPS.h>


//DEFINE PINS
#define FINGERPRINT_SLA 0
#define FINGERPRINT_SLK 1
#define CLK_DISPLAY A4
#define DIO_DISPLAY A5
#define GPS_TX 2
#define GPS_RX 3

#define RIGHT_TURN_IN 10
#define LEFT_TURN_IN 11
#define HEAD_LIGHT_IN 12
#define AUX_IN 9

#define OIL_SENSOR A0
#define VOLT_SENSOR A3

#define FINGERPRINT_RELAY 13
#define HORN_RELAY 7
#define RIGHT_TURN_RELAY 4
#define LEFT_TURN_RELAY 5
#define HEAD_LIGHT_RELAY 6
#define STARTER_RELAY 8

#ifndef TESTING

//DEFINE OBJECTS
DoloresTurnButton* buttonLeft;
DoloresTurnButton* buttonRight;
DoloresButton* buttonHeadLight;
DoloresAuxButton* buttonAux;

DoloresGPS* gps;
DoloresFingerprint* fingerprint;
DoloresDisplay* screen;
DoloresOil* oil;
DoloresVoltage* volt;


void setup() {
  //INIT OBJECTS
  buttonLeft = new DoloresTurnButton(LEFT_TURN_IN, LEFT_TURN_RELAY);
  buttonRight = new DoloresTurnButton(RIGHT_TURN_IN, RIGHT_TURN_RELAY);
  buttonHeadLight = new DoloresButton(HEAD_LIGHT_IN, HEAD_LIGHT_RELAY);
  buttonAux = new DoloresAuxButton(AUX_IN, STARTER_RELAY, HORN_RELAY);

  gps = new DoloresGPS(GPS_TX, GPS_RX);
  fingerprint = new DoloresFingerprint(FINGERPRINT_SLA, FINGERPRINT_SLK, FINGERPRINT_RELAY);
  screen = new DoloresDisplay(CLK_DISPLAY, DIO_DISPLAY);
  oil = new DoloresOil(OIL_SENSOR);
  volt = new DoloresVoltage(VOLT_SENSOR);

  // ELECTRONICS TEST
  buttonLeft->turnOn();
  buttonRight->turnOn();

  // DISPLAY ODOMETER
  screen->scrollOdometer(DoloresDatabase::getOdometer());

  buttonLeft->turnOff();
  buttonRight->turnOff();
}

void loop() {
  // LOGIN
  boolean isLoggedIn = fingerprint->logIn();
  if (isLoggedIn) {
    screen->scrollText("WELCOME VADIM");
  }

  // GET GPS DATA
  int displaySpeed = gps->getGps();
  
  // CHECK OIL SENSOR
  boolean isOil =  oil->isTriggered();

  // CHECK RUNNING
  boolean isRunning = volt->isTriggered();

  // CHECK INPUTS
  buttonLeft->check();
  buttonRight->check();
  buttonHeadLight->check();
  buttonAux->check(isRunning);

  // DISPLAY TO SCREEN
  screen->setDisplay(isOil, isRunning, buttonLeft->isOn(), buttonRight->isOn(), displaySpeed);
}

// TESTED: TIMER, DATABASE, Button, turn, aux, oil, display (numeric), fingerprint, gps. display alpha
// TIMER
#elif TESTING == 1
#endif
