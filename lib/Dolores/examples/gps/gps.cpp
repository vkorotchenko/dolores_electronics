#include <Arduino.h>
#include <DoloresGPS.h>

#define GPS_TX 2
#define GPS_RX 3

DoloresGPS* gps;
void setup() {
  Serial.begin(9600);
  gps = new DoloresGPS(GPS_TX, GPS_RX);
}

void loop() {
    int displaySpeed = gps->getGps();
    Serial.println("SPEED:" + displaySpeed);
}