#ifndef DOLORES_GPS_H
#define DOLORES_GPS_H

#include <Arduino.h>
#include <NMEAGPS.h>
#include <NeoSWSerial.h>
#include <DoloresDatabase.h>

class DoloresGPS {
  private:
    byte pin_TX;
    byte pin_RX;
    NMEAGPS gps;
    NeoGPS::Location_t prev_location;
    NeoSWSerial* gpsSerial;
    bool firstLocationScan = true;
    float runningDistance = 0;

    void persistRange(float range);
  public:
    DoloresGPS(byte pin_TX, byte pin_RX);
    int getGps();
    int getRunningDistance();
};
#endif