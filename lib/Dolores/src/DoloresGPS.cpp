#include "DoloresGPS.h"

#define MIN_SPEED 5
#define RUNNING_DISTANCE_THRESHOLD 1.00
void DoloresGPS::persistRange(float range) {
    DoloresDatabase::updateOdometer(DoloresDatabase::getOdometer() + range);
}

DoloresGPS::DoloresGPS(byte pin_TX, byte pin_RX) {
    this->pin_TX = pin_TX;
    this->pin_RX = pin_RX;


    gpsSerial = new NeoSWSerial(pin_TX, pin_RX);

    // initialize GPS
    gpsSerial->begin(9600);
    gps.send_P( gpsSerial, F("PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")); // RMC(Recomended minim specific) only
    gps.send_P( gpsSerial, F("PMTK220,1000") ); // 4Hz update
}

int DoloresGPS::getRunningDistance() {
    return runningDistance;
}

int DoloresGPS::getGps() {
    int displaySpeed = 0;

    while (gps.available(*gpsSerial)) {
        gps_fix fix = gps.read();
        if (fix.latitude() < 49) {
            DoloresDatabase::setMph();
        } else if (fix.latitude() >= 49) {
            DoloresDatabase::setKph();
        }

        if (fix.valid.speed) {
            int speed;
            if (DoloresDatabase::isMetric()) {
                speed = (int)fix.speed_kph();
            } else {
                speed = (int)fix.speed_mph();
            }

            displaySpeed = speed;
            if (speed < MIN_SPEED) {
                displaySpeed = 0;
            }
        }

        // record distance if location available and traveling more than min speed
        if (fix.valid.location && displaySpeed >= MIN_SPEED) {
            if (firstLocationScan) {
                firstLocationScan = false;
                prev_location = fix.location;
            } else {
                float range = fix.location.DistanceKm (prev_location);
                runningDistance = runningDistance + range;
                if (runningDistance > RUNNING_DISTANCE_THRESHOLD) {
                    persistRange(runningDistance);
                    runningDistance = 0;
                }
            }
        }
    }
    return displaySpeed;
}