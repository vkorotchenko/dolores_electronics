#include <Arduino.h>
#include <TM1637Display.h>
#include <NMEAGPS.h>
#include <NeoSWSerial.h>

// Module connection pins (Digital Pins)
#define CLK 2
#define DIO 3
#define GPS_TX 8
#define GPS_RX 7

// CONSTANTS
#define MIN_SPEED 5

// VARIABLES
boolean isMetric = true;

// declare GPS
NeoSWSerial gpsSerial(GPS_TX, GPS_RX);
NMEAGPS gps;


// declate LED Display
TM1637Display display(CLK, DIO);

void setup()
{
  // Display settings
  display.setBrightness(0x09);
  setKph();

  //init serial
  Serial.begin(115200);

  // initialize GPS
  gpsSerial.begin(9600);
  gps.send_P( &gpsSerial, F("PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0") ); // RMC(Recomended minim specific) only
  gps.send_P( &gpsSerial, F("PMTK220,1000") ); // 4Hz update
}

void loop()
{
  while (gps.available( gpsSerial ))
  {
    gps_fix fix = gps.read();

    if (fix.latitude() < 49 && isMetric) {
      setMph();
    } else if (fix.latitude() >= 49 && !isMetric) {
      setKph();
    }

    if (fix.valid.speed) {

      int speed;
      if (isMetric) {
        speed = (int)fix.speed_kph();
      } else {
        speed = (int)fix.speed_mph();
      }

      if (speed > MIN_SPEED) {
        setSpeed(speed);
      } else {
        setSpeed(0);
      }

    } 
  }
}

void setSpeed(int speed) {
  display.showNumberDec(speed, true, 3, 0);
}

void setKph() {
  uint8_t kph[] = { 0x00, 0x00, 0x00, SEG_A };
  display.setSegments(kph);
  isMetric = true;
}

void setMph() {
  uint8_t kph[] = { 0x00, 0x00, 0x00, SEG_D };
  display.setSegments(kph);
  isMetric = false;
}
