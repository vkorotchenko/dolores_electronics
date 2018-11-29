#include <Arduino.h>
#include <TM1637Display.h>
#include <SoftwareSerial.h>
#include <NMEAGPS.h>

// Module connection pins (Digital Pins)
#define CLK 2
#define DIO 3
#define GPS_TX 8
#define GPS_RX 7

SoftwareSerial gpsSerial(GPS_TX, GPS_RX);
NMEAGPS gps;


uint8_t data[] = { 0xff, 0xff, 0xff, 0xff };

TM1637Display display(CLK, DIO);
void setup()
{  
  display.setBrightness(0x09);
  setMph();
  
  Serial.begin(115200);
  Serial.println( F("Clock starting!") ); // F macro saves RAM!


  gpsSerial.begin(9600);
  gps.send_P( &gpsSerial, F("PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0") ); // RMC only
  gps.send_P( &gpsSerial, F("PMTK220,1000") ); // 1Hz update
}

void loop()
{
while (gps.available( gpsSerial ))
  {

    //  A new fix has been assembled from processed chars
    gps_fix fix = gps.read();

    //  Display fix status (the status may not be valid yet)
    Serial.println( "GPS RECEIVED");

    //  Display fix speed (the speed may not be valid yet)
    if (fix.valid.speed) {

      // Here is a way to get the whole number MPH
      int speed_mph = (fix.spd.whole * 115) / 100;

      // You could get the floating point (with decimals) MPH like this:
      //float speed_mph = fix.speed_mph();
      //speed_mph = random( 0, 100 ); // uncomment for testing
      Serial.print("speed: ");
      Serial.println(speed_mph);
      setSpeed(speed_mph);

    } else {
      Serial.print("invalid speed");
    }
    Serial.println();
  }

  
// int speed = random( 0, 999 );
//setSpeed(speed);
//delay(1000);
}

void setSpeed(int speed) {
  display.showNumberDec(speed, true, 3, 0); 
}

void setKph() {
  uint8_t kph[] = { 0x00, 0x00, 0x00, SEG_A };
  display.setSegments(kph);
  
}

void setMph() {
  uint8_t kph[] = { 0x00, 0x00, 0x00, SEG_D };
  display.setSegments(kph);
  
}
