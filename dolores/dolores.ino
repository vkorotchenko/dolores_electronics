#include <Arduino.h>
#include <TM1637Display.h>
#include <NMEAGPS.h>
#include <NeoSWSerial.h>
#include <Bounce2.h>

// Module connection pins (Digital Pins)
#define CLK 2
#define DIO 3
#define GPS_TX 8
#define GPS_RX 7
#define RIGHT_TURN_IN 4
#define RIGHT_TURN_RELAY 5
#define LEFT_TURN_IN 6
#define LEFT_TURN_RELAY 9
#define HEAD_LIGHT_IN 10
#define HEAD_LIFGHT_RELAY 11
#define STARTER_IN 12
#define STARTER_RELAY 13
#define HORN_IN A0
#define HORN_RELAY A1

// CONSTANTS
#define MIN_SPEED 5
#define DEBOUNCE_DELAY 50
#define TURN_SIGNAL_BLINK_DELAY 500


// VARIABLES
bool isMetric = true;
bool enableBothTurnSignals = false;

bool isLeftTurnOn = false;
bool isRightTurnOn = false;
bool isHeadlightOn = false;

//decleare Bounce
Bounce debouncerRightTurn = Bounce();
Bounce debouncerLeftTurn = Bounce();
Bounce debouncerHeadlight = Bounce();



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

  // set pin modes
  // INPUT
  pinMode(RIGHT_TURN_IN, INPUT);
  pinMode(LEFT_TURN_IN, INPUT);
  pinMode(HEAD_LIGHT_IN, INPUT);
  pinMode(STARTER_IN, INPUT);
  pinMode(HORN_IN, INPUT);

  //init bounce
  debouncerRightTurn.attach( RIGHT_TURN_IN );
  debouncerRightTurn.interval(DEBOUNCE_DELAY);
  debouncerLeftTurn.attach( LEFT_TURN_IN );
  debouncerLeftTurn.interval(DEBOUNCE_DELAY);
  debouncerHeadlight.attach( HEAD_LIGHT_IN );
  debouncerHeadlight.interval(DEBOUNCE_DELAY);


  //OUTPUT
  pinMode(RIGHT_TURN_RELAY, OUTPUT);
  pinMode(LEFT_TURN_RELAY, OUTPUT);
  pinMode(HEAD_LIFGHT_RELAY, OUTPUT);
  pinMode(STARTER_RELAY, OUTPUT);
  pinMode(HORN_RELAY, OUTPUT);

}

void loop()
{
  // read GPS if available
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

  debouncerRightTurn.update ( );
  debouncerLeftTurn.update ( );
  debouncerHeadlight.update ( );

  //read left turn
  if (debouncerLeftTurn.read() == HIGH) {
    isLeftTurnOn = !isLeftTurnOn;
    if (isLeftTurnOn && !enableBothTurnSignals) {
      isRightTurnOn = false;
    }
  }
  blinkTurnSignal(isLeftTurnOn, LEFT_TURN_RELAY);

  //read right turn
  if (debouncerRightTurn.read() == HIGH) {
    isRightTurnOn = !isRightTurnOn;
    if (isRightTurnOn && !enableBothTurnSignals) {
      isLeftTurnOn = false;
    }
  }
  blinkTurnSignal(isRightTurnOn, RIGHT_TURN_RELAY );

  // read headlight
  if (debouncerHeadlight.read() == HIGH) {
    isHeadlightOn = !isHeadlightOn;
  }
}

void blinkTurnSignal(bool isOn, int pin) {
  if (isOn) {
    if ((millis() % (2 * TURN_SIGNAL_BLINK_DELAY)) < TURN_SIGNAL_BLINK_DELAY) {
      digitalWrite(pin, HIGH);
    } else {
      digitalWrite(pin, LOW);
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
