#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Bounce2.h>
#include <NMEAGPS.h>
#include <NeoSWSerial.h>
#include <Fingerprint.h>
#include <TM1637Display.h>
#include <Adafruit_LEDBackpack.h>
#include <Adafruit_GFX.h>

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
#define VOLT_SENSOR  A3

#define FINGERPRINT_RELAY 13
#define HORN_RELAY 8
#define RIGHT_TURN_RELAY 4
#define LEFT_TURN_RELAY 5
#define HEAD_LIGHT_RELAY 6
#define STARTER_RELAY 7


class DoloresTimer {
#define TURN_SIGNAL_BLINK_DELAY 800

  public:
    static bool isTimeForBlink() {
      return (millis() % (2 * TURN_SIGNAL_BLINK_DELAY)) < TURN_SIGNAL_BLINK_DELAY;
    }
};

class DoloresDatabase {
#define EE_ODOMETER_ADDRESS 0
#define EE_CHECK_ADDRESS 99
#define EE_METRIC_ADDRESS 77
#define INIT_ODOMETER_READING_KM 0.0

  private:
    static void checkEEPROM() {
      int check_value;
      EEPROM.get(EE_CHECK_ADDRESS, check_value);

      if ( check_value == 0xFF) {
        EEPROM.put(EE_ODOMETER_ADDRESS, INIT_ODOMETER_READING_KM);
        EEPROM.put(EE_METRIC_ADDRESS, true);
        EEPROM.put(EE_CHECK_ADDRESS, 0);
      }
    }

  public:
    static void setKph() {
      EEPROM.update(EE_METRIC_ADDRESS, true);
    }

    static void setMph() {
      EEPROM.update(EE_METRIC_ADDRESS, false);
    }

    static boolean isMetric() {
      checkEEPROM();
      boolean isMetric;
      EEPROM.get(EE_METRIC_ADDRESS, isMetric);
      return isMetric;
    }

    static float getOdometer() {
      checkEEPROM();
      float odometer;
      EEPROM.get(EE_ODOMETER_ADDRESS, odometer);

      if (!DoloresDatabase::isMetric()) {
        odometer = odometer / 1.609;
      }

      return odometer;

    }

    static void updateOdometer(float odometer) {
      DoloresDatabase::checkEEPROM();
      EEPROM.update(EE_ODOMETER_ADDRESS, odometer);
    }
};

class DoloresRelay {
  private:
    byte pin;
  public:
    DoloresRelay(byte pin) {
      this->pin = pin;
      pinMode(pin, OUTPUT);
    }

    void turnOn() {
      digitalWrite(pin, HIGH);
    }

    void turnOff() {
      digitalWrite(pin, LOW);
    }

    void toggle() {
      digitalWrite(pin, !getState());
    }

    byte getState() {
      return digitalRead(pin);
    }

};

class DoloresOil {
  private:
    byte pin;
  public:
    DoloresOil(byte pin) {
      this->pin = pin;
      pinMode(pin, INPUT_PULLUP);
    }

    bool isTriggered() {
      return (digitalRead(pin) == LOW);
    }
};

class DoloresVoltage {
#define TIME_SINCE_STARTED_THRESHOLD 1000
#define THRESHOLD .8
#define RESISTOR1 30000.0
#define RESISTOR2 7500.0
  private:
    byte pin;
    int initialValue;
    long millisWhenStarted = 0;
  public:
    DoloresVoltage(byte pin) {
      this->pin = pin;
      this->initialValue = analogRead(pin);
      pinMode(pin, INPUT);
    }

    bool isTriggered() {
      if ( getCurrentVoltage() > THRESHOLD + initialValue) {
        if (millisWhenStarted == 0) {
          millisWhenStarted = millis();
          return false;
        } else if (millisWhenStarted < millis() - TIME_SINCE_STARTED_THRESHOLD) {
          return false;
        } else {
          return true;
        }
      }
      millisWhenStarted = 0;
      return false;
    }

    int getCurrentVoltage() {
      int value = analogRead(pin);
      float vOUT = (value * 5.0) / 1024.0;
      float vIN = vOUT / (RESISTOR2 / (RESISTOR1 + RESISTOR2));
      return vIN;
    }
};


class DoloresButton {
#define DEBOUNCE_DELAY 50
  private:
    byte pin;
    Bounce button = Bounce();
    DoloresRelay* relay;

  public:
    DoloresButton(byte pin, byte relayPin) {
      this->pin = pin;
      this->relay = new DoloresRelay(relayPin);

      pinMode(pin, INPUT_PULLUP);
      button.attach(pin);
      button.interval(DEBOUNCE_DELAY);
      button.update();
    }

    void turnOn() {
      relay->turnOn();
    }

    void turnOff() {
      relay->turnOff();
    }

    boolean isTriggered() {
      bool changed = button.update();
      return (changed && button.read() == LOW);
    }

    boolean isReleased() {
      bool changed = button.update();
      return (changed && button.read() == HIGH);
    }

    virtual void check() {
      if (isTriggered()) {
        turnOn();
      } else if (isReleased()) {
        turnOff();
      }
    }

    boolean isOn() {
      return relay->getState() == HIGH;
    }
};

class DoloresAuxButton : public DoloresButton {
  private:
    DoloresRelay* relayAlt;

    void turnAltOn() {
      relayAlt->turnOn();
    }
    void turnAltOff() {
      relayAlt->turnOff();
    }
  public:
    DoloresAuxButton (byte pin, byte relayPin, byte relayPinAlt) : DoloresButton(pin, relayPin){

      this->relayAlt = new DoloresRelay(relayPinAlt);
    }

  void check(bool useAlt) {
      if (isTriggered()) {
        if (useAlt) {
          turnAltOn();
        } else {
          turnOn();
        }
      } else if (isReleased()) {
        turnOff();
        turnAltOff();
      }
    }
};

class DoloresTurnButton : public DoloresButton {
  public:
    DoloresTurnButton (byte pin, byte relayPin) : DoloresButton(pin, relayPin) {
    }

    void check() {
      if (isTriggered()) {
        if (DoloresTimer::isTimeForBlink()) {
          turnOn();
        } else {
          turnOff();
        }
      } else if (isReleased()) {
        turnOff();
      }
    }
};

class DoloresDisplay {

#define SCROLL_SPEED 400
#define TURN_SIGNAL_BLINK_DELAY 800

    const uint8_t SEG_OIL[4] = {
      SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
      SEG_E | SEG_F,                                   // I
      SEG_E | SEG_F | SEG_D,                           // L
      0x00                                             //
    };

    const uint8_t SEG_TURN_RIGHT[4] = {
      0x00,
      0x00,
      0x00,
      SEG_A | SEG_D | SEG_G
    };

    const uint8_t SEG_TURN_LEFT[4] = {
      SEG_A | SEG_D | SEG_G,
      0x00,
      0x00,
      0x00
    };

    const uint8_t SEG_TURN_BOTH[4] = {
      SEG_A | SEG_D | SEG_G,
      0x00,
      0x00,
      SEG_A | SEG_D | SEG_G
    };

    const uint8_t SEG_READY[4] = {
      SEG_G,
      SEG_G,
      SEG_G,
      SEG_G
    };


  private:
    byte pin_CLK_SDA;
    byte pin_DIO_SCL;
    boolean isAlphanumeric;
    Adafruit_AlphaNum4 alpha4;
    TM1637Display* numeric4;

    void displayOdometerAlphanumeric(float input) {
      String result = String(input, DEC);
      int loc = result.indexOf('.');
      result = result.substring(0, loc);
      result = "ODO-" + result + (DoloresDatabase::isMetric() ? "K" : "M") ;
      scrollAlphaNumericDisaplay(result);
    }

    void scrollAlphaNumericDisaplay(String input) {
      input = input + "    ";
      char buf[input.length()];
      input.toCharArray(buf, input.length());
      char displaybuffer[4] = {' ', ' ', ' ', ' '};
      for (int i = 0; i < input.length() - 1 ; i++ ) {
        char c = buf[i];
        displaybuffer[0] = displaybuffer[1];
        displaybuffer[1] = displaybuffer[2];
        displaybuffer[2] = displaybuffer[3];
        displaybuffer[3] = c;

        // set every digit to the buffer
        alpha4.writeDigitAscii(0, displaybuffer[0]);
        alpha4.writeDigitAscii(1, displaybuffer[1]);
        alpha4.writeDigitAscii(2, displaybuffer[2]);
        alpha4.writeDigitAscii(3, displaybuffer[3]);

        alpha4.writeDisplay();
        delay(SCROLL_SPEED);
      }
      alpha4.clear();
      alpha4.writeDisplay();
      delay(SCROLL_SPEED);
    }

    void digitalDisplay(boolean isOil, boolean isRunning, boolean isLeftTurn, boolean isRightTurn, int displaySpeed) {
      if (isOil) {
        numeric4->setSegments(SEG_OIL);
      } else {
        if (isRunning) {
          if (isLeftTurn || isRightTurn) {
            if (DoloresTimer::isTimeForBlink()) {
              if (isLeftTurn && isRightTurn) {
                numeric4->setSegments(SEG_TURN_BOTH);
              } else if (isLeftTurn) {
                numeric4->setSegments(SEG_TURN_LEFT);
              } else if (isRightTurn) {
                numeric4->setSegments(SEG_TURN_RIGHT);
              }
            } else {
              numeric4->clear();
            }
          } else {
            uint8_t kph[] = { 0x00, 0x00, 0x00, (DoloresDatabase::isMetric() ? SEG_A : SEG_D) };
            numeric4->setSegments(kph);
            numeric4->showNumberDec(displaySpeed, true, 3, 0);
          }
        } else {
          numeric4->setSegments(SEG_READY);
        }
      }
    }

    void alphaNumericDisplay(boolean isOil, boolean isRunning, boolean isLeftTurn, boolean isRightTurn, int displaySpeed) {
      if (isOil) {
        alpha4.writeDigitAscii(0, 'O');
        alpha4.writeDigitAscii(1, 'I');
        alpha4.writeDigitAscii(2, 'L');
        alpha4.writeDigitAscii(3, ' ');
      } else {
        if (isRunning) {
          if (isLeftTurn || isRightTurn) {
            if (DoloresTimer::isTimeForBlink()) {
              if (isLeftTurn && isRightTurn) {
                alpha4.writeDigitAscii(0, '<');
                alpha4.writeDigitAscii(1, ' ');
                alpha4.writeDigitAscii(2, ' ');
                alpha4.writeDigitAscii(3, '>');
              } else if (isLeftTurn) {
                alpha4.writeDigitAscii(0, '<');
                alpha4.writeDigitAscii(1, ' ');
                alpha4.writeDigitAscii(2, ' ');
                alpha4.writeDigitAscii(3, ' ');
              } else if (isRightTurn) {
                alpha4.writeDigitAscii(0, ' ');
                alpha4.writeDigitAscii(1, ' ');
                alpha4.writeDigitAscii(2, ' ');
                alpha4.writeDigitAscii(3, '>');
              }
            } else {
              alpha4.writeDigitAscii(0, ' ');
              alpha4.writeDigitAscii(1, ' ');
              alpha4.writeDigitAscii(2, ' ');
              alpha4.writeDigitAscii(3, ' ');

            }
          } else {
            char buf [4];
            sprintf (buf, "%03i", displaySpeed);
            alpha4.writeDigitAscii(0, buf[0]);
            alpha4.writeDigitAscii(1, buf[1]);
            alpha4.writeDigitAscii(2, buf[2]);
            alpha4.writeDigitAscii(3, DoloresDatabase::isMetric() ? 'K' : 'M');
          }
        } else {
          alpha4.writeDigitAscii(0, '<');
          alpha4.writeDigitAscii(1, '{');
          alpha4.writeDigitAscii(2, '}');
          alpha4.writeDigitAscii(3, '>');
        }
      }
      alpha4.writeDisplay();
    }

    void displayOdometerDigital (float input) {
      int odometer = (int) input;

      int digits = ((int) pow(odometer , 0.1)) + 1;
      for (int i = 0; i < digits ; i++) {
        int display_value = getDisplayValue(digits, i, odometer);
        numeric4->showNumberDec(display_value, i > 3, 3, 0);
        delay(SCROLL_SPEED);
      }
    }

    int getDisplayValue(int digits, int offset , int reading) {
      int x0 = extractDigit(reading, digits - offset);
      int x1 = extractDigit(reading, digits - offset + 1);
      int x2 = extractDigit(reading, digits - offset + 2);

      return  (x2 * 100) + (x1 * 10) + x0;
    }

    int extractDigit(int v, int p) {
      return int(v / (pow(10, p - 1))) - int(v / (pow(10, p))) * 10;
    }

  public:
    DoloresDisplay(byte pin_CLK_SDA, byte pin_DIO_SCL, boolean isAlphanumeric = true) {
      this->pin_CLK_SDA = pin_CLK_SDA;
      this->pin_DIO_SCL = pin_DIO_SCL;
      this->isAlphanumeric = isAlphanumeric;
      this->alpha4 = Adafruit_AlphaNum4();
      this->numeric4 = new TM1637Display(CLK_DISPLAY, DIO_DISPLAY);
      alpha4.begin(0x70);
    }

    void scrollText(String value) {
      if (isAlphanumeric) {
        scrollAlphaNumericDisaplay(value);
      } else {
        delay(2000);
      }
    }

    void scrollOdometer(float odometer) {
      if (isAlphanumeric) {
        displayOdometerAlphanumeric(odometer);
      } else {
        displayOdometerDigital(odometer);
      }
    }

    void setDisplay(boolean isOil, boolean isRunning, boolean isLeftTurn, boolean isRightTurn, int displaySpeed) {
      if (isAlphanumeric) {
        alphaNumericDisplay(isOil, isRunning, isLeftTurn, isRightTurn, displaySpeed);
      } else {
        digitalDisplay(isOil, isRunning, isLeftTurn, isRightTurn, displaySpeed);
      }
    }
};

class DoloresFingerprint {
  private:
    byte pin_SLA;
    byte pin_SLK;
    DoloresRelay* relay;
    Fingerprint* finger;
    boolean loggedIn;

    int getFingerprintIDez() {
      uint8_t p = finger->getImage();
      if (p != FINGERPRINT_OK)  return -1;

      p = finger->image2Tz();
      if (p != FINGERPRINT_OK)  return -1;

      p = finger->fingerFastSearch();
      if (p != FINGERPRINT_OK)  return -1;

      return finger->fingerID;
    }
  public:
    DoloresFingerprint(byte pin_SLA, byte pin_SLK, byte relay) {
      this->pin_SLA = pin_SLA;
      this->pin_SLK = pin_SLK;
      this->relay = new DoloresRelay(relay);
      this->finger = new Fingerprint(&Serial);
      this->loggedIn = false;

      finger->begin(57600);
    }

    // return true if just logged in
    boolean logIn() {
      while (!loggedIn) {
        int id = getFingerprintIDez();
        if (id > 0 && id < 11) {
          relay->turnOn();
          loggedIn = true;
          return true;
        }
      }
      return false;
    }

};

class DoloresGPS {
#define MIN_SPEED 5
#define RUNNING_DISTANCE_THRESHOLD 1.00
  private:
    byte pin_TX;
    byte pin_RX;
    NMEAGPS gps;
    NeoGPS::Location_t prev_location;
    NeoSWSerial* gpsSerial;
    bool firstLocationScan = true;
    float runningDistance = 0;

    void persistRange(float range) {
      DoloresDatabase::updateOdometer(DoloresDatabase::getOdometer() + range);
    }
  public:
    DoloresGPS(byte pin_TX, byte pin_RX) {
      this->pin_TX = pin_TX;
      this->pin_RX = pin_RX;

      gpsSerial = new NeoSWSerial(pin_TX, pin_RX);

      // initialize GPS
      gpsSerial->begin(9600);
      gps.send_P( gpsSerial, F("PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")); // RMC(Recomended minim specific) only
      gps.send_P( gpsSerial, F("PMTK220,1000") ); // 4Hz update
    }

    int getGps() {
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
              persistRange(range);
              runningDistance = 0;
            }
          }
        }
      }
      return displaySpeed;
    }
};


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
  buttonLeft = new DoloresTurnButton(LEFT_TURN_IN, LEFT_TURN_RELAY);
  buttonRight = new DoloresTurnButton(RIGHT_TURN_IN, RIGHT_TURN_RELAY);
  buttonHeadLight = new DoloresButton(HEAD_LIGHT_IN, HEAD_LIGHT_RELAY);
  buttonAux = new DoloresAuxButton(AUX_IN,STARTER_RELAY, HORN_RELAY);

  gps = new DoloresGPS(GPS_TX, GPS_RX);
  fingerprint = new DoloresFingerprint(FINGERPRINT_SLA, FINGERPRINT_SLK, FINGERPRINT_RELAY);
  screen = new DoloresDisplay(CLK_DISPLAY, DIO_DISPLAY);
  oil = new DoloresOil(OIL_SENSOR);
  volt = new DoloresVoltage(VOLT_SENSOR);


  //ELECTRONICS TEST
  digitalWrite(RIGHT_TURN_RELAY, HIGH);
  digitalWrite(LEFT_TURN_RELAY, HIGH);

  //DISPLAY ODOMETER
  screen->scrollOdometer(DoloresDatabase::getOdometer());

  digitalWrite(RIGHT_TURN_RELAY, LOW);
  digitalWrite(LEFT_TURN_RELAY, LOW);
}

void loop() {

  boolean isLoggedIn = fingerprint->logIn();
  if (isLoggedIn) {
    screen->scrollText("WELCOME VADIM");
  }

  int displaySpeed = gps->getGps();
  
  //check oil sensor
  boolean isOil =  oil->isTriggered();

  // check if running
  boolean isRunning = volt->isTriggered();


  buttonLeft->check();
  buttonRight->check();
  buttonHeadLight->check();
  buttonAux->check(isRunning);

  screen->setDisplay(isOil, isRunning, buttonLeft->isOn(), buttonRight->isOn(), displaySpeed);
}




