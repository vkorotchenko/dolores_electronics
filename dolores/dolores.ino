#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_GFX.h>

// Module connection pins (Digital Pins)
#define FINGERPRINT_SLA 0 // /
#define FINGERPRINT_SLK 1 // /
#define CLK_DISPLAY A4 // SDA
#define DIO_DISPLAY A5 // SCL
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



class Voltage {
#define TIME_SINCE_STARTED_THRESHOLD 1000
#define THRESHOLD .8
#define RESISTOR1 30000.0
#define RESISTOR2 7500.0
  private:
    byte pin;
    int initialValue;
    long millisWhenStarted = 0;
  public:
    Voltage(byte pin) {
      this->pin = pin;
      this->initialValue = analogRead(pin);
    }

    void init() {
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
      return false;
    }

    int getCurrentVoltage() {
      int value = analogRead(pin);
      float vOUT = (value * 5.0) / 1024.0;
      float vIN = vOUT / (RESISTOR2 / (RESISTOR1 + RESISTOR2));
      return vIN;
    }
};

class Button {
#include <Bounce2.h>
#define DEBOUNCE_DELAY 50
  private:
    byte pin;
    Bounce button = Bounce();
    byte state;
  public:
    Button(byte pin) {
      this->pin = pin;
      pinMode(pin, INPUT_PULLUP);
    }

    void init() {
      button.attach(pin);
      button.interval(DEBOUNCE_DELAY);
      button.update();
      state = button.read();
    }

    boolean isTriggered() {
      bool changed = button.update();
      return (changed && button.read() == LOW);
    }

    boolean isReleased() {
      bool changed = button.update();
      return (changed && button.read() == HIGH);
    }

    byte getCurrentState() {

    }
};

class Relay {
  private:
    byte pin;
  public:
    Relay(byte pin) {
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
      digitalWrite(pin, !digitalRead(pin));
    }

};

class Display {
#include <TM1637Display.h>
#include <Adafruit_LEDBackpack.h>
#define SCROLL_SPEED 400

    const uint8_t SEG_OIL[] = {
      SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
      SEG_E | SEG_F,                                   // I
      SEG_E | SEG_F | SEG_D,                           // L
      0x00                                             //
    };

    const uint8_t SEG_TURN_RIGHT[] = {
      0x00,
      0x00,
      0x00,
      SEG_A | SEG_D | SEG_G
    };

    const uint8_t SEG_TURN_LEFT[] = {
      SEG_A | SEG_D | SEG_G,
      0x00,
      0x00,
      0x00
    };

    const uint8_t SEG_TURN_BOTH[] = {
      SEG_A | SEG_D | SEG_G,
      0x00,
      0x00,
      SEG_A | SEG_D | SEG_G
    };

    const uint8_t SEG_READY[] = {
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

    void displayOdometerAlphanumeric(float input) {
      String result = String(input, DEC);
      int loc = result.indexOf('.');
      result = result.substring(0, loc);
      result = "ODO-" + result + (isMetric ? "K" : "M") ;
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

    void digitalDisplay() {
      if (isOil) {
        display.setSegments(SEG_OIL);
      } else {
        if (isRunning) {
          if (isLeftTurnOn || isRightTurnOn) {
            if (isTimeForBlink()) {
              if (isLeftTurnOn && isRightTurnOn) {
                display.setSegments(SEG_TURN_BOTH);
              } else if (isLeftTurnOn) {
                display.setSegments(SEG_TURN_LEFT);
              } else if (isRightTurnOn) {
                display.setSegments(SEG_TURN_RIGHT);
              }
            } else {
              display.clear();
            }
          } else {
            uint8_t kph[] = { 0x00, 0x00, 0x00, (isMetric ? SEG_A : SEG_D) };
            display.setSegments(kph);
            display.showNumberDec(displaySpeed, true, 3, 0);
          }
        } else {
          display.setSegments(SEG_READY);
        }
      }

    }

    void alphaNumericDisplay() {
      if (isOil) {
        alpha4.writeDigitAscii(0, 'O');
        alpha4.writeDigitAscii(1, 'I');
        alpha4.writeDigitAscii(2, 'L');
        alpha4.writeDigitAscii(3, ' ');
      } else {
        if (isRunning) {
          if (isLeftTurnOn || isRightTurnOn) {
            if (isTimeForBlink()) {
              if (isLeftTurnOn && isRightTurnOn) {
                alpha4.writeDigitAscii(0, '<');
                alpha4.writeDigitAscii(1, ' ');
                alpha4.writeDigitAscii(2, ' ');
                alpha4.writeDigitAscii(3, '>');
              } else if (isLeftTurnOn) {
                alpha4.writeDigitAscii(0, '<');
                alpha4.writeDigitAscii(1, ' ');
                alpha4.writeDigitAscii(2, ' ');
                alpha4.writeDigitAscii(3, ' ');
              } else if (isRightTurnOn) {
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
            alpha4.writeDigitAscii(3, isMetric ? 'K' : 'M');
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
      int odometer;
      if (!isMetric) {
        odometer = input / 1.609;
      }

      int digits = ((int) pow(odometer , 0.1)) + 1;
      for (int i = 0; i < digits ; i++) {
        int display_value = getDisplayValue(digits, i, odometer);
        display.showNumberDec(display_value, i > 3, 3, 0);
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
    Relay(byte pin_CLK_SDA, byte pin_DIO_SCL, boolean isAlphanumeric = true) {
      this->pin_CLK_SDA = pin_CLK_SDA;
      this->pin_DIO_SCL = pin_DIO_SCL;
      this->isAlphanumeric = isAlphanumeric;
      this->alpha4 = Adafruit_AlphaNum4();
      TM1637Display display(CLK_DISPLAY, DIO_DISPLAY);
    }

    init() {
      alpha4.begin(0x70);  // pass in the address
    }

    void scrollText(String value) {
      if (alphaNumeric) {
        scrollAlphaNumericDisaplay(value);
      }
    }

    void scrollOdometer(float odometer) { // TODO move EEPROM and conversion out and pass into function
      //      float value;
      //      EEPROM.get(EE_ODOMETER_ADDRESS, value);
      //
      //      if (!isMetric) {
      //        value = value / 1.609;
      //      }


      if (alphaNumeric) {
        displayOdometerAlphanumeric(odometer);
      } else {
        displayOdometerDigital(odometer);
      }
    }

    void setDisplay() {
      if (alphaNumeric) {
        alphaNumericDisplay();
      } else {
        digitalDisplay();
      }
    }


};

class Fingerprint {
#include <Fingerprint.h>
  private:
    byte pin_SLA;
    byte pin_SLK;
    Relay relay;
    Fingerprint finger;
    boolean loggedIn;

    int getFingerprintIDez() {
      uint8_t p = finger.getImage();
      if (p != FINGERPRINT_OK)  return -1;

      p = finger.image2Tz();
      if (p != FINGERPRINT_OK)  return -1;

      p = finger.fingerFastSearch();
      if (p != FINGERPRINT_OK)  return -1;

      return finger.fingerID;
    }
  public:
    Fingerprint(byte pin_SLA, byte pin_SLK, byte relay) {
      this->pin_SLA = pin_SLA;
      this->pin_SLK = pin_SLK;
      this->relay = Relay(relay);
      this->finger = Fingerprint(&Serial);
      this->loggedIn = false;
    }

    void init() {
      finger.begin(57600);
    }

    // return true if just logged in
    boolean logIn() {
      while (!loggedIn) {
        int id = getFingerprintIDez();
        if (id > 0 && id < 11) {
          digitalWrite(FINGERPRINT_RELAY, HIGH);
          loggedIn = true;
          return true;
          //          if (alphaNumeric) {
          //            scrollAlphaNumericDisaplay("WELCOME VADIM"); // TODO move out -> run only once.
          //          }
        }
      }
      return false;
    }

};

class GPS {
#include <NMEAGPS.h>
#include <NeoSWSerial.h>
#define MIN_SPEED 5
#define RUNNING_DISTANCE_THRESHOLD 1.00f
  private:
    byte pin_TX;
    byte pin_RX;
    NMEAGPS gps;

    NeoGPS::Location_t prev_location;
    bool firstLocationScan = true;
    float runningDistance = 0;

    
void persistRange(float range) {
  float odometer = 0.00f;
  EEPROM.get( EE_ODOMETER_ADDRESS, odometer);
  odometer = odometer + range;
  EEPROM.update( EE_ODOMETER_ADDRESS, odometer);
}
  public:
    GPS(byte pin_TX, byte pin_RX) {
      this->pin_TX = pin_TX;
      this->pin_RX = pin_RX;
    }

    void init() {
      NeoSWSerial gpsSerial(pin_TX, pin_RX);

      // initialize GPS
      gpsSerial.begin(9600);
      gps.send_P( &gpsSerial, F("PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")); // RMC(Recomended minim specific) only
      gps.send_P( &gpsSerial, F("PMTK220,1000") ); // 4Hz update
    }

    void getGps() {
      while (gps.available( gpsSerial )) {
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
    }

};

//SEGMENTS

// CONSTANTS
#define DEBOUNCE_DELAY 50
#define TURN_SIGNAL_BLINK_DELAY 800
#define VOLTAGE_THRESHOLD 13.5
#define EE_ODOMETER_ADDRESS 0
#define EE_CHECK_ADDRESS 99
#define EE_METRIC_ADDRESS 77

#define INIT_ODOMETER_READING_KM 0



// CONFIG
bool isMetric = true;
bool enableBothTurnSignals = true;
bool alphaNumeric = true;

bool isLeftTurnOn = false;
bool isRightTurnOn = false;
bool isHeadlightOn = false;

long millisWhenStarted = 0;

// display variables
int displaySpeed = 0;
bool isOil = false;
bool isRunning = false;

//decleare Bounce
Bounce debouncerRightTurn = Bounce();
Bounce debouncerLeftTurn = Bounce();
Bounce debouncerHeadlight = Bounce();
Bounce debouncerAux = Bounce();



// declate LED Display
Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();
TM1637Display display(CLK_DISPLAY, DIO_DISPLAY);

void setup() {
  // Display settings

  //INIT EEPROM
  initEEPROM();

  if (isMetric) {
    setKph();
  } else {
    setMph();
  }



  // set pin modes
  // INPUT
  pinMode(RIGHT_TURN_IN, INPUT_PULLUP);
  pinMode(LEFT_TURN_IN, INPUT_PULLUP);
  pinMode(HEAD_LIGHT_IN, INPUT_PULLUP);
  pinMode(AUX_IN, INPUT_PULLUP);
  pinMode(OIL_SENSOR, INPUT_PULLUP);
  pinMode(VOLT_SENSOR, INPUT);

  //init bounce
  debouncerRightTurn.attach(RIGHT_TURN_IN);
  debouncerRightTurn.interval(DEBOUNCE_DELAY);
  debouncerLeftTurn.attach(LEFT_TURN_IN);
  debouncerLeftTurn.interval(DEBOUNCE_DELAY);
  debouncerHeadlight.attach(HEAD_LIGHT_IN);
  debouncerHeadlight.interval(DEBOUNCE_DELAY);
  debouncerAux.attach(AUX_IN);
  debouncerAux.interval(DEBOUNCE_DELAY);


  //OUTPUT
  pinMode(RIGHT_TURN_RELAY, OUTPUT);
  pinMode(LEFT_TURN_RELAY, OUTPUT);
  pinMode(HEAD_LIGHT_RELAY, OUTPUT);
  pinMode(STARTER_RELAY, OUTPUT);
  pinMode(HORN_RELAY, OUTPUT);

  //ELECTRONICS TEST
  digitalWrite(RIGHT_TURN_RELAY, HIGH);
  digitalWrite(LEFT_TURN_RELAY, HIGH);

  //DISPLAY ODOMETER
  display.scrollOdometer(odometer);

  digitalWrite(RIGHT_TURN_RELAY, LOW);
  digitalWrite(LEFT_TURN_RELAY, LOW);
}

void loop() {

  boolean isLoggedIn = fingerprint.logIn();
  if (isLoggedIn) {
    display.scrollText("WELCOME VADIM");
  }

  gps.getGps();

  bool rightChanged = debouncerRightTurn.update();
  bool leftChanged = debouncerLeftTurn.update();
  bool headChanged = debouncerHeadlight.update();
  bool auxChanged = debouncerAux.update();

  //read left turn
  if (leftChanged) {
    if ((debouncerLeftTurn.read() == LOW)) {
      isLeftTurnOn = true;
      if (isLeftTurnOn && !enableBothTurnSignals) {
        isRightTurnOn = false;
      }
    } else {
      isLeftTurnOn = false;
    }
  }

  //read right turn
  if (rightChanged) {
    if (debouncerRightTurn.read() == LOW) {
      isRightTurnOn = true;
      if (isRightTurnOn && !enableBothTurnSignals) {
        isLeftTurnOn = false;
      }
    } else {
      isRightTurnOn = false;
    }
  }

  //check oil sensor
  isOil =  (digitalRead(OIL_SENSOR) == LOW);

  // check if running
  isRunning = isMotorcycleRunning();


  // set outputs
  if (isRunning) {
    //AUX Button
    digitalWrite(HORN_RELAY, !debouncerAux.read());
    digitalWrite(STARTER_RELAY, LOW);
  } else {
    //AUX Button
    digitalWrite(STARTER_RELAY, !debouncerAux.read());
    digitalWrite(HORN_RELAY, LOW);
  }

  blinkTurnSignal(isLeftTurnOn, LEFT_TURN_RELAY);
  blinkTurnSignal(isRightTurnOn, RIGHT_TURN_RELAY);
  digitalWrite(HEAD_LIGHT_RELAY, !debouncerHeadlight.read());
  setDisplay();
}


boolean isMotorcycleRunning() {


}

void blinkTurnSignal(bool isOn, int pin) {
  if (isOn) {
    if (isTimeForBlink()) {
      digitalWrite(pin, HIGH);
    } else {
      digitalWrite(pin, LOW);
    }
  } else {
    digitalWrite(pin, LOW);
  }
}

bool isTimeForBlink() {
  return (millis() % (2 * TURN_SIGNAL_BLINK_DELAY)) < TURN_SIGNAL_BLINK_DELAY;
}


void setKph() {
  isMetric = true;
  EEPROM.update(EE_METRIC_ADDRESS, isMetric);
}

void setMph() {
  isMetric = false;
  EEPROM.update(EE_METRIC_ADDRESS, isMetric);
}

void initEEPROM() {
  int check_value;
  EEPROM.get(EE_CHECK_ADDRESS, check_value);

  if ( check_value == 0xFF) {
    EEPROM.put(EE_ODOMETER_ADDRESS, INIT_ODOMETER_READING_KM);
    EEPROM.put(EE_METRIC_ADDRESS, isMetric);
    setKph();
    EEPROM.put(EE_CHECK_ADDRESS, 0);
  } else {
    EEPROM.get(EE_METRIC_ADDRESS, isMetric);
    int odo_val;
    EEPROM.get(EE_ODOMETER_ADDRESS, odo_val);
    int m_val;
    EEPROM.get(EE_METRIC_ADDRESS, m_val);
  }
}

