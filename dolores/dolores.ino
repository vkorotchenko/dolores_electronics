#include <Arduino.h>
#include <TM1637Display.h>
#include <NMEAGPS.h>
#include <NeoSWSerial.h>
#include <Bounce2.h>
#include <EEPROM.h>
#include <Fingerprint.h>


// Module connection pins (Digital Pins)
#define CLK 2
#define DIO 3
#define GPS_TX 8
#define GPS_RX 7
#define RIGHT_TURN_IN 4
#define RIGHT_TURN_RELAY 13
#define LEFT_TURN_IN 6
#define LEFT_TURN_RELAY 9
#define HEAD_LIGHT_IN 10
#define HEAD_LIGHT_RELAY A2
#define AUX_IN 12
#define STARTER_RELAY 11
#define HORN_RELAY 5

#define OIL_SENSOR A0
#define VOLT_SENSOR A1
#define FINGERPRINT_SLA 0
#define FINGERPRINT_SLK 1
#define FINGERPRINT_RELAY A2


//SEGMENTS
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

// CONSTANTS
#define MIN_SPEED 5
#define DEBOUNCE_DELAY 50
#define TURN_SIGNAL_BLINK_DELAY 800
#define VOLTAGE_THRESHOLD 13.5
#define RUNNING_DISTANCE_THRESHOLD 1.00f
#define EE_ODOMETER_ADDRESS 0
#define EE_CHECK_ADDRESS 99
#define EE_METRIC_ADDRESS 77
#define TIME_SINCE_STARTED_THRESHOLD 1000
#define SCROLL_SPEED 800

#define INIT_ODOMETER_READING_KM 0


// VOLTAGE SENSOR VALUES
float R1 = 30000.0;
float R2 = 7500.0;

//fingerprint

Fingerprint finger = Fingerprint(&Serial);
bool loggedIn = false;

// VARIABLES
bool isMetric = true;
bool enableBothTurnSignals = true;
bool voltSensorBypass = true;

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

// declare GPS
NeoSWSerial gpsSerial(GPS_TX, GPS_RX);
NMEAGPS gps;

NeoGPS::Location_t prev_location;
bool firstLocationScan = true;
float runningDistance = 0;


// declate LED Display
TM1637Display display(CLK, DIO);

void setup()
{
  // Display settings
  display.setBrightness(0x09);

  //INIT EEPROM
  initEEPROM();

  if (isMetric) {
    setKph();
  } else {
    setMph();
  }

  //fingerprint
  pinMode(FINGERPRINT_RELAY, OUTPUT);
  finger.begin(57600);


  // initialize GPS
  gpsSerial.begin(9600);
  gps.send_P( &gpsSerial, F("PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")); // RMC(Recomended minim specific) only
  gps.send_P( &gpsSerial, F("PMTK220,1000") ); // 4Hz update

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
  scrollOdometer();

  digitalWrite(RIGHT_TURN_RELAY, LOW);
  digitalWrite(LEFT_TURN_RELAY, LOW);
}

void loop()
{
  while (!loggedIn) {
    int id = getFingerprintIDez();
    if (id > 0 && id < 11) {
      digitalWrite(FINGERPRINT_RELAY, HIGH);
      loggedIn = true;

      // TODO SCROLL "WELCOME VADIM"
    }
  }

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

void persistRange(float range) {
  float odometer = 0.00f;
  EEPROM.get( EE_ODOMETER_ADDRESS, odometer);
  odometer = odometer + range;
  EEPROM.update( EE_ODOMETER_ADDRESS, odometer);
}

void scrollOdometer() {
  float value;
  EEPROM.get(EE_ODOMETER_ADDRESS, value);

  if (!isMetric) {
    value = value / 1.609;
  }

  int odometer = (int) value;

  int digits = ((int) pow(odometer , 0.1)) + 1;

  for (int i = 0; i < digits ; i++) {
    int display_value = getDisplayValue(digits, i, odometer);
    display.showNumberDec(display_value, i > 3, 3, 0);
    delay(SCROLL_SPEED);
  }
  delay(SCROLL_SPEED);
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

boolean isMotorcycleRunning() {
  if (voltSensorBypass) {
    return digitalRead(VOLT_SENSOR) == HIGH;
  } else {
    int value = analogRead(VOLT_SENSOR);
    float vOUT = (value * 5.0) / 1024.0;
    float vIN = vOUT / (R2 / (R1 + R2));

    if ( vIN > VOLTAGE_THRESHOLD) {
      if (millisWhenStarted = 0) {
        millisWhenStarted = millis();
        return false;
      } else if (millisWhenStarted < millis() - TIME_SINCE_STARTED_THRESHOLD) {
        return false;
      } else {
        return true;
      }
    } else {
      return false;
    }
  }
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

void setDisplay() {
  if (isOil) {
    display.setSegments(SEG_OIL);
    return;
  }

  if (isRunning) {
    if ((isLeftTurnOn || isRightTurnOn) && isTimeForBlink()) {
      if (isLeftTurnOn && isRightTurnOn) {
        display.setSegments(SEG_TURN_BOTH);
      } else if (isLeftTurnOn) {
        display.setSegments(SEG_TURN_LEFT);
      } else if (isRightTurnOn) {
        display.setSegments(SEG_TURN_RIGHT);
      } else {
        setSpeed(displaySpeed);
      }
    } else {
      setSpeed(displaySpeed);
    }
  } else {
    display.setSegments(SEG_READY);
  }
}


void setSpeed(int speed) {
  display.showNumberDec(speed, true, 3, 0);
}

void setKph() {
  uint8_t kph[] = { 0x00, 0x00, 0x00, SEG_A };
  display.setSegments(kph);
  isMetric = true;
  EEPROM.update(EE_METRIC_ADDRESS, isMetric);
}

void setMph() {
  uint8_t kph[] = { 0x00, 0x00, 0x00, SEG_D };
  display.setSegments(kph);
  isMetric = false;
  EEPROM.update(EE_METRIC_ADDRESS, isMetric);
}

void initEEPROM() {
  int check_value;
  EEPROM.get(EE_CHECK_ADDRESS, check_value);

  if ( check_value == 255) {
    EEPROM.put(EE_ODOMETER_ADDRESS, INIT_ODOMETER_READING_KM);
    EEPROM.put(EE_METRIC_ADDRESS, isMetric);
    setKph();
    EEPROM.put(EE_CHECK_ADDRESS, 0);
  } else {
    EEPROM.get(EE_METRIC_ADDRESS, isMetric);
  }
}

int getFingerprintIDez() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK)  return -1;

  return finger.fingerID;
}
