#include <Arduino.h>
#include <DoloresDisplay.h>

#define LED_PIN 13
#define CLK_DISPLAY A4
#define DIO_DISPLAY A5
#define ALPHANUMERIC true

  
DoloresDisplay* screen;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  screen = new DoloresDisplay(CLK_DISPLAY, DIO_DISPLAY, ALPHANUMERIC);
  Serial.begin(9600);
}

void loop() {
  // display oil
  Serial.println("DISPLAY OIL");
  screen->setDisplay(true, false, false, false, 123);
  delay(2000);

  // display ready
  Serial.println("DISPLAY READY");
  screen->setDisplay(false, false, true, true, 123);
  delay(2000);

  // display left
  Serial.println("DISPLAY LEFT");
  screen->setDisplay(false, true, true, false, 123);
  delay(2000);

  // display right
  Serial.println("DISPLAY RIGHT");
  screen->setDisplay(false, true, false, true, 123);
  delay(2000);

  // display both
  Serial.println("DISPLAY BOTH");
  screen->setDisplay(false, true, true, true, 123);
  delay(2000);

  // display speed
  Serial.println("DISPLAY SPEED");
  screen->setDisplay(false, true, false, false, 123);
  delay(2000);

  // scroll ODOMETER
  Serial.println("SCROLL ODO");
  screen->scrollOdometer(123456.7);
  delay(2000);

  // scroll ODOMETER
  Serial.println("SCROLL TEXT");
  screen->scrollText("WELCOME");
  delay(2000);


}