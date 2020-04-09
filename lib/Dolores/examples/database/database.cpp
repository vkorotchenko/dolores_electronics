#include <Arduino.h>
#include <DoloresDatabase.h>

void setup() {
  Serial.begin(9600);
}

void loop() {
  String odo = String(DoloresDatabase::getOdometer(), DEC);
  Serial.println("CURRENT ODOMETER READING(0?)" + odo);
  Serial.println("CURRENT IsMETRIC(t)" + (String)(DoloresDatabase::isMetric() ? "TRUE": "FALSE"));
  
  DoloresDatabase::setMph();
  Serial.println("CURRENT IsMETRIC(f)" + (String)(DoloresDatabase::isMetric() ? "TRUE": "FALSE"));
  
  DoloresDatabase::setKph();
  Serial.println("CURRENT IsMETRIC(t)" + (String)(DoloresDatabase::isMetric() ? "TRUE": "FALSE"));
  
  DoloresDatabase::updateOdometer(123.0);
  String odo_updated = String(DoloresDatabase::getOdometer(), DEC);
  Serial.println("CURRENT ODOMETER READING(123)" + odo_updated);
  DoloresDatabase::updateOdometer(123123.0);
   odo_updated = String(DoloresDatabase::getOdometer(), DEC);
  Serial.println("CURRENT ODOMETER READING(123123)" + odo_updated);
  DoloresDatabase::updateOdometer(44444444.0);
   odo_updated = String(DoloresDatabase::getOdometer(), DEC);
  Serial.println("CURRENT ODOMETER READING(44444444)" + odo_updated);
  DoloresDatabase::updateOdometer(9999999999.0);
   odo_updated = String(DoloresDatabase::getOdometer(), DEC);
  Serial.println("CURRENT ODOMETER READING(9999999999)" + odo_updated); // LOSS OF PERCISION

  DoloresDatabase::updateOdometer(0.0);
  String odo_updated2 = String(DoloresDatabase::getOdometer(), DEC);
  Serial.println("CURRENT ODOMETER READING(0)" + odo_updated2);

  while(1) {
    // we do not want to update database too many times as it becomes volatile after 100,000 updates(per address)
  }
}