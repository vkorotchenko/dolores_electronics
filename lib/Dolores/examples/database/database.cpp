#include <Arduino.h>
#include <DoloresDatabase.h>


void setup() {
  Serial.begin(9600);
}

void loop() {
  String odo = String(DoloresDatabase::getOdometer(), DEC);
  Serial.println("CURRENT ODOMETER READING" + odo);
  Serial.println("CURRENT IsMETRIC" + DoloresDatabase::isMetric());
  
  DoloresDatabase::setMph();
  Serial.println("CURRENT IsMETRIC" + DoloresDatabase::isMetric());
  
  DoloresDatabase::setKph();
  Serial.println("CURRENT IsMETRIC" + DoloresDatabase::isMetric());
  
  DoloresDatabase::updateOdometer(123.0);
  String odo_updated = String(DoloresDatabase::getOdometer(), DEC);
  Serial.println("CURRENT ODOMETER READING" + odo_updated);


  DoloresDatabase::updateOdometer(0.0);
  String odo_updated2 = String(DoloresDatabase::getOdometer(), DEC);
  Serial.println("CURRENT ODOMETER READING" + odo_updated2);

  while(1) {
    // we do not want to update database too many times as it becomes volatile after 100,000 updates(per address)
  }
}