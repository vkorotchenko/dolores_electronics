#include "DoloresDatabase.h"
#include "EEPROM.h"

#define EE_ODOMETER_ADDRESS 0
#define EE_CHECK_ADDRESS 99
#define EE_METRIC_ADDRESS 77
#define INIT_ODOMETER_READING_KM 0.0


void DoloresDatabase::checkEEPROM() {
    int check_value;
    EEPROM.get(EE_CHECK_ADDRESS, check_value);

    if ( check_value == 0xFF || check_value == 0) {
    EEPROM.put(EE_ODOMETER_ADDRESS, INIT_ODOMETER_READING_KM);
    EEPROM.put(EE_METRIC_ADDRESS, true);
    EEPROM.put(EE_CHECK_ADDRESS, 1);
    }
}

void DoloresDatabase::setKph() {
    EEPROM.update(EE_METRIC_ADDRESS, true);
}

void DoloresDatabase::setMph() {
    EEPROM.update(EE_METRIC_ADDRESS, false);
}

boolean DoloresDatabase::isMetric() {
    DoloresDatabase::checkEEPROM();
    boolean isMetric;
    EEPROM.get(EE_METRIC_ADDRESS, isMetric);
    return isMetric;
}

float DoloresDatabase::getOdometer() {
    DoloresDatabase::checkEEPROM();
    float odometer;
    EEPROM.get(EE_ODOMETER_ADDRESS, odometer);

    if (!isMetric()) {
    odometer = odometer / 1.609;
    }

    return odometer;

}

void DoloresDatabase::updateOdometer(float odometer) {
    DoloresDatabase::checkEEPROM();
    EEPROM.update(EE_ODOMETER_ADDRESS, odometer);
}