#ifndef DOLORES_DATABASE_H
#define DOLORES_DATABASE_H
#include <Arduino.h>

class DoloresDatabase {

  private:
    static void checkEEPROM() ;
  public:
    static void setKph();
    static void setMph();
    static boolean isMetric();
    static float getOdometer();
    static void updateOdometer(float odometer);
};
#endif