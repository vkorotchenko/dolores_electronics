 #include "DoloresTimer.h"
 
#define TURN_SIGNAL_BLINK_DELAY 800
 bool DoloresTimer::isTimeForBlink(){
    return (millis() % (2 * TURN_SIGNAL_BLINK_DELAY)) < TURN_SIGNAL_BLINK_DELAY;
 }
