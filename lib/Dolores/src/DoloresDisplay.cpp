#include "DoloresDisplay.h"

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

void DoloresDisplay::displayOdometerAlphanumeric(float input) {
    String result = String(input, DEC);
    int loc = result.indexOf('.');
    result = result.substring(0, loc);
    result = "ODO-" + result + (DoloresDatabase::isMetric() ? "K" : "M") ;
    scrollAlphaNumericDisaplay(result);
}

void DoloresDisplay::scrollAlphaNumericDisaplay(String input) {
    input = input + "    ";
    char buf[input.length()];
    input.toCharArray(buf, input.length());
    char displaybuffer[4] = {' ', ' ', ' ', ' '};
    for (uint8_t i = 0; i < input.length() - 1 ; i++ ) {
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

void DoloresDisplay::digitalDisplay(boolean isOil, boolean isRunning, boolean isLeftTurn, boolean isRightTurn, int displaySpeed) {
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
        uint8_t kph[] = { 0x00, 0x00, 0x00, (DoloresDatabase::isMetric() ? (uint8_t)SEG_A : (uint8_t)SEG_D)};
        numeric4->setSegments(kph);
        numeric4->showNumberDec(displaySpeed, true, 3, 0);
        }
    } else {
        numeric4->setSegments(SEG_READY);
    }
    }
}

void DoloresDisplay::alphaNumericDisplay(boolean isOil, boolean isRunning, boolean isLeftTurn, boolean isRightTurn, int displaySpeed) {
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

void DoloresDisplay::displayOdometerDigital (float input) {
    int odometer = (int) input;

    int digits = ((int) pow(odometer , 0.1)) + 1;
    for (int i = 0; i < digits ; i++) {
    int display_value = getDisplayValue(digits, i, odometer);
    numeric4->showNumberDec(display_value, i > 3, 3, 0);
    delay(SCROLL_SPEED);
    }
}

int DoloresDisplay::getDisplayValue(int digits, int offset , int reading) {
    int x0 = extractDigit(reading, digits - offset);
    int x1 = extractDigit(reading, digits - offset + 1);
    int x2 = extractDigit(reading, digits - offset + 2);

    return  (x2 * 100) + (x1 * 10) + x0;
}

int DoloresDisplay::extractDigit(int v, int p) {
    return int(v / (pow(10, p - 1))) - int(v / (pow(10, p))) * 10;
}

DoloresDisplay::DoloresDisplay(byte pin_CLK_SDA, byte pin_DIO_SCL, boolean isAlphanumeric = true) {
    this->pin_CLK_SDA = pin_CLK_SDA;
    this->pin_DIO_SCL = pin_DIO_SCL;
    this->isAlphanumeric = isAlphanumeric;
    this->alpha4 = Adafruit_AlphaNum4();
    this->numeric4 = new TM1637Display(pin_CLK_SDA, pin_DIO_SCL);
    alpha4.begin(0x70);
}

void DoloresDisplay::scrollText(String value) {
    if (isAlphanumeric) {
    scrollAlphaNumericDisaplay(value);
    } else {
    delay(2000);
    }
}

void DoloresDisplay::scrollOdometer(float odometer) {
    if (isAlphanumeric) {
    displayOdometerAlphanumeric(odometer);
    } else {
    displayOdometerDigital(odometer);
    }
}

void DoloresDisplay::setDisplay(boolean isOil, boolean isRunning, boolean isLeftTurn, boolean isRightTurn, int displaySpeed) {
    if (isAlphanumeric) {
    alphaNumericDisplay(isOil, isRunning, isLeftTurn, isRightTurn, displaySpeed);
    } else {
    digitalDisplay(isOil, isRunning, isLeftTurn, isRightTurn, displaySpeed);
    }
}
