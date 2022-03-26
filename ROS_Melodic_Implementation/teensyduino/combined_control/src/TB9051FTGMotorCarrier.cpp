#include "TB9051FTGMotorCarrier.h"

TB9051FTGMotorCarrier::TB9051FTGMotorCarrier(const uint8_t pwm1,
                                               const uint8_t pwm2,
                                               const uint8_t ocm,
                                               const uint8_t diag,
                                               const uint8_t occ,
                                               const uint8_t en,
                                               const uint8_t enb) :
                                               pwm1(pwm1),
                                               pwm2(pwm2),
                                               ocm(ocm),
                                               diag(diag),
                                               occ(occ),
                                               en(en),
                                               enb(enb),
                                               brakeMode(false),
                                               deadbandLower(0),
                                               deadbandUpper(0) {

    pinMode(this->pwm1, OUTPUT);
    pinMode(this->pwm2, OUTPUT);

    if (this->ocm != kPinNotUsed) {
        pinMode(this->ocm, INPUT);
    }

    if (this->diag != kPinNotUsed) {
        pinMode(this->diag, INPUT);
    }

    if (this->occ != kPinNotUsed) {
        pinMode(this->occ, OUTPUT);
    }

    if (this->en != kPinNotUsed) {
        pinMode(this->en, OUTPUT);
    }

    if (this->enb != kPinNotUsed) {
        pinMode(this->enb, OUTPUT);
    }
}

float TB9051FTGMotorCarrier::getCurrent(void) const {
    if (this->ocm != kPinNotUsed) {
        // 4.9 mV per analog unit (for Arduino Uno)
        // 500 mV per A on TB9051FTG current sense
        // 1000 mA per A
        return (analogRead(this->ocm) * 4.9) / 500 * 1000;
    }

    return kPinNotUsed;
}

uint8_t TB9051FTGMotorCarrier::getDiagnostic(void) const {
    return this->diag != kPinNotUsed? digitalRead(this->diag): kPinNotUsed;
}

void TB9051FTGMotorCarrier::setDeadband(const float lower, const float upper) {
    this->deadbandLower = lower;
    this->deadbandUpper = upper;
}

void TB9051FTGMotorCarrier::setBrakeMode(const bool brakeMode) {
    this->brakeMode = brakeMode;
}

void TB9051FTGMotorCarrier::setOutput(const float percent) const {
    const auto output{static_cast<uint8_t>(abs(percent) * 255)};

    if (withinDeadband(percent)) {
        if (this->brakeMode) {
            analogWrite(this->pwm1, 0);
            analogWrite(this->pwm2, 0);
        } else {
            disableOutputs();
        }
    } else if (this->enabled && percent < 0) {
        enableOutputs();
        analogWrite(this->pwm1, 0);
        analogWrite(this->pwm2, output);
    } else if (this->enabled) {
        enableOutputs();
        analogWrite(this->pwm1, output);
        analogWrite(this->pwm2, 0);
    }
}

void TB9051FTGMotorCarrier::setOcc(const uint8_t value) const {
    if (this->occ != kPinNotUsed) {
        digitalWrite(this->occ, value);
    }
}

void TB9051FTGMotorCarrier::enable(void) {
    this->enabled = true;
    enableOutputs();
}

void TB9051FTGMotorCarrier::disable(void) {
    this->enabled = false;
    disableOutputs();
}

bool TB9051FTGMotorCarrier::withinDeadband(const float value) const {
    return value < this->deadbandUpper && value > this->deadbandLower;
}

void TB9051FTGMotorCarrier::enableOutputs(void) const {
    if (this->en != kPinNotUsed) {
        digitalWrite(this->en, 1);
    }

    if (this->en != kPinNotUsed) {
        digitalWrite(this->enb, 0);
    }
}

void TB9051FTGMotorCarrier::disableOutputs(void) const {
    if (this->en != kPinNotUsed) {
        digitalWrite(this->en, 0);
    }

    if (this->enb != kPinNotUsed) {
        digitalWrite(this->enb, 1);
    }
}
