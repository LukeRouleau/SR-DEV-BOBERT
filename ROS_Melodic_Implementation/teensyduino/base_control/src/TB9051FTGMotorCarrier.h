#pragma once

#include <Arduino.h>

constexpr uint8_t kPinNotUsed{255};

class TB9051FTGMotorCarrier {
public:
    /**
     * Custom constructor.
     */
    TB9051FTGMotorCarrier(uint8_t pwm1,
                           uint8_t pwm2,
                           uint8_t ocm = kPinNotUsed,
                           uint8_t diag = kPinNotUsed,
                           uint8_t occ = kPinNotUsed,
                           uint8_t en = kPinNotUsed,
                           uint8_t enb = kPinNotUsed);

    // Getters
    float getCurrent(void) const;
    uint8_t getDiagnostic(void) const;

    // Setters
    void setDeadband(float lower, float upper);
    void setBrakeMode(bool mode);
    void setOutput(float percent) const;
    void setOcc(uint8_t value) const;

    // Other functions
    void enable(void);
    void disable(void);

private:
    bool withinDeadband(float value) const;
    void enableOutputs(void) const;
    void disableOutputs(void) const;

    // Deadband values
    float deadbandLower;
    float deadbandUpper;

    bool brakeMode;  // true if motor in brake mode
    bool enabled;  // true if driver enabled

    // Pin values
    uint8_t occ;  // Over current (OCC) response
    uint8_t en;  // Enable (EN)
    uint8_t enb;  // Inverted enable (ENB)
    uint8_t pwm1;  // PWM for OUT1 (PWM1)
    uint8_t pwm2;  // PWM for OUT2 (PWM2)
    uint8_t ocm;  // Output current monitor (OCM)
    uint8_t diag;  // Diagnostic error (DIAG)
};
