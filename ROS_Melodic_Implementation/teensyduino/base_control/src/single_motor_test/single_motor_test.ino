// Library functions from: https://github.com/adamlm/tb9051ftg-motor-carrier-arduino
// Modified for our application

#include <TB9051FTGMotorCarrier.h>

/* ---------- LEFT ---------- */
// TB9051FTGMotorCarrier pin definitions
static constexpr uint8_t pwm1Pin{5};
static constexpr uint8_t pwm2Pin{6};
// Instantiate TB9051FTGMotorCarrier
static TB9051FTGMotorCarrier driver{pwm1Pin, pwm2Pin};
static float throttlePercent{0.0f};

/* ---------- DEFAULT SPEED ---------- */
// power % to make the wheel travel at 1m/s

void setup() {
    // Init the motor carrier handles
    driver.enable();
    driver.setBrakeMode(1); // Break when no output, rather than coast
    driver.setOutput(throttlePercent);
}

void loop() {
    // Ramp up to full forward throttle
    for (auto i{0}; i < 10; i++) {
        driver.setOutput(throttlePercent);
        throttlePercent += 0.1;
        delay(1000);
    }

    // Ramp down to 0 throttle
    for (auto i{0}; i < 10; i++) {
        driver.setOutput(throttlePercent);
        throttlePercent -= 0.1;
        delay(1000);
    }

    // Ramp up to full reverse throttle
    for (auto i{0}; i < 10; i++) {
        driver.setOutput(throttlePercent);
        throttlePercent -= 0.1;
        delay(1000);
    }

    // Ramp up to 0 throttle
    for (auto i{0}; i < 10; i++) {
        driver.setOutput(throttlePercent);
        throttlePercent += 0.1;
        delay(1000);
    }
}
