// Library functions from: https://github.com/adamlm/tb9051ftg-motor-carrier-arduino
// Modified for our application

#include <TB9051FTGMotorCarrier.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

/* ---------- LEFT ---------- */
// TB9051FTGMotorCarrier pin definitions
static constexpr uint8_t pwm1Pin_L{5};
static constexpr uint8_t pwm2Pin_L{6};
// Instantiate TB9051FTGMotorCarrier
static TB9051FTGMotorCarrier driver_L{pwm1Pin, pwm2Pin};
static float throttlePercent_L{0.0f};

/* ---------- RIGHT ---------- */
// TB9051FTGMotorCarrier pin definitions
static constexpr uint8_t pwm1Pin_R{5};
static constexpr uint8_t pwm2Pin_R{6};
// Instantiate TB9051FTGMotorCarrier
static TB9051FTGMotorCarrier driver_R{pwm1Pin, pwm2Pin};
static float throttlePercent_R{0.0f};

/* ---------- DEFAULT SPEED ---------- */
const float default_L = 0.5; // To make the wheel travel at 1m/s
const float default_R = 0.5; // To make the wheel travel at 1m/s

// Create rosserial node handle
ros::NodeHandle nh;

void cmd_vel_cb(const geometry_msgs::Twist & msg){
    // Read the message and act accordingly
    const float x = msg.linear.x;
    const float z_rot = msg.angular.z;

    float right_cmd = {SOME EQUATION HERE};
    float left_cmd = {SOME EQUATION HERE};


}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);

void setup() {
    // Init the motor carrier handles
    driver_L.enable();
    driver_L.setBrakeMode(1); // Break when no output, rather than coast
    driver_L.setOutput(throttlePercent_L);

    driver_R.enable();
    driver_R.setBrakeMode(1); // Break when no output, rather than coast
    driver_R.setOutput(throttlePercent_R);

    // Init the node handle
    nh.initNode();
    nh.subscribe(sub);
}

void loop() {



    nh.spinOnce();
    delay(1);


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
