// Library functions from: https://github.com/adamlm/tb9051ftg-motor-carrier-arduino
// Modified for our application

#include <TB9051FTGMotorCarrier.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

// Define your operation type here:
#define STD_OPERATION
#define CMD_VEL         // for normal subscription to /cmd_vel
//#define TURTLE_CMD_VEL  // for subscription to /turtle1/cmd_vel

// Define the max cmd_vel that is issued from move_base
// Based on our experimentation, the max operational speed is 0.27686 m/s
// At half speed, this is 0.13208
#define MAX_VEL 0.13208         // m/s, x - linear
#define MAX_THETA 1.2235        // rad/s, z - rotational

// Based on our wheel-base of 8.5in = 0.2159m, a conversion of m to rad is :
#define RAD_TO_MTR 0.10795      // m/rad , thus if you have a z in rad/s, then z * RAD_TO_MTR = m/s for one wheel

// The wheel tuning constant: without this, the robot skews leftward, so slow down the right motor,
#define R_MOTOR_TUNING  0.96



/* ---------- LEFT ---------- */
// TB9051FTGMotorCarrier pin definitions
static constexpr uint8_t pwm1Pin_L{7};
static constexpr uint8_t pwm2Pin_L{8};
// Instantiate TB9051FTGMotorCarrier
static TB9051FTGMotorCarrier driver_L{pwm1Pin_L, pwm2Pin_L};
static float throttlePercent_L{0.0f};

/* ---------- RIGHT ---------- */
// TB9051FTGMotorCarrier pin definitions
static constexpr uint8_t pwm1Pin_R{5};
static constexpr uint8_t pwm2Pin_R{6};
// Instantiate TB9051FTGMotorCarrier
static TB9051FTGMotorCarrier driver_R{pwm1Pin_R, pwm2Pin_R};
static float throttlePercent_R{0.0f};

/* ---------- DEFAULT SPEED ---------- */
const float default_L = 0.5; // power % to make the wheel travel at 1m/s
const float default_R = 0.5; // power % to make the wheel travel at 1m/s

#ifdef STD_OPERATION
// Create rosserial node handle
ros::NodeHandle nh;

// Used for the actual 
void cmd_vel_cb(const geometry_msgs::Twist & msg){
    // Read the message and act accordingly
    const float x = msg.linear.x;
    const float z_rot = msg.angular.z;

    // Translate this into a % for each wheel
    float right_cmd = 0.0;
    float left_cmd = 0.0;
    if (z_rot > 0) {
        right_cmd = x + min((z_rot * RAD_TO_MTR), MAX_THETA * RAD_TO_MTR);
        left_cmd = x - min((z_rot * RAD_TO_MTR), MAX_THETA * RAD_TO_MTR);
    }
    else {
        right_cmd = x - min((abs(z_rot) * RAD_TO_MTR), MAX_THETA * RAD_TO_MTR);
        left_cmd = x + min((abs(z_rot) * RAD_TO_MTR), MAX_THETA * RAD_TO_MTR);
    }

    // Apply the scaling constant & convert to a %:
    right_cmd = (min(right_cmd, (2 * MAX_VEL)) / (2 * MAX_VEL)) * R_MOTOR_TUNING;
    left_cmd  = min(left_cmd, (2 * MAX_VEL)) / (2 * MAX_VEL);

    //int right_cmd = x * 100 * min(1, max(z_rot*1.5 + 1, -1));
    //int left_cmd = x * 100 * min(1, max(-z_rot*1.5 + 1, -1));

    //int right_cmd = x * 100;  // This is what I did for testing 
    //int left_cmd = x * 100;   // This is what I did for testing

    driver_L.setOutput(left_cmd);
    driver_R.setOutput(right_cmd);
}

/* ---------- Set up the subscriber ---------- */
#ifdef CMD_VEL
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);
#else
ros::Subscriber<geometry_msgs::Twist> sub("turtle1/cmd_vel", cmd_vel_cb);
#endif

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
}
#else
void setup() {
    // Init the motor carrier handles
    driver_L.enable();
    driver_L.setBrakeMode(1); // Break when no output, rather than coast
    driver_L.setOutput(throttlePercent_L);

    driver_R.enable();
    driver_R.setBrakeMode(1); // Break when no output, rather than coast
    driver_R.setOutput(throttlePercent_R);
}
#endif

#ifdef LINEAR_MAX_TEST
void loop() {
    // Run for 10s
    driver_L.setOutput(1.0);
    driver_R.setOutput(1.0 * R_MOTOR_TUNING);
    for (auto i{0}; i < 10; i++) {
        delay(1000);
    }
    // Stop for 5s
    driver_L.setOutput(0.0);
    driver_R.setOutput(0.0);
    for (auto i{0}; i < 5; i++) {
        delay(1000);
    }
}
#endif

#ifdef LINEAR_HALF_TEST
void loop() {
    // Run for 10s
    driver_L.setOutput(0.5);
    driver_R.setOutput(0.5 * R_MOTOR_TUNING);
    for (auto i{0}; i < 10; i++) {
        delay(1000);
    }
    // Stop for 5s
    driver_L.setOutput(0.0);
    driver_R.setOutput(0.0);
    for (auto i{0}; i < 5; i++) {
        delay(1000);
    }
}
#endif

#ifdef ANGULAR_MAX_CW_TEST
void loop() {
    // Run for 10s
    driver_L.setOutput(1.0);
    driver_R.setOutput(-1.0);
    for (auto i{0}; i < 10; i++) {
        delay(1000);
    }
    // Stop for 5s
    driver_L.setOutput(0.0);
    driver_R.setOutput(0.0);
    for (auto i{0}; i < 5; i++) {
        delay(1000);
    }
}
#endif

#ifdef ANGULAR_HALF_CW_TEST
void loop() {
    // Run for 10s
    driver_L.setOutput(0.5);
    driver_R.setOutput(-0.5 * R_MOTOR_TUNING);
    for (auto i{0}; i < 10; i++) {
        delay(1000);
    }
    // Stop for 5s
    driver_L.setOutput(0.0);
    driver_R.setOutput(0.0);
    for (auto i{0}; i < 5; i++) {
        delay(1000);
    }
}
#endif

#ifdef ANGULAR_MAX_CC_TEST
void loop() {
    // Run for 10s
    driver_L.setOutput(-1.0);
    driver_R.setOutput(1.0);
    for (auto i{0}; i < 10; i++) {
        delay(1000);
    }
    // Stop for 5s
    driver_L.setOutput(0.0);
    driver_R.setOutput(0.0);
    for (auto i{0}; i < 5; i++) {
        delay(1000);
    }
}
#endif

#ifdef ANGULAR_HALF_CC_TEST
void loop() {
    // Run for 10s
    driver_L.setOutput-0.5);
    driver_R.setOutput(0.5);
    for (auto i{0}; i < 10; i++) {
        delay(1000);
    }
    // Stop for 5s
    driver_L.setOutput(0.0);
    driver_R.setOutput(0.0);
    for (auto i{0}; i < 5; i++) {
        delay(1000);
    }
}
#endif
