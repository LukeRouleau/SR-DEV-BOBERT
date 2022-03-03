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
static TB9051FTGMotorCarrier driver_L{pwm1Pin_L, pwm2Pin_L};
static float throttlePercent_L{0.0f};

/* ---------- RIGHT ---------- */
// TB9051FTGMotorCarrier pin definitions
static constexpr uint8_t pwm1Pin_R{7};
static constexpr uint8_t pwm2Pin_R{8};
// Instantiate TB9051FTGMotorCarrier
static TB9051FTGMotorCarrier driver_R{pwm1Pin_R, pwm2Pin_R};
static float throttlePercent_R{0.0f};

/* ---------- DEFAULT SPEED ---------- */
const float default_L = 0.5; // power % to make the wheel travel at 1m/s
const float default_R = 0.5; // power % to make the wheel travel at 1m/s

// Create rosserial node handle
ros::NodeHandle nh;

void cmd_vel_cb(const geometry_msgs::Twist & msg){
    // Read the message and act accordingly
    const float x = msg.linear.x;
    const float z_rot = msg.angular.z;

    int right_cmd = x * 100 * min(1, max(z_rot*1.5 + 1, -1));
    int left_cmd = x * 100 * min(1, max(-z_rot*1.5 + 1, -1));

    //int right_cmd = x * 100;  // This is what I did for testing 
    //int left_cmd = x * 100;   // This is what I did for testing

    float throttlePercent_R = (float)right_cmd / 255;
    float throttlePercent_L = (float)left_cmd / 255;

    driver_L.setOutput(throttlePercent_L);
    driver_R.setOutput(throttlePercent_R);
}

ros::Subscriber<geometry_msgs::Twist> sub("turtle1/cmd_vel", cmd_vel_cb);

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
