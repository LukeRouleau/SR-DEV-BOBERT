#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ros.h>
#include <std_msgs/Float64.h>

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
//Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

//#define SERVOMIN  120 // this is the 'minimum' pulse length count (out of 4096)
//#define SERVOMAX  630 // this is the 'maximum' pulse length count (out of 4096)

/*#define SERVOMIN5  150
#define SERVOMAX5  300
#define SERVOMIN4  130
#define SERVOMAX4  570
#define SERVOMIN3  250
#define SERVOMAX3  600
#define SERVOMIN2  150
#define SERVOMAX2  610
#define SERVOMIN1  130
#define SERVOMAX1  600
#define SERVOMIN0  150
#define SERVOMAX0  610*/

int servomin[6] = {150, 130, 150, 250, 130, 150};
int servomax[6] = {610, 600, 610, 600, 570, 300};
uint8_t servonum = 0;

std_msgs::Float64 servo_positions[6];

void callback0(const std_msgs::Float64 &msg){
  servo_positions[0].data = msg.data;
}
void callback1(const std_msgs::Float64 &msg){
  servo_positions[1].data = msg.data;
}
void callback2(const std_msgs::Float64 &msg){
  servo_positions[2].data = msg.data;
}
void callback3(const std_msgs::Float64 &msg){
  servo_positions[3].data = msg.data;
}
void callback4(const std_msgs::Float64 &msg){
  servo_positions[4].data = msg.data;
}
void callback5(const std_msgs::Float64 &msg){
  servo_positions[5].data = msg.data;
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Float64> sub0("servo_position_0", callback0);
ros::Subscriber<std_msgs::Float64> sub1("servo_position_1", callback1);
ros::Subscriber<std_msgs::Float64> sub2("servo_position_2", callback2);
ros::Subscriber<std_msgs::Float64> sub3("servo_position_3", callback3);
ros::Subscriber<std_msgs::Float64> sub4("servo_position_4", callback4);
ros::Subscriber<std_msgs::Float64> sub5("servo_position_5", callback5);

void setup() {

  nh.initNode();

  nh.subscribe(sub0);
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);
  nh.subscribe(sub5); 

  //Wire.setSCL(19);
  //Wire.setSDA(18);
  //Wire.begin();
  
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");

  pwm1.begin();
  pwm1.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  //pwm2.begin();
  //pwm2.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  //yield();
}

void loop() {

  nh.spinOnce();
  delay(1);

  for(int i = 0; i < 6; i++){
    pwm1.setPWM(i, 0, angleToPulse(servo_positions[i].data, i));
    delay(500);
  }  

  /*for(int angle = 0; angle < 181; angle += 20){
    for(int i = 0; i < 6; i++){
      delay(500);
      pwm1.setPWM(i, 0, angleToPulse(angle, i));
    }
  }*/

}

int angleToPulse(int ang, int servo){
   int pulse = map(ang, 0, 180, servomin[servo], servomax[servo]); // map angle of 0 to 180 to Servo min and Servo max 
   Serial.print("Angle: ");Serial.print(ang);
   Serial.print(" pulse: ");Serial.println(pulse);
   return pulse;
}