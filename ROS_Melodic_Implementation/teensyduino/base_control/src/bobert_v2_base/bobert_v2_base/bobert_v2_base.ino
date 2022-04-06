/* +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ Wheels +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ */
// Library functions from: https://github.com/adamlm/tb9051ftg-motor-carrier-arduino
// Modified for our application
#include <TB9051FTGMotorCarrier.h>

// Based on our wheel-base of 8.5in = 0.2159m, a conversion of m to rad is :
#define RAD_TO_MTR 0.10795      // m/rad , thus if you have a z in rad/s, then z * RAD_TO_MTR = m/s for one wheel
// The wheel tuning constant: without this, the robot skews leftward, so slow down the right motor,
#define R_MOTOR_TUNING  0.96
// Define the max x (m/s) and rotational that can be issued from
// Based on our experimentation, the max operational speed is 0.27686 m/s
// At half speed, this is 0.13208. We want to define half as the max linear speed:
#define MAX_VEL 0.13208         // m/s, x - linear
#define MAX_THETA 1.2235        // rad/s, z - rotational

/* ---------- LEFT ---------- */
// TB9051FTGMotorCarrier pin definitions
static constexpr uint8_t pwm1Pin_L{7};
static constexpr uint8_t pwm2Pin_L{8};
// Instantiate TB9051FTGMotorCarrier
static TB9051FTGMotorCarrier driver_L{pwm1Pin_L, pwm2Pin_L};

/* ---------- RIGHT ---------- */
// TB9051FTGMotorCarrier pin definitions
static constexpr uint8_t pwm1Pin_R{5};
static constexpr uint8_t pwm2Pin_R{6};
// Instantiate TB9051FTGMotorCarrier
static TB9051FTGMotorCarrier driver_R{pwm1Pin_R, pwm2Pin_R};

/* +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ Servos +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ */
// Added servo controller library from https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
#include <Adafruit_PWMServoDriver.h>

// The factor and delay constants used for servo motion on the arm.
#define SERVOFACTOR 50
#define SERVODELAY 50

// Global servo controller object
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);

// Global servo position array
float servo_positions[6];

// Max and min raw pwm values for the servos by experimentation.
int servomin[6] = {150, 100, 150, 150, 130, 100};
int servomax[6] = {500, 500, 650, 600, 570, 300};


/* +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ Sensors +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ */
// Added ultrasonic sensor library from https://github.com/Martinsos/arduino-lib-hc-sr04
#include <HCSR04.h>
#include <Wire.h>   // used to set pin to I2C if we have issues with it not defaulting to such

// The pins used for the line trackers, left, middle, and right.
#define TRACK_L 21
#define TRACK_M 22
#define TRACK_R 23
// The threshold of the raw analog value between black and non-black, if the reading is above this value, it's black.
#define TRACK_THRESHOLD 900
// Global light sensor status in the format of {left, middle, right}, if an element is 1, then that sensor detects the white line, otherwise it does not.
#define L_LINE 0
#define M_LINE 1
#define R_LINE 2
int lineStatus[3] = {0, 0, 0};

#define frontSize 3
#define frontLeftSize 3
#define frontRightSize 3
#define rightLowSize 3
#define rightHighSize 3
#define clawSize 10
#define startDistance 5
#define distanceDelay 100
#define CHECK_FRONT_DIST 1
#define NO_CHECK_FRONT_DIST 0
float frontDistances[frontSize], frontLeftDistances[frontLeftSize], frontRightDistances[frontRightSize], rightLowDistances[rightLowSize], rightHighDistances[rightHighSize], clawDistances[clawSize];
float frontDistance, frontLeftDistance, frontRightDistance, rightLowDistance, rightHighDistance, clawDistance;
int frontIndex, frontLeftIndex, frontRightIndex, rightLowIndex, rightHighIndex, clawIndex;
// Global ultrasonic sensor objects running on pins param1 and param2
UltraSonicDistanceSensor frontSensor(0, 1);
UltraSonicDistanceSensor frontLeftSensor(24, 25);
UltraSonicDistanceSensor frontRightSensor(26, 27);
UltraSonicDistanceSensor rightLowSensor(41, 40);
UltraSonicDistanceSensor rightHighSensor(39, 38);
UltraSonicDistanceSensor clawSensor(37, 36);

volatile int triggered = 0;
volatile uint64_t cupCount = 0;
volatile uint64_t netCount = 0;
volatile uint64_t treeCount = 0;
#define S_NONE 0
#define S_CUP 1
#define S_TREE 2
volatile int cameraState = S_NONE;
volatile int grabbed = 0;

volatile int on = 0;

/* +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ MAIN FUNCTIONS +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ */
void setup() {
  // Init the motor carrier handles
  driver_L.enable();
  driver_L.setBrakeMode(1); // Break when no output, rather than coast
  driver_L.setOutput(0.0);

  driver_R.enable();
  driver_R.setBrakeMode(1); // Break when no output, rather than coast
  driver_R.setOutput(0.0);

  pinMode(TRACK_L, INPUT);
  pinMode(TRACK_M, INPUT);
  pinMode(TRACK_R, INPUT);

  //I2C init for the servo controllers, may not be needed
  //Wire.setSCL(19);
  //Wire.setSDA(18);
  //Wire.begin();

  // Initialize the data globals used for the ultra sonic sensors
  initDistances();

  pwm1.begin(); // For servo controller
  pwm1.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  // Init positions
  //servo_positions[5] = 90;
  //servo_positions[4] = 90;
  //servo_positions[3] = 30;
  //servo_positions[2] = 150;
  //servo_positions[1] = 90;
  //servo_positions[0] = 90;
  //updateServo();
  //moveToRest();

  Serial.begin(9600);
  Serial.println("Beginning the Line Following Test");

  pinMode(30, INPUT);
  pinMode(31, INPUT);
  pinMode(32, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(32), ISR_Rising, RISING);

  attachInterrupt(digitalPinToInterrupt(33), ISR_Start, LOW);

}

void loop() {
  // Our state machine for line following in here...
  while(!on);
  linearFollow(0.5, NO_CHECK_FRONT_DIST);  
  //delay(500);

  rotRight90(-0.5);
  //delay(500);

  linearFollow(0.5, CHECK_FRONT_DIST); 
  //delay(500);
  backupForTurn(0.5);
  rotLeft180(0.5);
  //delay(500);
  
  linearFollow(0.5, NO_CHECK_FRONT_DIST);
  //delay(500);

  rotLeft90(0.5);
  //delay(500);
  
  linearFollow(0.5, CHECK_FRONT_DIST); 
  while(on);  
}

void updateCameraCount(){
  if((!digitalRead(31)) && (!digitalRead(30))){
    // 00 - . not possible
  }
  if((!digitalRead(31)) && (digitalRead(30))){
    // 01 - cup
    cupCount++;
  }
  if((digitalRead(31)) && (!digitalRead(30))){
    // 10 - net
    netCount++;
  }
  if((digitalRead(31)) && (digitalRead(30))){
    // 11 - tree
    treeCount++;
  }
}

void analyzeCameraCount(){
  // return 0 - net, 1 - cup, 2 - tree
  //uint64_t total = cupCount + netCount + treeCount;
  if((!cupCount) && (!treeCount)){
    cameraState = S_NONE;
  }
  else if(cupCount > treeCount){
    cameraState = S_CUP;
  }
  else{
    cameraState = S_TREE;
  }
}

void ISR_Rising(){
  detachInterrupt(digitalPinToInterrupt(32));
  triggered = 1;
  updateCameraCount();  
  //halt();
  //delay(1000);
  attachInterrupt(digitalPinToInterrupt(32), ISR_Falling, FALLING);
}

void ISR_Falling(){
  detachInterrupt(digitalPinToInterrupt(32));
  triggered = 0;
  analyzeCameraCount();
  cupCount = 0;
  netCount = 0;
  treeCount = 0;
  if((cameraState == S_TREE) && (!grabbed)){
    halt();
    // Rotate a little more
    rotLeft110(0.5);
    // Move foreward
    cmd_vel(0.5,0.0);
    delay(1000);
    halt();
    
    moveToRest();
    moveToGrab();
    moveToRest();

    // Move backwards
    cmd_vel(-0.5,0.0);
    delay(1000);
    halt();
        
    rotRight110(0.5);
    grabbed = 1;
  }
  if((cameraState == S_CUP) && (grabbed)){
    halt();
    // Rotate a little more
    rotLeft110(0.5);
    // Move foreward
    cmd_vel(0.5,0.0);
    delay(1000);
    halt();
    
    moveToRest();
    moveToRelease();
    moveToRest();

    // Move backwards
    cmd_vel(-0.5,0.0);
    delay(1000);
    halt();
    
    rotRight110(0.5);
    grabbed = 0;
  }
  attachInterrupt(digitalPinToInterrupt(32), ISR_Rising, RISING);
}

void ISR_Start(){
  detachInterrupt(digitalPinToInterrupt(33));
  delay(100);
  on = 1;
  Serial.println("Started");
  attachInterrupt(digitalPinToInterrupt(33), ISR_End, HIGH);
}

void ISR_End(){
  detachInterrupt(digitalPinToInterrupt(33));
  delay(100);
  on = 0;
  halt();
  Serial.println("Ended");
  attachInterrupt(digitalPinToInterrupt(33), ISR_Start, LOW);
  while(1); // Wait until reloaded
}

/* +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ NAVIGATION FUNCTIONS +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ */
void halt(){
  driver_L.setOutput(0.0);
  driver_R.setOutput(0.0);
}
void linearFollow(float linearSpeedPercentage, bool checkFrontFlag){
  if (linearSpeedPercentage < 0.1) {
    linearSpeedPercentage = 0.1;  
  }

  float linearVel = MAX_VEL * linearSpeedPercentage;

  /* Refresh Population of the buffer */
  for (int i = 0; i < frontSize; i++) updateDistances();

  while(1){
    if(triggered){
      //halt();
      //break;
      updateCameraCount();      
    }
    
    // Printing for object detection
    float data_C = clawSensor.measureDistanceCm();
    float data_F = frontSensor.measureDistanceCm();
    float data_RL = rightLowSensor.measureDistanceCm();
    float data_RH = rightHighSensor.measureDistanceCm();
    Serial.print("Claw: ");
    Serial.print(data_C);
    Serial.print(" Front: ");
    Serial.print(data_F);
    Serial.print(" Low: ");
    Serial.print(data_RL);
    Serial.print(" High: ");
    Serial.print(data_RH);
    Serial.print('\n');/**/
    
    updateDistances();    // Comply with the API by calling this often

    if (checkFrontFlag == CHECK_FRONT_DIST) {
      // Am I close to the wall?
      if (((frontDistance <= 10.0) && (frontDistance > 0)) && ((frontLeftDistance <= 10.0) && (frontLeftDistance > 0)) && ((frontRightDistance <= 10.0) && (frontRightDistance > 0))) {
        halt();
        break;
      }
    }

    /* Read the sensors */
    updateLineStatus();   // Update the line sensor data array
    if (lineStatus[M_LINE] == 1){
      //cmd_vel(linearVel, 0.0);
      if (lineStatus[R_LINE] && !lineStatus[L_LINE]){
        cmd_vel(linearVel, -0.15);          
      }
      else if (!lineStatus[R_LINE] && lineStatus[L_LINE]) {
        cmd_vel(linearVel, 0.15); 
      }
      // If just the middle line or all three, drive straight
      else {
        cmd_vel(linearVel, 0.0);
      }
    }
    else if (lineStatus[R_LINE]){
      cmd_vel(linearVel, -0.10);
    }
    else if (lineStatus[L_LINE]){
      cmd_vel(linearVel, 0.10);
    }
    else {
      // Drive straight for a ways to center the axle over the line...
      cmd_vel(linearVel, 0.0);
      halt();
      break;
    }
  }
  return;
}
void backupForTurn(float linearSpeedPercentage){
  float linearVel = MAX_VEL * linearSpeedPercentage;
  cmd_vel(-linearVel, 0.0);
  delay(2000);
  halt();
}
void rotRight90(float rotSpeed){
  float turnTune = 1.3; //1.3
  if (rotSpeed > 0) rotSpeed = -rotSpeed;
  if (rotSpeed < -MAX_THETA) rotSpeed = -MAX_THETA;

  int turnTime = ((PI/2) / abs(rotSpeed)) * 1000;
  cmd_vel(0.0, rotSpeed);
  delay (turnTime * turnTune);
  halt();
}
void rotLeft90(float rotSpeed){
  float turnTune = 1.3;
  if (rotSpeed < 0) rotSpeed = -rotSpeed;
  if (rotSpeed > MAX_THETA) rotSpeed = MAX_THETA;

  int turnTime = ((PI/2) / abs(rotSpeed)) * 1000;
  cmd_vel(0.0, rotSpeed);
  delay (turnTime * turnTune);
  halt();
}
void rotRight110(float rotSpeed){
  float turnTune = 1.5; //1.3
  if (rotSpeed > 0) rotSpeed = -rotSpeed;
  if (rotSpeed < -MAX_THETA) rotSpeed = -MAX_THETA;

  int turnTime = ((PI/2) / abs(rotSpeed)) * 1000;
  cmd_vel(0.0, rotSpeed);
  delay (turnTime * turnTune);
  halt();
}
void rotLeft110(float rotSpeed){
  float turnTune = 1.5; //1.3
  if (rotSpeed < 0) rotSpeed = -rotSpeed;
  if (rotSpeed > MAX_THETA) rotSpeed = MAX_THETA;

  int turnTime = ((PI/2) / abs(rotSpeed)) * 1000;
  cmd_vel(0.0, rotSpeed);
  delay (turnTime * turnTune);
  halt();
}
void rotRight180(float rotSpeed){
  float turnTune = 1.3;
  if (rotSpeed > 0) rotSpeed = -rotSpeed;
  if (rotSpeed < -MAX_THETA) rotSpeed = -MAX_THETA;

  int turnTime = (PI / abs(rotSpeed)) * 1000;
  cmd_vel(0.0, rotSpeed);
  delay (turnTime * turnTune);
  halt();
}
void rotLeft180(float rotSpeed){
  float turnTune = 1.3;
  if (rotSpeed < 0) rotSpeed = -rotSpeed;
  if (rotSpeed > MAX_THETA) rotSpeed = MAX_THETA;

  int turnTime = (PI / abs(rotSpeed)) * 1000;
  cmd_vel(0.0, rotSpeed);
  delay (turnTime * turnTune);
  halt();
}
void loopLongStrech(float linearSpeedPercentage) {
  /* Set the robot after the turn looking down the long stretch. This will loop the line following down the whole course */
  //linearFollow(0.25, CHECK_FRONT_DIST);  
  linearFollow(0.3, CHECK_FRONT_DIST);
  delay(1000);
  backupForTurn(linearSpeedPercentage);
  rotLeft180(0.5);
  delay(1000);
  linearFollow(0.3, NO_CHECK_FRONT_DIST);  
  delay(1000);
  rotRight180(-0.5);
} 
/* +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ WHEELS FUNCTIONS +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ */


// Use this function if you want to command the movement in terms of m/s and rad/s :
void cmd_vel(float x, float z_rot){
  // Constrain the inputs:
  if (x> MAX_VEL) x = MAX_VEL;
  else if (x < -MAX_VEL) x = -MAX_VEL;
  if (z_rot > MAX_THETA) z_rot = MAX_THETA;
  else if (z_rot < -MAX_THETA) z_rot = -MAX_THETA;

  // Translate inputs into a % for each wheel
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

  driver_L.setOutput(left_cmd);
  driver_R.setOutput(right_cmd);
}

// USe this command if you want to control by percentages of power to each motor, though I doubt how often you'd use this:
void cmd_percent(float r_percent, float l_percent){
  driver_L.setOutput(l_percent);
  driver_R.setOutput(r_percent);
}

/* +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ SENSOR FUNCTIONS +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ */
// -------------------------------   Line Sensors   ----------------------------------
// Check left light sensor
int readTrackerL(){
  return analogRead(TRACK_L);
}
// Check middle light sensor
int readTrackerM(){
  return analogRead(TRACK_M);
}
// Check right light sensor
int readTrackerR(){
  return analogRead(TRACK_R);
}
// Check all 3 sensors to see if they are within the white line. Updates the global array. 1 = White, 0 = Black
void updateLineStatus(){
  lineStatus[0] = (readTrackerL() < TRACK_THRESHOLD) ? 1 : 0;
  lineStatus[1] = (readTrackerM() < TRACK_THRESHOLD) ? 1 : 0;
  lineStatus[2] = (readTrackerR() < TRACK_THRESHOLD) ? 1 : 0;
}

void updateLineStatus_reversed(){
  lineStatus[0] = (readTrackerL() < TRACK_THRESHOLD) ? 0 : 1;
  lineStatus[1] = (readTrackerM() < TRACK_THRESHOLD) ? 0 : 1;
  lineStatus[2] = (readTrackerR() < TRACK_THRESHOLD) ? 0 : 1;
}
// -------------------------------   Ultra Sensors  ----------------------------------
void initDistances(){
  for(int i = 0; i < frontSize; i++){
    frontDistances[i] = 50;
  }
  frontDistance = 50;
  frontIndex = 0;
  for(int i = 0; i < frontLeftSize; i++){
    frontLeftDistances[i] = 0;
  }
  frontLeftDistance = 0;
  frontLeftIndex = 0;
  for(int i = 0; i < frontRightSize; i++){
    frontRightDistances[i] = 0;
  }
  frontRightDistance = 0;
  frontRightIndex = 0;
  /*for(int i = 0; i < rightLowSize; i++){
    rightLowDistances[i] = 0;
  }
  rightLowDistance = 0;
  rightLowIndex = 0;
  for(int i = 0; i < rightHighSize; i++){
    rightHighDistances[i] = 0;
  }
  rightHighDistance = 0;
  rightHighIndex = 0;*/
  for(int i = 0; i < clawSize; i++){
    clawDistances[i] = 0;
  }
  clawDistance = 0;
  clawIndex = 0;
}

void incIndex(int & index, int maxSize){
  if(index >= maxSize - 1){
    index = 0;
  }
  else{
    index++;
  }
}

void pushBuffer(float * buf, int & index, int maxSize, float val){
  if(val > 0){
    buf[index] = val;
    incIndex(index, maxSize);
  }
}

float getBufferAvg(float * buf, int maxSize){
  float sum = 0;
  for(int i = 0; i < maxSize; i++){
    sum += buf[i];
  }
  return sum / (float)maxSize;
}

void readDistances(){
  float currDistance = frontSensor.measureDistanceCm();
  pushBuffer(frontDistances, frontIndex, frontSize, currDistance);
  currDistance = frontLeftSensor.measureDistanceCm();
  pushBuffer(frontLeftDistances, frontLeftIndex, frontLeftSize, currDistance);
  currDistance = frontRightSensor.measureDistanceCm();
  pushBuffer(frontRightDistances, frontRightIndex, frontRightSize, currDistance);
  /*currDistance = rightLowSensor.measureDistanceCm();
  pushBuffer(rightLowDistances, rightLowIndex, rightLowSize, currDistance);
  currDistance = rightHighSensor.measureDistanceCm();
  pushBuffer(rightHighDistances, rightHighIndex, rightHighSize, currDistance);*/
  currDistance = clawSensor.measureDistanceCm();
  pushBuffer(clawDistances, clawIndex, clawSize, currDistance);
}

void updateDistancesAvg(){
  frontDistance = getBufferAvg(frontDistances, frontSize);
  frontLeftDistance = getBufferAvg(frontLeftDistances, frontLeftSize);
  frontRightDistance = getBufferAvg(frontRightDistances, frontRightSize);
  /*rightLowDistance = getBufferAvg(rightLowDistances, rightLowSize);
  rightHighDistance = getBufferAvg(rightHighDistances, rightHighSize);*/
  clawDistance = getBufferAvg(clawDistances, clawSize);
}
// Our API requires that you call this function often so the decay average can collect data points
// Put it in a high freq loop, then read frontDistance when you want the value;
void updateDistances(){
  
  readDistances();
  updateDistancesAvg();
}


/* +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ ARM FUNCTIONS +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+ */
// Convert angle to pulse to send to the servo controllers
int angleToPulse(int ang, int servo){
  int pulse = map(ang, 0, 180, servomin[servo], servomax[servo]); // map angle of 0 to 180 to Servo min and Servo max 
  Serial.print("Angle: ");Serial.print(ang);
  Serial.print(" pulse: ");Serial.println(pulse);
  return pulse;
}
// Move one servo to a specific angle
void moveToAngle(float angle, int servo){
  float current = servo_positions[servo];
  float diff = (angle - current) / SERVOFACTOR;
  for(int i = 0; i < SERVOFACTOR; i++){
    servo_positions[servo] = current + diff;
    current = servo_positions[servo];
    pwm1.setPWM(servo, 0, angleToPulse(servo_positions[servo], servo));
    delay(SERVODELAY);
  }
}
// Update all servos to the positions in the global array
void updateServo(){
  for(int i = 0; i < 6; i++){
    delay(100);
    pwm1.setPWM(i, 0, angleToPulse(servo_positions[i], i));
  }
}
// Grabbing position, the arm yanks back and slowly moves forward to grab
void moveToGrab(){
  moveToAngle(0, 5);
  moveToAngle(30, 4);
  moveToAngle(110, 3);
  moveToAngle(120, 2);
  moveToAngle(90, 1);
  moveToAngle(85, 0);
  moveToAngle(180, 5);
}
// Resting position, the arm slowly yanks back and then folds
void moveToRest(){
  moveToAngle(30, 3); //10
  moveToAngle(120, 2); //150
  moveToAngle(0, 1);
  moveToAngle(85, 0);
  moveToAngle(180, 5);
  moveToAngle(130, 4);
  moveToAngle(30, 3);
  moveToAngle(0, 2);
}
// Release position, the arm yanks back and reaches in front to release
void moveToRelease(){
  moveToAngle(120, 2);
  moveToAngle(180, 5);
  moveToAngle(130, 4);
  moveToAngle(150, 3);
  moveToAngle(150, 1);
  moveToAngle(85, 0);
  moveToAngle(0, 5);
}
// Sweeping motion, the arm sweeps in front of it.
void moveToSweep(int times){
  moveToAngle(90, 5);
  moveToAngle(130, 4);
  moveToAngle(150, 3);
  moveToAngle(120, 2);
  moveToAngle(180, 1);
  moveToAngle(85, 0);
  for(int i = 0; i < times; i++){
    moveToAngle(45, 0);
    delay(10);
    moveToAngle(135, 0);
    delay(10);  
  }
  moveToAngle(85, 0);
}
