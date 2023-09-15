//Define Compass
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#define OUTPUT_READABLE_YAWPITCHROLL
MPU6050 mpu;

//#define LED_PIN 13
bool blinkState = false;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


volatile bool mpuInterrupt = false;     
void dmpDataReady() {
    mpuInterrupt = true;
}

//Define all pins
#define trigPinR 22
#define echoPinR 23
#define trigPinL 24
#define echoPinL 25
#define trigPinF 26
#define echoPinF 27
#define trigPinB 28
#define echoPinB 29

#define LFDirection 6
#define LFPWM 7
#define LBDirection 8
#define LBPWM 9
#define RFDirection 10
#define RFPWM 11
#define RBDirection 12
#define RBPWM 13

int minimum_distance = 9;
int side_distance = 4;
long duration, distance, RightSensor,BackSensor,FrontSensor,LeftSensor;
int fullSpeedCCW = 200; //Direction Pin is LOW
int fullSpeedCW = 255-fullSpeedCCW; //Direction Pin is HIGH
int minorSpeedCCW = 100; //Direction Pin is LOW
int minorSpeedCW = 225-minorSpeedCCW; //Direction Pin is HIGH
String currentDirection = "F";
long CurrentAngle;
long RefAngle;
long MinAngle = 3;
int stupidCount = 1;


void setup() {
  //Setup Compass
  
      #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
      #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
      #endif
        Serial.begin(9600);
        mpu.initialize();
        devStatus = mpu.dmpInitialize();
        mpu.setXGyroOffset(220);
        mpu.setYGyroOffset(76);
        mpu.setZGyroOffset(-85);
        mpu.setZAccelOffset(1788);
        
        if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    //pinMode(LED_PIN, OUTPUT);

  //Setup Rover
  Serial.begin(9600);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinF, OUTPUT);
  pinMode(echoPinF, INPUT);
  pinMode(trigPinB, OUTPUT);
  pinMode(echoPinB, INPUT);
  
  pinMode(LFDirection, OUTPUT);
  pinMode(LFPWM, OUTPUT);
  pinMode(LBDirection, OUTPUT);
  pinMode(LBPWM, OUTPUT);
  pinMode(RFDirection, OUTPUT);
  pinMode(RFPWM, OUTPUT);
  pinMode(RBDirection, OUTPUT);
  pinMode(RBPWM, OUTPUT);

  delay(30000);
  
  GetRefAngle();
  delay(100);

  GetAngle();
  delay(100);

  GetAngle();
  delay(100);

  GetAngle();
  delay(100);

  RefAngle = CurrentAngle;
  

  delay(1000);
  
  
  checkDistance();
  if(FrontSensor > minimum_distance){
    currentDirection = "F";  
  }

  else if(LeftSensor > minimum_distance){
    currentDirection = "L";
  }

  else if(RightSensor > minimum_distance){
    currentDirection = "R";
  }

  else{
    currentDirection = "B";
  }
}

void loop() {
  if (currentDirection == "F"){
    GoFrontEnd();
  }
  
  if (currentDirection == "B"){
    GoBackEnd();
  }
  
  if (currentDirection == "L"){
    GoLeftEnd();
  }
  
  if (currentDirection == "R"){
    GoRightEnd();
  }

  Serial.print("Current Direction = ");
  Serial.println(currentDirection);
}

void SonarSensor(int trigPin,int echoPin)
{
digitalWrite(trigPin, LOW);
delay(10);
digitalWrite(trigPin, HIGH);
delay(10);
digitalWrite(trigPin, LOW);
duration = pulseIn(echoPin, HIGH);
distance = 0.034*(duration/2);
}

void checkDistance(){
  // sensoring 4 directions
  SonarSensor(trigPinR, echoPinR);
  RightSensor = distance;
  SonarSensor(trigPinL, echoPinL);
  LeftSensor = distance;
  SonarSensor(trigPinF, echoPinF);
  FrontSensor = distance;
  SonarSensor(trigPinB, echoPinB);
  BackSensor = distance;
}

void GoFrontEnd() {
  //Go front to the end
  if (stupidCount == 1 && FrontSensor > minimum_distance ){
    GoFront();
    checkDistance();
    delay(100);
    stupidCount++;
  }

  while(FrontSensor > minimum_distance){
    GoFront();
    checkStraight();
    checkDistance();
    delay(100);
  }

  Brake();

  CorrectDirection();

  //After reaching the front end, change direction to left or right
  if (RightSensor >= LeftSensor && RightSensor>minimum_distance){
    currentDirection = "R";
  }
  else{
    currentDirection = "L";
  }
}

void GoBackEnd() {
  //Go Back to the end
  while(BackSensor > minimum_distance){
    GoBack();
    checkStraight();
    checkDistance();
    delay(100);
  }
  
  Brake();

  CorrectDirection();

  //After reaching the back end, change direction to left or right
  if (RightSensor >= LeftSensor && RightSensor>minimum_distance){
    currentDirection = "R";
  }
  else{
    currentDirection = "L";
  }
}

void GoLeftEnd() {
  //Go Left to the end
  while(LeftSensor > minimum_distance){
    GoLeft();
    checkStraight();
    checkDistance();
    delay(100);
  }

  Brake();

  CorrectDirection();

  //After reaching the left end, change direction to front or back
  if (FrontSensor >= BackSensor && FrontSensor>minimum_distance){
    currentDirection = "F";
  }
  else{
    currentDirection = "B";
  }
}

void GoRightEnd() {
  //Go Right to the end
  while(RightSensor > minimum_distance){
    GoRight();
    checkStraight();
    checkDistance();
    delay(100);
  }

  Brake();

  CorrectDirection();

  //After reaching the right end, change direction to front or back
  if (FrontSensor >= BackSensor && FrontSensor>minimum_distance){
    currentDirection = "F";
  }
  else{
    currentDirection = "B";
  }
}

void Brake(){
  digitalWrite(LFDirection, LOW);
  digitalWrite(LFPWM, LOW);
  
  digitalWrite(LBDirection, LOW);
  digitalWrite(LBPWM, LOW);
  
  digitalWrite(RFDirection, LOW);
  digitalWrite(RFPWM, LOW);
  
  digitalWrite(RBDirection, LOW);
  digitalWrite(RBPWM, LOW);

  delay(1000);
}

void GoFront(){
Serial.print("going Front");
Serial.print("\n");

digitalWrite(LFDirection, LOW);
analogWrite(LFPWM, fullSpeedCCW);

digitalWrite(LBDirection, LOW);
analogWrite(LBPWM, fullSpeedCCW);

digitalWrite(RFDirection, HIGH);
analogWrite(RFPWM, fullSpeedCW);

digitalWrite(RBDirection, HIGH);
analogWrite(RBPWM, fullSpeedCW);

delay(100);
}

void GoBack(){
Serial.print("going Back");
Serial.print("\n");

digitalWrite(LFDirection, HIGH);
analogWrite(LFPWM, fullSpeedCW);

digitalWrite(LBDirection, HIGH);
analogWrite(LBPWM, fullSpeedCW);

digitalWrite(RFDirection, LOW);
analogWrite(RFPWM, fullSpeedCCW);

digitalWrite(RBDirection, LOW);
analogWrite(RBPWM, fullSpeedCCW);

delay(100);
}

void GoLeft(){
Serial.print("going Left");
Serial.print("\n");

digitalWrite(LFDirection, HIGH);
analogWrite(LFPWM, fullSpeedCW);

digitalWrite(LBDirection, LOW);
analogWrite(LBPWM, fullSpeedCCW);

digitalWrite(RFDirection, HIGH);
analogWrite(RFPWM, fullSpeedCW);

digitalWrite(RBDirection, LOW);
analogWrite(RBPWM, fullSpeedCCW);

delay(100);
}

void GoRight(){
Serial.print("going Right");
Serial.print("\n");

digitalWrite(LFDirection, LOW);
analogWrite(LFPWM, fullSpeedCCW);

digitalWrite(LBDirection, HIGH);
analogWrite(LBPWM, fullSpeedCW);

digitalWrite(RFDirection, LOW);
analogWrite(RFPWM, fullSpeedCCW);

digitalWrite(RBDirection, HIGH);
analogWrite(RBPWM, fullSpeedCW);

delay(100);
}

void GoFrontMinor(){
Serial.print("going Front minor");
Serial.print("\n");

digitalWrite(LFDirection, LOW);
analogWrite(LFPWM, minorSpeedCCW);

digitalWrite(LBDirection, LOW);
analogWrite(LBPWM, minorSpeedCCW);

digitalWrite(RFDirection, HIGH);
analogWrite(RFPWM, minorSpeedCW);

digitalWrite(RBDirection, HIGH);
analogWrite(RBPWM, minorSpeedCW);

delay(10);
}

void GoBackMinor(){
Serial.print("going Back minor");
Serial.print("\n");

digitalWrite(LFDirection, HIGH);
analogWrite(LFPWM, minorSpeedCW);

digitalWrite(LBDirection, HIGH);
analogWrite(LBPWM, minorSpeedCW);

digitalWrite(RFDirection, LOW);
analogWrite(RFPWM, minorSpeedCCW);

digitalWrite(RBDirection, LOW);
analogWrite(RBPWM, minorSpeedCCW);

delay(10);
}

void GoLeftMinor(){
Serial.print("going Left minor");
Serial.print("\n");

digitalWrite(LFDirection, HIGH);
analogWrite(LFPWM, minorSpeedCW);

digitalWrite(LBDirection, LOW);
analogWrite(LBPWM, minorSpeedCCW);

digitalWrite(RFDirection, HIGH);
analogWrite(RFPWM, minorSpeedCW);

digitalWrite(RBDirection, LOW);
analogWrite(RBPWM, minorSpeedCCW);

delay(10);
}

void GoRightMinor(){
Serial.print("going Right Minor");
Serial.print("\n");

digitalWrite(LFDirection, LOW);
analogWrite(LFPWM, minorSpeedCCW);

digitalWrite(LBDirection, HIGH);
analogWrite(LBPWM, minorSpeedCW);

digitalWrite(RFDirection, LOW);
analogWrite(RFPWM, minorSpeedCCW);

digitalWrite(RBDirection, HIGH);
analogWrite(RBPWM, minorSpeedCW);

delay(10);
}


void checkStraight (){
while ((currentDirection =="F"||currentDirection =="B")&& RightSensor<=side_distance){
  delay(100);
  GoLeftMinor();
  checkDistance();
  }
  
while ((currentDirection =="F"||currentDirection =="B") && LeftSensor<=side_distance){
  delay(100);
  GoRightMinor();
  checkDistance();
  } 

while ((currentDirection =="L"||currentDirection=="R") && FrontSensor<=side_distance){
  delay(100);
  GoBackMinor();
  checkDistance();
  }
  
while ((currentDirection =="L"||currentDirection=="R") && BackSensor<=side_distance){
  delay(100);
  GoFrontMinor();
  checkDistance();
  }    

//while (currentDirection == "B" && LeftSensor <= side_distance){
//  delay(100);
//  GoRightMinor();
//  checkDistance();
//  }
//
//while (currentDirection == "B" && LeftSensor >= 8){
//    delay(100);
//    GoLeftMinor();
//    checkDistance();
//  }
  
  CorrectDirection();
}


void CorrectDirection(){
  GetAngle();
  while ((CurrentAngle - RefAngle) > MinAngle){
    RotateCCWMinor();
    GetAngle();
  }

  while ((RefAngle - CurrentAngle) > MinAngle){
    RotateCWMinor();
    GetAngle();
  }
}

void RotateCCWMinor(){
  Serial.println("Rotate CCW minor");

  digitalWrite(LFDirection, HIGH);
  analogWrite(LFPWM, 220);

  digitalWrite(LBDirection, HIGH);
  analogWrite(LBPWM, 220);

  digitalWrite(RFDirection, HIGH);
  analogWrite(RFPWM, 220);

  digitalWrite(RBDirection, HIGH);
  analogWrite(RBPWM, 220);

  delay(10);
}

void RotateCWMinor(){
  Serial.println("Rotate CW minor");

  digitalWrite(LFDirection, LOW);
  analogWrite(LFPWM, 35);

  digitalWrite(LBDirection, LOW);
  analogWrite(LBPWM, 35);

  digitalWrite(RFDirection, LOW);
  analogWrite(RFPWM, 35);

  digitalWrite(RBDirection, LOW);
  analogWrite(RBPWM, 35);

  delay(10);
}

void GetAngle(){
    if (!dmpReady) return;
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
    } 
    else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//            Serial.print("ypr\t");
//            Serial.print(ypr[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(ypr[2] * 180/M_PI);
            delay(100);
        #endif
    }
    
    CurrentAngle = ypr[0] * 180/M_PI;
//    Serial.print("ypr[0]");
//    Serial.println(ypr[0]);
    Serial.print("CurrentAngle = ");
    Serial.println(CurrentAngle);
//    Serial.print("RefAngle = ");
//    Serial.println(RefAngle);
}

void GetRefAngle(){
    if (!dmpReady) return;
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
    } 
    else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//            Serial.print("ypr\t");
//            Serial.print(ypr[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(ypr[2] * 180/M_PI);
            delay(100);
        #endif
    }
    RefAngle = ypr[0] * 180/M_PI;
//    Serial.print("ypr[0] initial");
//    Serial.println(ypr[0]);
    Serial.print("RefAngle = ");
    Serial.println(RefAngle);
}
