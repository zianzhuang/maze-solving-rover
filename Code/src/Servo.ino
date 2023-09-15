// Include the Servo library
#include <Servo.h> 

// Declare the Servo pin
int servoPin1 = 6;
int servoPin2 = 5;
// Create a servo object
Servo Servo1;
Servo Servo2;

void setup() {
  // We need to attach the servo to the used pin number
  Servo1.attach(servoPin1);
  Servo2.attach(servoPin2);
  
}

void loop(){
  liftG();
  dropG();
  
}

void openG(){
  Servo1.write(160);
}

void closeG(){
  Servo1.write(100);
}

void liftG(){
  Servo2.write(90);
}

void dropG(){
  Servo2.write(0);
}
