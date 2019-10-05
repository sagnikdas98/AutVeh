// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

#include <AFMotor.h>
#define LR 4
#define LF 3
#define RF 2
#define RR 1

// DC motors
AF_DCMotor motorLR(LR);
AF_DCMotor motorLF(LF);
AF_DCMotor motorRF(RF);
AF_DCMotor motorRR(RR);

void setup() {
  
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor party!");

  // turn on motors
  motorLR.setSpeed(200);
  motorLF.setSpeed(200);
  motorRF.setSpeed(200);
  motorRR.setSpeed(200);
  motorLR.run(RELEASE);
  motorLF.run(RELEASE);
  motorRF.run(RELEASE);
  motorRR.run(RELEASE);
}

int i;

void loop() {
  
  motorLR.run(FORWARD);
  motorLF.run(FORWARD);
  motorRF.run(FORWARD);
  motorRR.run(FORWARD);
  for (i=0; i<255; i++) {
  motorLR.setSpeed(i);
  motorLF.setSpeed(i);
  motorRF.setSpeed(i);
  motorRR.setSpeed(i);  
    delay(3);
 }
//  for (i=255; i!=0; i--) {
//    motor.setSpeed(i);  
//    delay(3);
// }
// 
//motor.run(BACKWARD);
//  for (i=0; i<255; i++) {
// 
//    motor.setSpeed(i);  
//    delay(3);
//   
// }
// 
//  for (i=255; i!=0; i--) {
//
//    motor.setSpeed(i);  
//    delay(3);
// }
}


