#include <Servo.h>

Servo arm;
Servo leftClaw;
Servo rightClaw;

int leftClawOpenAngle;
int rightClawOpenAngle;
int leftClawClosedAngle;
int rightClawClosedAngle;

enum Claw {right, left};

void setup() {
  leftClaw.attach(4);
  rightClaw.attach(5);
  arm.attach(6);
  Serial.begin(9600);  // start serial for output
  Serial.println("Program started.");
  Serial.println();
  //ConfigureClaws();
  clawGrab (Claw::right);
}

void loop() {
  /*
  // Cause arm and claw movement
  leftClaw.write(180);
  rightClaw.write(180);
  arm.write(180);
  delay(500);
  leftClaw.write(0);
  rightClaw.write(0);
  arm.write(0);
  delay(500);
  */

  // Move arm and single claw
  /*
  //leftClaw.write(180);
  //rightClaw.write(180);
  Serial.print("rightClaw: ");
  Serial.println(rightClaw.read());
  //arm.write(180);
  delay(500);
  //leftClaw.write(0);
  //rightClaw.write(0);
  //arm.write(0);
  //delay(500);
  */
  
}

// Set up both claws
void ConfigureClaws () {
  // Opens both claws to the max, and read what angle they are at
  leftClaw.write(180);
  leftClawOpenAngle = leftClaw.read();
  rightClaw.write(180);
  rightClawOpenAngle = rightClaw.read();

  // Close both claws to the max, and read what angle they are at
  leftClaw.write(0);
  leftClawClosedAngle = leftClaw.read();
  rightClaw.write(0);
  rightClawClosedAngle = rightClaw.read();

  // print left and right claw's max open angle
  Serial.print ("Left Claw Open Max Servo: ");
  Serial.print (leftClawOpenAngle);
  Serial.print (", Right Claw Open Max Servo: ");
  Serial.println (rightClawOpenAngle);

  delay(1000); // wait 1 sec (1000 ms)
  
  // print left and right claw's max open angle
  Serial.print ("Left Claw Closed Min Servo: ");
  Serial.print (leftClawClosedAngle);
  Serial.print (", Right Claw Closed Min Servo: ");
  Serial.println (rightClawClosedAngle);
}

// Motion to Grab Ball
// ASSUMES CLAW is INITIALLY OPEN, and assuming arm is INITIALLY in MIDDLE
void clawGrab (Claw claw) {
  
  // Get the correct servo
  Servo * clawServo;
  if (claw == Claw::right) {
    clawServo = &rightClaw;
  } else if (claw == Claw::left) {
    clawServo = &leftClaw;
  } else {
    Serial.println ("Error, unknown claw");
  }
  
  //get initial claw and arm values, to reset them later
  int initialClaw = clawServo->read();
  int initialArm = arm.read();

  // Move arm to side
  if (claw == Claw::right) {
    // move arm right, and right claw to ball
    arm.write(135);// right direction????????????????????????????????????????????????????????????????
    delay(250);
    arm.write(90);
    Serial.println ("Moving arm right");
  } else if (claw == Claw::left) {
    // move arm left, and left claw to ball
    arm.write(45);// left direction????????????????????????????????????????????????????????????????
    delay(250);
    arm.write(90);
    Serial.println ("Moving arm left");
  } else {
    Serial.println ("Error: \"clawGrab\" called on a servo that is not a claw!");
  }

  // Close claw around ball
  clawServo->write(0);

  // RESET CLAW to OPEN and ARM to MIDDLE
  arm.write(initialArm);  // Reset arm to middle BEFORE resetting claw to open
  delay(500);
  arm.write(90);
  clawServo->write(initialClaw);  // So that claw is over basket before opening, and dropping ball in basket
}

/*
int openClaw (Servo claw) {
  
}
*/

/*
closeClaw (Servo s) {
  
}
*/




