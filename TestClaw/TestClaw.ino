#include <Servo.h>

Servo arm;
Servo leftClaw;
Servo rightClaw;

int leftClawOpenAngle;
int rightClawOpenAngle;
int leftClawClosedAngle;
int rightClawClosedAngle;

enum Claw {right, left};

int photoResistorPin = A1;

void setup() {
  leftClaw.attach(4);
  rightClaw.attach(5);
  arm.attach(6);
  Serial.begin(9600);  // start serial for output
  Serial.println("Program started.");
  Serial.println();
  //ConfigureClaws();
  //clawGrab (Claw::right);
}

void loop() {

  // Print photoresistor value
  //          PhotoR     10K
  //+5    o---/\/\/--.--/\/\/---o GND
  //                 |
  //Pin 0 o-----------
  /*
  Serial.println(analogRead(photoResistorPin * 5.0 / 1023.0));  // Analog pin read returns a 10 bit unsigned integer (from 0 to 1023) showing the voltage level of the pin (between 0V and 5V). So multiply sensorvalue by 5V/1023 to convert to voltage
  delay(10);
  */

  
  // Cause arm and claw movement/spasms
  //*
  //leftClaw.write(180);
  //rightClaw.write(180);
  arm.write(180);
  delay(500);
  //leftClaw.write(0);
  //rightClaw.write(0);
  arm.write(0);
  delay(500);
  //*/

  // Move arm and single claw
  /*
  arm.write(45);  //move arm left
  delay(1000);  //for 1 sec
  arm.write(90);  //stop moving arm left
  delay(1000);  //wait 1 sec
  // grab ball
  delay(1000);  //for 1 sec;
  arm.write(135);  //move arm right
  delay(1000);  // for 1 sec
  arm.write(90);  // stop moving arm right
  delay(5000);
  */

  // Claw grabs ball (sort of works if the servos are set up right)
  //*
  leftClaw.write(90);
  //rightClaw.write(180);
  //Serial.print("rightClaw: ");
  //Serial.println(rightClaw.read());
  
  delay(1000);
  leftClaw.write(-90);
  delay(1000);
  //rightClaw.write(-130);
  //arm.write(110);
  delay(1000);
  //*/
  
}

void rightClawGrasp () {
  clawGrasp(Claw::right);
}

void leftClawGrasp () {
  clawGrasp(Claw::left);
}

// Closes claw around ball (sort of works if the servos are set up right)
void clawGrasp (Claw claw) {
  
  // Get the correct servo
  Servo * clawServo;
  if (claw == Claw::right) {
    clawServo = &rightClaw;
  } else if (claw == Claw::left) {
    clawServo = &leftClaw;
  } else {
    Serial.println ("Error, unknown claw");
  }
  
  clawServo->write(90);
  delay(1000);
  clawServo->write(-90);
  delay(1000);
}

int armSpeed = 45;  // is this 45 degrees per second? The documentation says the servo write function takes a number from 0 to 180
int servoRPM = 0; // RPM of the servo at the "armSpeed" speed. We will have to test this by running servo at "armSpeed" for 2-5 minutes, counting the number of rotations, and dividing this by how many minutes it ran
int currentToothPosition = 0; // assume arm starts at position 0
// move the arm by a certain number of teeth (There are 17 teeth on arm, labeled -8 to 8, but the ends -8 and 8 are not reachable. So arm can only go from -6 to 6).
// Positive + values move arm right, negative - values move arm left
// There are 10 teeth on the top wheel
// returns the change in position from current position (final position - initial position)
int moveArm (int toothPosition) {
  // check if toothPosition is between -6 and 6
  if (toothPosition < -6 || toothPosition > 6) {
    Serial.print   ("Error: can't move arm to pin ");
    Serial.print   (toothPosition);
    Serial.println (" because it's out of arm's range.");
    return 0;
  }
  int distance = toothPosition - currentToothPosition;

  // Calculate amount of time to move for (distance = speed * time)
  int moveTime = distance / armSpeed;
  
  currentToothPosition += distance; // after moving, reset current tooth position
  return distance;  // return distance traveled (in teeth)
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




