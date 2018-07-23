#include <Servo.h>

Servo arm;
Servo leftClaw;
Servo rightClaw;

int leftClawOpenAngle;
int rightClawOpenAngle;
int leftClawClosedAngle;
int rightClawClosedAngle;



void setup() {
  leftClaw.attach(4);
  rightClaw.attach(5);
  arm.attach(6);
  Serial.begin(9600);  // start serial for output
  Serial.println("Program started.");
  Serial.println();
  ConfigureClaws();
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


/*
openClaw () {
  
}

closeClaw (Servo s) {
  if (s.read() < )
}
*/


