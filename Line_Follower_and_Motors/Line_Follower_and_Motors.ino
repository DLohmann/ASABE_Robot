/******************************************************************************
MostBasicFollower.ino

A very simple method for following a line with a redbot and the line follower array.

Marshall Taylor, SparkFun Engineering

5-27-2015

Library:
https://github.com/sparkfun/SparkFun_Line_Follower_Array_Arduino_Library
Product:
https://github.com/sparkfun/Line_Follower_Array

This example demonstrates the easiest way to interface the redbot sensor bar.
  "SensorBar mySensorBar(SX1509_ADDRESS);" creates the sensor bar object.
  "mySensorBar.init();" gets the bar ready.
  "mySensorBar.getDensity()" gets the number of points sensed.
  "mySensorBar.getPosition()" gets the average center of sensed points.

The loop has three main points of operation.
  1.  check if the density is reasonable
  2.  get the position
  3.  choose a drive mode based on position

Note:
The wheel direction polarity can be switched with RIGHT/LEFT_WHEEL_POL (or by flipping the wires)
  #define RIGHT_WHEEL_POL 1
  #define LEFT_WHEEL_POL 1
To check, hold the bot centered over a line so that the two middle sensors detect
  then observe.  Does the bot try to drive forward?
  
Resources:
sensorbar.h

Development environment specifics:
arduino > v1.6.4
hw v1.0

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/


#include "Servo.h"
#include "Wire.h"
#include "sensorbar.h"
//#include <Servo.h>
//#include <Wire.h>


// Uncomment one of the four lines to match your SX1509's address
//  pin selects. SX1509 breakout defaults to [0:0] (0x3E).
const uint8_t SX1509_ADDRESS = 0x3E;  // SX1509 I2C address (00)
//const byte SX1509_ADDRESS = 0x3F;  // SX1509 I2C address (01)
//const byte SX1509_ADDRESS = 0x70;  // SX1509 I2C address (10)
//const byte SX1509_ADDRESS = 0x71;  // SX1509 I2C address (11)

SensorBar mySensorBar(SX1509_ADDRESS);
Servo servoleft;
Servo servoright;


//#define printMovements    //Boolean for printing motor actions (forward, right, left, back, stop_servos). defined for printing, undefined for not printing
//#define printState          //Boolean for printing current robot state (Idle, Read_Line, Forward, Left, Right). defined for printing, undefined for not printing
#define printSensorBar      //Boolean for printing current sensorbar values (mySensorBar.getPosition(), mySensorBar.getDensity()). defined for printing, undefined for not printing

//Define the states that the decision making machines uses:
#define Idle 0
#define Read_Line 1
#define Forward 2
#define Left 3
#define Right 4

uint8_t state;


void setup()
{
  servoleft.attach(2);
  servoright.attach(3);
  Serial.begin(9600);  // start serial for output
  Serial.println("Program started.");
  Serial.println();
  
  //Default: the IR will only be turned on during reads.
  mySensorBar.setBarStrobe();
  //Other option: Command to run all the time
  //mySensorBar.clearBarStrobe();

  //Default: dark on light
  mySensorBar.clearInvertBits();
  //Other option: light line on dark
  //mySensorBar.setInvertBits();
  
  //Don't forget to call .begin() to get the bar ready.  This configures HW.
  uint8_t returnStatus = mySensorBar.begin();
  if(returnStatus)
  {
   Serial.println("sx1509 IC communication OK");
  }
  else
  {
    Serial.println("sx1509 IC communication FAILED!");
  }
  Serial.println();

  #ifdef printState
    Serial.print ("Initial state: ");
    printStateString();
    Serial.println();
  #endif
}

void loop()
{
uint8_t nextState = state;

#ifdef printState
  printStateString();
  Serial.println();
#endif

switch (state){
case Idle:
  stop_servos();
  nextState= Read_Line;
  break;
  
case Read_Line:
 if( mySensorBar.getDensity() < 7 )
    {
      nextState = Forward;
      if( mySensorBar.getPosition() < -50 )
      {
        nextState = Left;
      }
      if( mySensorBar.getPosition() > 50 )
      {
        nextState = Right;
      }
    }
    else
    {
      nextState = Idle;
    }
    break;
    
case Forward:
  forward();
  nextState = Read_Line;
  break;

case Left:
  left();
  break;

case Right:
  right();
  break;
  
default:
stop_servos();
break;
}

 state = nextState; 

}
void forward(){
  servoleft.write(0);
  servoright.write(0);

  #ifdef printMovements
    Serial.println("MOVE forward");
  #endif
}

void left() {
  servoleft.write(0);
  servoright.write(0);

  #ifdef printMovements
    Serial.println("MOVE left");
  #endif
}

void back() {
  servoleft.write(0);
  servoright.write(180);
  
  #ifdef printMovements
    Serial.println("MOVE back");
  #endif
}

void right() {
  servoleft.write(180);
  servoright.write(180);
  
  #ifdef printMovements
  Serial.println("MOVE right");
  #endif
}


void stop_servos(){
  servoleft.write(90);
  servoright.write(90);
  
 #ifdef printMovements
  Serial.println("MOVE stop_servos");
 #endif
}

void printStateString () {
  switch (state) {
    case Idle:
      Serial.print ("STATE Idle");
      break;
    case Read_Line:
      Serial.print ("STATE Read_Line");
      break;
    case Forward:
      Serial.print ("STATE Forward");
      break;
    case Left:
      Serial.print ("STATE Left");
      break;
    case Right:
      Serial.print ("STATE Right");
      break;
    default:
      Serial.print ("STATE not found");
  }

}


void printSensorBarVals () {
  Serial.print ("SENSORBAR Position: ");
  Serial.print (mySensorBar.getPosition());
  Serial.print (", Density: ");
  Serial.println (mySensorBar.getDensity());
}
