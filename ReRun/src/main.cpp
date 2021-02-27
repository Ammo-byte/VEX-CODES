/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// motor5               motor         1               
// motor2               motor         11              
// motor3               motor         10              
// motor4               motor         20              
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "AutonReplay.h"
#include "robot-config.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
wait(170, msec); 
motor2.spin(forward, -17, velocityUnits::pct); 
motor3.spin(forward, -17, velocityUnits::pct); 
motor4.spin(forward, -17, velocityUnits::pct); 
motor5.spin(forward, -17, velocityUnits::pct); 
wait(50, msec); 
motor2.spin(forward, -64, velocityUnits::pct); 
motor3.spin(forward, -64, velocityUnits::pct); 
motor4.spin(forward, -64, velocityUnits::pct); 
motor5.spin(forward, -64, velocityUnits::pct); 
wait(50, msec); 
motor2.spin(forward, -100, velocityUnits::pct); 
motor3.spin(forward, -100, velocityUnits::pct); 
motor4.spin(forward, -100, velocityUnits::pct); 
motor5.spin(forward, -100, velocityUnits::pct); 
wait(250, msec); 
motor2.spin(forward, -79, velocityUnits::pct); 
motor3.spin(forward, -79, velocityUnits::pct); 
motor4.spin(forward, -79, velocityUnits::pct); 
motor5.spin(forward, -79, velocityUnits::pct); 
wait(50, msec); 
motor2.spin(forward, 0, velocityUnits::pct); 
motor3.spin(forward, 0, velocityUnits::pct); 
motor4.spin(forward, 0, velocityUnits::pct); 
motor5.spin(forward, 0, velocityUnits::pct); 
wait(570, msec); 
motor2.spin(forward, -44, velocityUnits::pct); 
motor3.spin(forward, -44, velocityUnits::pct); 
motor4.spin(forward, -44, velocityUnits::pct); 
motor5.spin(forward, -44, velocityUnits::pct); 
wait(170, msec); 
motor2.spin(forward, -20, velocityUnits::pct); 
motor3.spin(forward, -20, velocityUnits::pct); 
motor4.spin(forward, -20, velocityUnits::pct); 
motor5.spin(forward, -20, velocityUnits::pct); 
wait(50, msec); 
motor2.spin(forward, 0, velocityUnits::pct); 
motor3.spin(forward, 0, velocityUnits::pct); 
motor4.spin(forward, 0, velocityUnits::pct); 
motor5.spin(forward, 0, velocityUnits::pct); 
wait(330, msec); 
motor2.spin(forward, -99, velocityUnits::pct); 
motor3.spin(forward, -99, velocityUnits::pct); 
motor4.spin(forward, 99, velocityUnits::pct); 
motor5.spin(forward, 99, velocityUnits::pct); 
wait(170, msec); 
motor2.spin(forward, 0, velocityUnits::pct); 
motor3.spin(forward, 0, velocityUnits::pct); 
motor4.spin(forward, 0, velocityUnits::pct); 
motor5.spin(forward, 0, velocityUnits::pct); 
wait(450, msec); 
motor2.spin(forward, 58, velocityUnits::pct); 
motor3.spin(forward, 58, velocityUnits::pct); 
motor4.spin(forward, -58, velocityUnits::pct); 
motor5.spin(forward, -58, velocityUnits::pct); 
wait(50, msec); 
motor2.spin(forward, 100, velocityUnits::pct); 
motor3.spin(forward, 100, velocityUnits::pct); 
motor4.spin(forward, -100, velocityUnits::pct); 
motor5.spin(forward, -100, velocityUnits::pct); 
wait(90, msec); 
motor2.spin(forward, 0, velocityUnits::pct); 
motor3.spin(forward, 0, velocityUnits::pct); 
motor4.spin(forward, 0, velocityUnits::pct); 
motor5.spin(forward, 0, velocityUnits::pct); 
wait(450, msec); 
motor2.spin(forward, 100, velocityUnits::pct); 
motor3.spin(forward, 100, velocityUnits::pct); 
motor4.spin(forward, -100, velocityUnits::pct); 
motor5.spin(forward, -100, velocityUnits::pct); 
wait(170, msec); 
motor2.spin(forward, 0, velocityUnits::pct); 
motor3.spin(forward, 0, velocityUnits::pct); 
motor4.spin(forward, 0, velocityUnits::pct); 
motor5.spin(forward, 0, velocityUnits::pct); 
wait(250, msec); 
motor2.spin(forward, -65, velocityUnits::pct); 
motor3.spin(forward, -65, velocityUnits::pct); 
motor4.spin(forward, -65, velocityUnits::pct); 
motor5.spin(forward, -65, velocityUnits::pct); 
wait(50, msec); 
motor2.spin(forward, -92, velocityUnits::pct); 
motor3.spin(forward, -92, velocityUnits::pct); 
motor4.spin(forward, -92, velocityUnits::pct); 
motor5.spin(forward, -92, velocityUnits::pct); 
wait(210, msec); 
motor2.spin(forward, -62, velocityUnits::pct); 
motor3.spin(forward, -62, velocityUnits::pct); 
motor4.spin(forward, -62, velocityUnits::pct); 
motor5.spin(forward, -62, velocityUnits::pct); 
wait(50, msec); 
motor2.spin(forward, 0, velocityUnits::pct); 
motor3.spin(forward, 0, velocityUnits::pct); 
motor4.spin(forward, 0, velocityUnits::pct); 
motor5.spin(forward, 0, velocityUnits::pct); 
wait(410, msec); 
motor2.spin(forward, -100, velocityUnits::pct); 
motor3.spin(forward, -100, velocityUnits::pct); 
motor4.spin(forward, 100, velocityUnits::pct); 
motor5.spin(forward, 100, velocityUnits::pct); 
wait(90, msec); 
motor2.spin(forward, 18, velocityUnits::pct); 
motor3.spin(forward, 18, velocityUnits::pct); 
motor4.spin(forward, -18, velocityUnits::pct); 
motor5.spin(forward, -18, velocityUnits::pct); 
wait(50, msec); 
motor2.spin(forward, 0, velocityUnits::pct); 
motor3.spin(forward, 0, velocityUnits::pct); 
motor4.spin(forward, 0, velocityUnits::pct); 
motor5.spin(forward, 0, velocityUnits::pct); 
wait(690, msec); 
motor2.spin(forward, -30, velocityUnits::pct); 
motor3.spin(forward, -30, velocityUnits::pct); 
motor4.spin(forward, -30, velocityUnits::pct); 
motor5.spin(forward, -30, velocityUnits::pct); 
wait(170, msec); 
motor2.spin(forward, 0, velocityUnits::pct); 
motor3.spin(forward, 0, velocityUnits::pct); 
motor4.spin(forward, 0, velocityUnits::pct); 
motor5.spin(forward, 0, velocityUnits::pct); 
wait(410, msec); 
motor2.spin(forward, 26, velocityUnits::pct); 
motor3.spin(forward, 26, velocityUnits::pct); 
motor4.spin(forward, -26, velocityUnits::pct); 
motor5.spin(forward, -26, velocityUnits::pct); 
wait(50, msec); 
motor2.spin(forward, 85, velocityUnits::pct); 
motor3.spin(forward, 85, velocityUnits::pct); 
motor4.spin(forward, -85, velocityUnits::pct); 
motor5.spin(forward, -85, velocityUnits::pct); 
wait(90, msec); 
motor2.spin(forward, 0, velocityUnits::pct); 
motor3.spin(forward, 0, velocityUnits::pct); 
motor4.spin(forward, 0, velocityUnits::pct); 
motor5.spin(forward, 0, velocityUnits::pct); 
wait(410, msec); 
motor2.spin(forward, -22, velocityUnits::pct); 
motor3.spin(forward, -22, velocityUnits::pct); 
motor4.spin(forward, -22, velocityUnits::pct); 
motor5.spin(forward, -22, velocityUnits::pct); 
wait(130, msec); 
motor2.spin(forward, 0, velocityUnits::pct); 
motor3.spin(forward, 0, velocityUnits::pct); 
motor4.spin(forward, 0, velocityUnits::pct); 
motor5.spin(forward, 0, velocityUnits::pct); 
wait(370, msec); 
motor2.spin(forward, -55, velocityUnits::pct); 
motor3.spin(forward, -55, velocityUnits::pct); 
motor4.spin(forward, -55, velocityUnits::pct); 
motor5.spin(forward, -55, velocityUnits::pct); 
wait(50, msec); 
motor2.spin(forward, -86, velocityUnits::pct); 
motor3.spin(forward, -86, velocityUnits::pct); 
motor4.spin(forward, -86, velocityUnits::pct); 
motor5.spin(forward, -86, velocityUnits::pct); 
wait(130, msec); 
motor2.spin(forward, -54, velocityUnits::pct); 
motor3.spin(forward, -54, velocityUnits::pct); 
motor4.spin(forward, -54, velocityUnits::pct); 
motor5.spin(forward, -54, velocityUnits::pct); 
wait(50, msec); 
motor2.spin(forward, 0, velocityUnits::pct); 
motor3.spin(forward, 0, velocityUnits::pct); 
motor4.spin(forward, 0, velocityUnits::pct); 
motor5.spin(forward, 0, velocityUnits::pct); 
wait(1450, msec); 
motor2.spin(forward, 16, velocityUnits::pct); 
motor3.spin(forward, 16, velocityUnits::pct); 
motor4.spin(forward, 16, velocityUnits::pct); 
motor5.spin(forward, 16, velocityUnits::pct); 
wait(90, msec); 
motor2.spin(forward, 34, velocityUnits::pct); 
motor3.spin(forward, 34, velocityUnits::pct); 
motor4.spin(forward, 34, velocityUnits::pct); 
motor5.spin(forward, 34, velocityUnits::pct); 
wait(1090, msec); 
motor2.spin(forward, 14, velocityUnits::pct); 
motor3.spin(forward, 14, velocityUnits::pct); 
motor4.spin(forward, 60, velocityUnits::pct); 
motor5.spin(forward, 60, velocityUnits::pct); 
wait(130, msec); 
motor2.spin(forward, 36, velocityUnits::pct); 
motor3.spin(forward, 36, velocityUnits::pct); 
motor4.spin(forward, 36, velocityUnits::pct); 
motor5.spin(forward, 36, velocityUnits::pct); 
wait(1850, msec); 
motor2.spin(forward, 62, velocityUnits::pct); 
motor3.spin(forward, 62, velocityUnits::pct); 
wait(50, msec); 
motor2.spin(forward, 99, velocityUnits::pct); 
motor3.spin(forward, 99, velocityUnits::pct); 
motor4.spin(forward, 0, velocityUnits::pct); 
motor5.spin(forward, 0, velocityUnits::pct); 
wait(130, msec); 
motor2.spin(forward, 51, velocityUnits::pct); 
motor3.spin(forward, 51, velocityUnits::pct); 
motor4.spin(forward, 51, velocityUnits::pct); 
motor5.spin(forward, 51, velocityUnits::pct); 
wait(1210, msec); 
motor2.spin(forward, 0, velocityUnits::pct); 
motor3.spin(forward, 0, velocityUnits::pct); 
motor4.spin(forward, 0, velocityUnits::pct); 
motor5.spin(forward, 0, velocityUnits::pct); 
wait(690, msec); 
motor2.spin(forward, -22, velocityUnits::pct); 
motor3.spin(forward, -22, velocityUnits::pct); 
motor4.spin(forward, -22, velocityUnits::pct); 
motor5.spin(forward, -22, velocityUnits::pct); 
wait(1010, msec); 
motor2.spin(forward, 0, velocityUnits::pct); 
motor3.spin(forward, 0, velocityUnits::pct); 
motor4.spin(forward, -60, velocityUnits::pct); 
motor5.spin(forward, -60, velocityUnits::pct); 
wait(170, msec); 
motor2.spin(forward, 16, velocityUnits::pct); 
motor3.spin(forward, 16, velocityUnits::pct); 
wait(210, msec); 
motor2.spin(forward, -28, velocityUnits::pct); 
motor3.spin(forward, -28, velocityUnits::pct); 
motor4.spin(forward, -30, velocityUnits::pct); 
motor5.spin(forward, -30, velocityUnits::pct); 
wait(250, msec); 
motor2.spin(forward, -73, velocityUnits::pct); 
motor3.spin(forward, -73, velocityUnits::pct); 
motor4.spin(forward, 15, velocityUnits::pct); 
motor5.spin(forward, 15, velocityUnits::pct); 
wait(50, msec); 
motor2.spin(forward, -100, velocityUnits::pct); 
motor3.spin(forward, -100, velocityUnits::pct); 
motor4.spin(forward, 62, velocityUnits::pct); 
motor5.spin(forward, 62, velocityUnits::pct); 
wait(290, msec); 
motor4.spin(forward, 35, velocityUnits::pct); 
motor5.spin(forward, 35, velocityUnits::pct); 
wait(50, msec); 
motor2.spin(forward, -29, velocityUnits::pct); 
motor3.spin(forward, -29, velocityUnits::pct); 
motor4.spin(forward, -29, velocityUnits::pct); 
motor5.spin(forward, -29, velocityUnits::pct); 
wait(490, msec); 
motor2.spin(forward, -71, velocityUnits::pct); 
motor3.spin(forward, -71, velocityUnits::pct); 
motor4.spin(forward, -71, velocityUnits::pct); 
motor5.spin(forward, -71, velocityUnits::pct); 
wait(90, msec); 
motor2.spin(forward, -100, velocityUnits::pct); 
motor3.spin(forward, -100, velocityUnits::pct); 
motor4.spin(forward, -100, velocityUnits::pct); 
motor5.spin(forward, -100, velocityUnits::pct); 
wait(250, msec); 
motor2.spin(forward, -34, velocityUnits::pct); 
motor3.spin(forward, -34, velocityUnits::pct); 
motor4.spin(forward, -34, velocityUnits::pct); 
motor5.spin(forward, -34, velocityUnits::pct); 
wait(50, msec); 
motor2.spin(forward, 0, velocityUnits::pct); 
motor3.spin(forward, 0, velocityUnits::pct); 
wait(50, msec); 
motor2.spin(forward, 80, velocityUnits::pct); 
motor3.spin(forward, 80, velocityUnits::pct); 
motor4.spin(forward, -100, velocityUnits::pct); 
motor5.spin(forward, -100, velocityUnits::pct); 
wait(170, msec); 
motor2.spin(forward, 0, velocityUnits::pct); 
motor3.spin(forward, 0, velocityUnits::pct); 
motor4.spin(forward, 0, velocityUnits::pct); 
motor5.spin(forward, 0, velocityUnits::pct); 
wait(410, msec); 
motor2.spin(forward, -26, velocityUnits::pct); 
motor3.spin(forward, -26, velocityUnits::pct); 
motor4.spin(forward, -26, velocityUnits::pct); 
motor5.spin(forward, -26, velocityUnits::pct); 
wait(530, msec); 
motor2.spin(forward, 0, velocityUnits::pct); 
motor3.spin(forward, 0, velocityUnits::pct); 
motor4.spin(forward, -80, velocityUnits::pct); 
motor5.spin(forward, -80, velocityUnits::pct); 
wait(50, msec); 
motor2.spin(forward, 34, velocityUnits::pct); 
motor3.spin(forward, 34, velocityUnits::pct); 
motor4.spin(forward, -100, velocityUnits::pct); 
motor5.spin(forward, -100, velocityUnits::pct); 
wait(90, msec); 
motor2.spin(forward, -39, velocityUnits::pct); 
motor3.spin(forward, -39, velocityUnits::pct); 
motor4.spin(forward, -39, velocityUnits::pct); 
motor5.spin(forward, -39, velocityUnits::pct); 
wait(330, msec); 
motor2.spin(forward, -100, velocityUnits::pct); 
motor3.spin(forward, -100, velocityUnits::pct); 
motor4.spin(forward, 45, velocityUnits::pct); 
motor5.spin(forward, 45, velocityUnits::pct); 
wait(50, msec); 
motor4.spin(forward, 63, velocityUnits::pct); 
motor5.spin(forward, 63, velocityUnits::pct); 
wait(210, msec); 
motor2.spin(forward, -39, velocityUnits::pct); 
motor3.spin(forward, -39, velocityUnits::pct); 
motor4.spin(forward, -39, velocityUnits::pct); 
motor5.spin(forward, -39, velocityUnits::pct); 
wait(2410, msec); 
motor2.spin(forward, 0, velocityUnits::pct); 
motor3.spin(forward, 0, velocityUnits::pct); 
motor4.spin(forward, 0, velocityUnits::pct); 
motor5.spin(forward, 0, velocityUnits::pct); 
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

#define MAX_POWER 100
#define DEADBAND 10

int sgn(float num) {
  if(num > 0) {                                                                 
    return(1);                                                             
  } else if(num < 0) {
    return(-1);
  } else {
    return(0);
  }
}

void usercontrol(void) {

  int leftpower;
  int rightpower;

  while(true) {
    AR_BEGINSegment(); //Does the correct initialization at the start of the loop

    leftpower = (Controller1.Axis3.position() + Controller1.Axis1.position());
    rightpower = (Controller1.Axis3.position() - Controller1.Axis1.position());

    if(abs(rightpower) > MAX_POWER) {
      rightpower = sgn(rightpower) * MAX_POWER;
    }

    if(abs(leftpower) > MAX_POWER) {
      leftpower = sgn(leftpower) * MAX_POWER;
    }

    if(abs(leftpower) < DEADBAND) leftpower = 0;
    if(abs(rightpower) < DEADBAND) rightpower = 0;

    //SET MOTORS TO RIGHT POWER AND LEFT POWER HERE
    // motor[motor1]=leftpower;
    motor5.spin(forward, leftpower, velocityUnits::pct);
		AR_GenerateMotorPower(1,leftpower); // Adds motor power to queue
		// motor[motor2] = leftpower;
    motor2.spin(forward, leftpower, velocityUnits::pct);
		AR_GenerateMotorPower(2,leftpower);
		// motor[motor3] = rightpower;
    motor3.spin(forward, rightpower, velocityUnits::pct);
		AR_GenerateMotorPower(3,rightpower);
		// motor[motor4] = rightpower;
    motor4.spin(forward, rightpower, velocityUnits::pct);
		AR_GenerateMotorPower(4, rightpower);

		AR_ENDSegment(); // Calculations at end of the loop

  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
