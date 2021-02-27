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
// pushOut              motor         12              
// leftFront            motor         20              
// leftBack             motor         18              
// rightFront           motor         7               
// rightBack            motor         10              
// lift                 motor         3               
// intakeLeft           motor         13              
// intakeRight          motor         6               
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "math.h"
#include "algorithm"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perightFrontorm some actions before the competition starts.      */
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

//A minimum velocity is necessary. This could also be done per command instead
//of globally.
double minimum_velocity = 15.0;

//This function provides an increasing speed as the robot moves away from start
double mincreasing_speed (double starting_point, double current_position) {
    static const double acceleration_constant = 45.0;
    return acceleration_constant * std::abs(current_position - starting_point) + minimum_velocity;
}

//This function provides a decreasing speed as the robot approaches the end
double mdecreasing_speed (double ending_point, double current_position) {
    static const double deceleration_constant = 45.0;
    return deceleration_constant * std::abs(ending_point - current_position) + minimum_velocity;
}

double tincreasing_speed (double starting_point, double current_position) 
{
    static const double tacceleration_constant = 100.0;
    return tacceleration_constant * std::abs(current_position - starting_point) + minimum_velocity;
}

double tdecreasing_speed (double ending_point, double current_position) 
{
    static const double tdeceleration_constant = 100.0;
    return tdeceleration_constant * std::abs(ending_point - current_position) + minimum_velocity;
}

double pincreasing_speed (double starting_point, double current_position) 
{
    static const double acceleration_constant = 100.0;
    return acceleration_constant * std::abs(current_position - starting_point) + minimum_velocity;
}

double pdecreasing_speed (double ending_point, double current_position) 
{
    static const double deceleration_constant = 100.0;
    return deceleration_constant * std::abs(ending_point - current_position) + minimum_velocity;
}

//This function takes a distance, a maximum velocity, and tries to send the
//robot in a straight line for that distance using a trapezoidal motion profile
//controlled by increasing_speed, decreasing_speed, and maxVelocity
void move (double distanceIn, double maxVelocity) {
    //record nominal wheel circumference
    static const double circumference = 3.14159 * 4;
    
    //if we've got a joker on our hands, punch out
    if (distanceIn == 0) return;
    
    //figure out which direction we're supposed to be going
    double direction = distanceIn > 0 ? 1.0 : -1.0;
    
    //using circumference and commanded inches, convert to revolutions
    double wheelRevs = distanceIn / circumference;
    
    //set the motors to do a position move with 0 velocity
     rightFront.spin(directionType::fwd,direction * minimum_velocity,velocityUnits::pct);
     leftFront.spin(directionType::fwd,direction * minimum_velocity,velocityUnits::pct);
     rightBack.spin(directionType::fwd,direction * minimum_velocity,velocityUnits::pct);
     leftBack.spin(directionType::fwd,direction * minimum_velocity,velocityUnits::pct);
    
    //record starting positions and ending positions
    double leftStartPoint = leftFront.rotation(rotationUnits::rev);
    double leftEndPoint = leftStartPoint + wheelRevs;
    double rightStartPoint = rightFront.rotation(rotationUnits::rev);
    double rightEndPoint = rightStartPoint + wheelRevs;
    
    //Back Motors
    double leftBStartPoint = leftBack.rotation(rotationUnits::rev);
    double leftBEndPoint = leftBStartPoint + wheelRevs;
    double rightBStartPoint = rightBack.rotation(rotationUnits::rev);
    double rightBEndPoint = rightBStartPoint + wheelRevs;
    
    //execute motion profile
    while (
        (direction * (rightFront.rotation(rotationUnits::rev) - rightStartPoint) < direction * wheelRevs) ||
        (direction * (leftFront.rotation(rotationUnits::rev) - leftStartPoint) < direction * wheelRevs)  ||
        (direction * (leftBack.rotation(rotationUnits::rev) - leftBStartPoint) < direction * wheelRevs)  ||
        (direction * (rightBack.rotation(rotationUnits::rev) - rightBStartPoint) < direction * wheelRevs)  
    ) {
        
        //set right motor speed to minimum of increasing function, decreasing
        //function, and max velocity, based on current position
        if (direction * (rightFront.rotation(rotationUnits::rev) - rightStartPoint) < direction * wheelRevs) {
            rightFront.setVelocity(
                direction * std::min(
                    maxVelocity,
                    std::min(
                        mincreasing_speed(rightStartPoint,rightFront.rotation(rotationUnits::rev)),
                        mdecreasing_speed(rightEndPoint,rightFront.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } else {
            rightFront.stop(brakeType::brake);
        }
        
        //do the same for the left motor
        if (direction * (leftFront.rotation(rotationUnits::rev) - leftStartPoint) < direction * wheelRevs) {
            leftFront.setVelocity(
                direction * std::min(
                    maxVelocity,
                    std::min(
                        mincreasing_speed(leftStartPoint,leftFront.rotation(rotationUnits::rev)),
                        mdecreasing_speed(leftEndPoint,leftFront.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } else {
            leftFront.stop(brakeType::brake);
        }
        
        if (direction * (leftBack.rotation(rotationUnits::rev) - leftBStartPoint) < direction * wheelRevs) {
            leftBack.setVelocity(
                direction * std::min(
                    maxVelocity,
                    std::min(
                        mincreasing_speed(leftBStartPoint,leftBack.rotation(rotationUnits::rev)),
                        mdecreasing_speed(leftBEndPoint,leftBack.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } else {
            leftBack.stop(brakeType::brake);
        }
        
        if (direction * (rightBack.rotation(rotationUnits::rev) - rightBStartPoint) < direction * wheelRevs) {
            rightBack.setVelocity(
                direction * std::min(
                    maxVelocity,
                    std::min(
                        mincreasing_speed(rightBStartPoint,rightBack.rotation(rotationUnits::rev)),
                        mdecreasing_speed(rightBEndPoint,rightBack.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } else {
            rightBack.stop(brakeType::brake);
        }
    }
}

void move (double distanceIn) {
    //no max velocity specified, call the version that uses it with max velocity
    //of 100%
    move(distanceIn, 100.0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void pointturn (double distanceIn, double maxVelocity, double sideval) 
{
    static const double circumference = 360;
    double direction4 = distanceIn > 0 ? 1.0 : -1.0;
    double wheelRevs4 = distanceIn / circumference;
    
    if (sideval == 1)
    {
        leftFront.stop(hold);
        leftBack.stop(hold);
        rightFront.spin(forward,direction4 * minimum_velocity,pct);
        rightBack.spin(forward,direction4 * minimum_velocity,pct);

        double rightStartPoint = rightFront.rotation(rev);
        double rightEndPoint = rightStartPoint + wheelRevs4;
        double rightBStartPoint = rightBack.rotation(rev);
        double rightBEndPoint = rightBStartPoint + wheelRevs4;

        while (
                (direction4 * (rightFront.rotation(rev) - rightStartPoint) < direction4 * wheelRevs4) ||
                (direction4 * (rightBack.rotation(rev) - rightBStartPoint) < direction4 * wheelRevs4)  
              ) 
        {
            if (direction4 * (rightFront.rotation(rev) - rightStartPoint) < direction4 * wheelRevs4) 
            {
                rightFront.setVelocity(
                    direction4 * std::min(
                        maxVelocity,
                        std::min(
                            pincreasing_speed(rightStartPoint,rightFront.rotation(rev)),
                            pdecreasing_speed(rightEndPoint,rightFront.rotation(rev))
                        )
                    ),
                    pct
                );
            } 
            else 
            {
                rightFront.stop(brake);
            }

            if (direction4 * (rightBack.rotation(rev) - rightBStartPoint) < direction4 * wheelRevs4) {
                rightBack.setVelocity(
                    direction4 * std::min(
                        maxVelocity,
                        std::min(
                            pincreasing_speed(rightBStartPoint,rightBack.rotation(rev)),
                            pdecreasing_speed(rightBEndPoint,rightBack.rotation(rev))
                        )
                    ),
                    pct
                );
            } 
            else 
            {
                rightBack.stop(brake);
            }     
        }
        rightFront.stop(brake);
        rightBack.stop(brake);
    }
    
    if (sideval == -1)
    {
        rightFront.stop(hold);
        rightBack.stop(hold);
        leftFront.spin(forward,direction4 * minimum_velocity,pct);
        leftBack.spin(forward,direction4 * minimum_velocity,pct);

        double leftStartPoint = leftFront.rotation(rev);
        double leftEndPoint = leftStartPoint + wheelRevs4;

        double leftBStartPoint = leftBack.rotation(rev);
        double leftBEndPoint = leftBStartPoint + wheelRevs4;

        while (
                (direction4 * (leftFront.rotation(rev) - leftStartPoint) < direction4 * wheelRevs4)  ||
                (direction4 * (leftBack.rotation(rev) - leftBStartPoint) < direction4 * wheelRevs4)
              ) 
        {
            if (direction4 * (leftFront.rotation(rev) - leftStartPoint) < direction4 * wheelRevs4) 
            {
                leftFront.setVelocity(
                    direction4 * std::min(
                        maxVelocity,
                        std::min(
                            pincreasing_speed(leftStartPoint,leftFront.rotation(rev)),
                            pdecreasing_speed(leftEndPoint,leftFront.rotation(rev))
                        )
                    ),
                    pct
                );
            } 
            else 
            {
                leftFront.stop(brake);
            }

            if (direction4 * (leftBack.rotation(rev) - leftBStartPoint) < direction4 * wheelRevs4) {
                leftBack.setVelocity(
                    direction4 * std::min(
                        maxVelocity,
                        std::min(
                            pincreasing_speed(leftBStartPoint,leftBack.rotation(rev)),
                            pdecreasing_speed(leftBEndPoint,leftBack.rotation(rev))
                        )
                    ),
                    pct
                );
            } 
            else 
            {
                leftBack.stop(brake);
            }

        }
        leftFront.stop(brake);
        leftBack.stop(brake);
    }
    leftFront.stop(brake);
    leftFront.stop(brake);
    rightFront.stop(brake);
    rightBack.stop(brake);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void turn (double distanceIn, double maxVelocity) 
{
    static const double circumference = 360;
    double direction2 = distanceIn > 0 ? -1 : 1;
    double direction3 = distanceIn > 0 ? 1 : -1;
    double wheelRevs2 = (direction2*std::abs(distanceIn)) / circumference;
    double wheelRevs3 = (direction3*std::abs(distanceIn)) / circumference;
    
    rightFront.spin(forward,direction2 * minimum_velocity,pct);
    leftFront.spin(forward,direction3 * minimum_velocity,pct);
    rightBack.spin(forward,direction2* minimum_velocity,pct);
    leftBack.spin(forward,direction3* minimum_velocity,pct);
    
    double leftStartPoint = leftFront.rotation(rev);
    double leftEndPoint = leftStartPoint + wheelRevs3;
    double rightStartPoint = rightFront.rotation(rev);
    double rightEndPoint = rightStartPoint + wheelRevs2;
    
    double leftBStartPoint = leftBack.rotation(rev);
    double leftBEndPoint = leftBStartPoint + wheelRevs3;
    double rightBStartPoint = rightBack.rotation(rev);
    double rightBEndPoint = rightBStartPoint + wheelRevs2;
    
    while (
            (direction2* (rightFront.rotation(rev) - rightStartPoint) < direction2* wheelRevs2) ||
            (direction3* (leftFront.rotation(rev) - leftStartPoint) < direction3 * wheelRevs3)  ||
            (direction3* (leftBack.rotation(rev) - leftBStartPoint) < direction3 * wheelRevs3)  ||
            (direction2* (rightBack.rotation(rev) - rightBStartPoint) < direction2 * wheelRevs2)  
          ) 
    {
        if (direction2 * (rightFront.rotation(rev) - rightStartPoint) < direction2 * wheelRevs2) 
        {
            rightFront.setVelocity(
                direction2 * std::min(
                    maxVelocity,
                    std::min(
                        tincreasing_speed(rightStartPoint,rightFront.rotation(rev)),
                        tdecreasing_speed(rightEndPoint,rightFront.rotation(rev))
                    )
                ),
                pct
            );
        } 
        else 
        {
            rightFront.stop(brake);
        }
        
        if (direction3 * (leftFront.rotation(rev) - leftStartPoint) < direction3 * wheelRevs3) 
        {
            leftFront.setVelocity(
                direction3 * std::min(
                    maxVelocity,
                    std::min(
                        tincreasing_speed(leftStartPoint,leftFront.rotation(rev)),
                        tdecreasing_speed(leftEndPoint,leftFront.rotation(rev))
                    )
                ),
                pct
            );
        } 
        else 
        {
            leftFront.stop(brake);
        }
        
        if (direction3 * (leftBack.rotation(rev) - leftBStartPoint) < direction3 * wheelRevs3) {
            leftBack.setVelocity(
                direction3 * std::min(
                    maxVelocity,
                    std::min(
                        tincreasing_speed(leftBStartPoint,leftBack.rotation(rev)),
                        tdecreasing_speed(leftBEndPoint,leftBack.rotation(rev))
                    )
                ),
                pct
            );
        } 
        else 
        {
            leftBack.stop(brake);
        }
        
        if (direction2 * (rightBack.rotation(rev) - rightBStartPoint) < direction2 * wheelRevs2) {
            rightBack.setVelocity(
                direction2 * std::min(
                    maxVelocity,
                    std::min(
                        tincreasing_speed(rightBStartPoint,rightBack.rotation(rev)),
                        tdecreasing_speed(rightBEndPoint,rightBack.rotation(rev))
                    )
                ),
                pct
            );
        } 
        else 
        {
            rightBack.stop(brake);
        }
    }
    leftFront.stop(brake);
    leftBack.stop(brake);
    rightFront.stop(brake);
    rightBack.stop(brake);
}

void intakeM () {
 intakeLeft.spin(directionType::fwd, 100, velocityUnits::pct);
 intakeRight.spin(directionType::fwd, 100, velocityUnits::pct);
}
 
void outake (int speed) {
 intakeLeft.spin(directionType::fwd, -speed, velocityUnits::rpm);
 intakeRight.spin(directionType::fwd, -speed, velocityUnits::rpm);
}

void intakeStop () {
  intakeLeft.stop(hold);
  intakeRight.stop(hold);
}
 
void pushOutForward() {
 pushOut.rotateTo(1170, deg, 40, rpm, true);
}
void pushOutReturn() {
 pushOut.rotateTo(0, rev, 80, rpm, true);
}

void stopB() {
  leftFront.stop(brake);
  leftBack.stop(brake);
  rightFront.stop(brake);
  rightBack.stop(brake);
}

void stopH() {
  leftFront.stop(hold);
  leftBack.stop(hold);
  rightFront.stop(hold);
  rightBack.stop(hold);
}

void outakeTime(int time, int speed) {
  intakeLeft.spin(fwd, -speed, pct);
  intakeRight.spin(fwd, -speed, pct);
  wait(time, msec);
  intakeLeft.stop(hold);
  intakeRight.stop(hold);
}

void moveTime(int speed, int time) {
  leftFront.spin(fwd, speed, pct);
  leftBack.spin(fwd, speed, pct);
  rightFront.spin(fwd, speed, pct);
  rightBack.spin(fwd, speed, pct);
  
  wait(time, msec);

  leftFront.stop(hold);
  leftBack.stop(hold);
  rightFront.stop(hold);
  rightBack.stop(hold);
}


void autonomous(void) {
  pushOut.rotateTo(500, deg, 100, rpm, true);
  pushOut.rotateTo(0, deg, 100, rpm, true);
  wait(50, msec);
  lift.rotateTo(260, deg, 90, rpm, true);
  lift.rotateTo(-20, deg, 90, rpm, true);
  lift.stop(hold);
  lift.resetRotation();
  wait(250, msec);
  
  intakeM();
  move(102.5, 50);
  stopH();
  intakeStop();

  moveTime(20, 2500);

  wait(500, msec);

  move(-7, 35);
  stopH();

  wait(500, msec);
  turn(87, 30);
  wait(500, msec);

  intakeM();
  wait(800, msec);
  intakeStop();

  moveTime(20, 1275);

  outakeTime(150, 50);

  intakeLeft.stop(hold);
  intakeRight.stop(hold);

  wait(500, msec);
  intakeLeft.stop(coast);
  intakeRight.stop(coast);
  pushOutForward();
  wait(500, msec);

  moveTime(6, 1250);

  wait(500, msec);

  intakeStop();

  outake(50);
  wait(500, msec);

  move(-7, 25);
  stopH();

  intakeStop();

  move(-29.25, 60);
  stopH();

  wait(500, msec);
  turn(320, 30);
  wait(500, msec);

  pushOutReturn();
  wait(500, msec);
  intakeM();

  move(17, 60);
  stopH();

  wait(500, msec);

  move(2, 30);
  stopH();

  wait(500, msec);

  move(-4.5, 20);
  stopH();

  intakeStop();
  wait(500, msec);
  
  outakeTime(500, 55);
  intakeStop();

  wait(500, msec);

  lift.rotateTo(500,deg,100,velocityUnits::pct);
  lift.stop(hold);

  wait(500, msec);

  move(7.25, 20);
  stopH();

  outake(73);

  wait(600, msec);

  intakeStop();

  move(-6, 40);
  stopH();

  lift.rotateTo(0,deg,100,velocityUnits::pct);
  lift.stop(hold);

  pushOut.rotateTo(585, deg, 90, rpm, false);
  move(-26, 90);
  stopH();
  pushOut.stop(hold);

  wait(500, msec);  

  turn(219, 40);

  wait(250, msec);

  pushOut.rotateTo(0, deg, 80, rpm, false);
  wait(350, msec);
  intakeM();
  move(18, 70);
  stopH();
  pushOut.stop(hold);

  wait(500, msec);

  move(-5.025, 20);
  stopH();

  intakeStop();
  wait(500, msec);
  
  outakeTime(500, 55);
  intakeStop();

  wait(500, msec);

  lift.rotateTo(650,deg,100,velocityUnits::pct);
  lift.stop(hold);

  wait(500, msec);

  move(6.8, 20);
  stopH();

  outake(77);

  wait(1000, msec);

  move(-10, 80);
  stopH();
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

int liftVal1 = 0;
int liftVal2 = 0;
int depositVal = 0;

int driveTank() 
{
  leftFront.spin(fwd, Controller1.Axis3.value(), pct);
  leftBack.spin(fwd, Controller1.Axis3.value(), pct);
  rightFront.spin(fwd, Controller1.Axis2.value(), pct);
  rightBack.spin(fwd, Controller1.Axis2.value(), pct);
  return(0);
}

int driveSplitArcade() 
{
  leftFront.spin(fwd, Controller1.Axis3.value() + (0.50*Controller1.Axis1.value()), pct);
  leftBack.spin(fwd, Controller1.Axis3.value() + (0.50*Controller1.Axis1.value()), pct);
  rightFront.spin(fwd, Controller1.Axis3.value() - (0.50*Controller1.Axis1.value()), pct);
  rightBack.spin(fwd, Controller1.Axis3.value() - (0.50*Controller1.Axis1.value()), pct);
  return(0);
}

int driveOneArcade() 
{
  leftFront.spin(fwd, Controller1.Axis3.value() + Controller1.Axis4.value(), pct);
  leftBack.spin(fwd, Controller1.Axis3.value() + Controller1.Axis4.value(), pct);
  rightFront.spin(fwd, Controller1.Axis3.value() - Controller1.Axis4.value(), pct);
  rightBack.spin(fwd, Controller1.Axis3.value() - Controller1.Axis4.value(), pct);
  return(0);
}

int intake() 
{
  if (Controller1.ButtonR1.pressing())
  {
    intakeLeft.spin(fwd, 100, pct);
    intakeRight.spin(fwd, 100, pct);
  }
  else if (Controller1.ButtonR2.pressing())
  {
    intakeLeft.spin(fwd, -50, pct);
    intakeRight.spin(fwd, -50, pct);
  }
  else
  {
    intakeLeft.stop(hold);
    intakeRight.stop(hold);
  }
  return(0);
}

int lifting() 
{
  if (Controller1.ButtonL1.pressing() && liftVal1==0 && Brain.timer(msec)>300)
  {
    Brain.resetTimer();
    liftVal1=1;
    lift.rotateTo(650,deg,100,velocityUnits::pct);
    lift.stop(hold);
  }

  if (Controller1.ButtonL1.pressing() && liftVal1==1 && Brain.timer(msec)>300)
  {
    Brain.resetTimer();
    liftVal1=0;
    lift.rotateTo(0,deg,100,velocityUnits::pct);
    lift.stop(hold);
  }

  if (Controller1.ButtonL2.pressing() && liftVal2==0 && Brain.timer(msec)>300)
  {
    Brain.resetTimer();
    liftVal2=1;
    lift.rotateTo(500,deg,100,velocityUnits::pct);
    lift.stop(hold);
  }

  if (Controller1.ButtonL2.pressing() && liftVal2==1 && Brain.timer(msec)>300)
  {
    Brain.resetTimer();
    liftVal2=0;
    lift.rotateTo(0,deg,100,velocityUnits::pct);
    lift.stop(hold);
  }
  return(0);
}

int removing() 
{
  if (Controller1.ButtonX.pressing() && liftVal1==0 && Brain.timer(msec)>300)
  {
    Brain.resetTimer();
    liftVal1=1;
    lift.rotateTo(800,deg,100,velocityUnits::pct);
    lift.stop(hold);
  }

  if (Controller1.ButtonX.pressing() && liftVal1==1 && Brain.timer(msec)>300)
  {
    Brain.resetTimer();
    liftVal1=0;
    lift.rotateTo(0,deg,100,velocityUnits::pct);
    lift.stop(hold);
  }

  if (Controller1.ButtonB.pressing() && liftVal2==0 && Brain.timer(msec)>300)
  {
    Brain.resetTimer();
    liftVal2=1;
    lift.rotateTo(650,deg,100,velocityUnits::pct);
    lift.stop(hold);
  }

  if (Controller1.ButtonB.pressing() && liftVal2==1 && Brain.timer(msec)>300)
  {
    Brain.resetTimer();
    liftVal2=0;
    lift.rotateTo(0,deg,100,velocityUnits::pct);
    lift.stop(hold);
  }
  return(0);
}

int deposit() 
{
  if (Controller1.ButtonUp.pressing() && depositVal==0 && Brain.timer(msec)>300)
  {
    Brain.resetTimer();
    depositVal=1;
    pushOut.rotateTo(1120,deg,50,velocityUnits::pct);
    pushOut.stop(hold);

  }

  if (Controller1.ButtonUp.pressing() && depositVal==1 && Brain.timer(msec)>300)
  {
    Brain.resetTimer();
    depositVal=0;
    pushOut.rotateTo(0,deg,100,velocityUnits::pct);
    pushOut.stop(hold);
  }
  return(0);
}

void usercontrol(void) {
  // User control code here, inside the loop
  pushOut.rotateTo(400, deg, 100, rpm, true);
  pushOut.rotateTo(0, deg, 70, rpm, true);
  wait(50, msec);
  lift.rotateTo(260, deg, 65, rpm, true);
  lift.rotateTo(-35, deg, 70, rpm, true);
  lift.resetRotation();
  wait(250, msec);
  while (1) {
    vex::task drives(driveSplitArcade);
    vex::task intakes(intake);
    vex::task liftings(lifting);
    vex::task deposits(deposit);
    vex::task removings(removing);

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and calleftBackacks.
//
int main() {
  // Set up calleftBackacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
