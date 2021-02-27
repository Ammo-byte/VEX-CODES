#include "vex.h"

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// pushOut              motor         5               
// leftFront            motor         20              
// leftBack             motor         10              
// rightFront           motor         11              
// rightBack            motor         1               
// lift                 motor         6               
// intakeLeft           motor         17              
// intakeRight          motor         14              
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "math.h"
#include "algorithm"
using namespace vex;

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
    lift.rotateTo(-900,deg,100,velocityUnits::pct);
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
    lift.rotateTo(-700,deg,100,velocityUnits::pct);
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
    lift.rotateTo(-800,deg,100,velocityUnits::pct);
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
    lift.rotateTo(-650,deg,100,velocityUnits::pct);
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
    pushOut.rotateTo(1400,deg,50,velocityUnits::pct);
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

int main() 
{
  vexcodeInit();
  
  while(true) 
  {
    vex::task drives(driveSplitArcade);
    vex::task intakes(intake);
    vex::task liftings(lifting);
    vex::task deposits(deposit);
    vex::task removings(removing);
  }
}
