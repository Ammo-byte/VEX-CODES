/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       akulrishi                                                 */
/*    Created:      Fri Sep 18 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// frontleft            motor         1               
// frontright           motor         10              
// backleft             motor         11              
// backright            motor         20              
// Controller1          controller                    
// Backencoder          encoder       C, D            
// Leftencoder          encoder       E, F            
// Rightencoder         encoder       G, H            
// LimitSwitchA         limit         A               
// Optical              optical       6               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;
//sep 25:
// created tank control/drive control
// created deadzone for the controller on both tank/drive control
int deadzone = 5;
int tankcontrol()
{
  while(true)
  {
    frontleft.spin(forward,Controller1.Axis3.value(),pct);
    frontright.spin(forward,Controller1.Axis2.value(),pct);
    backleft.spin(forward,Controller1.Axis3.value(),pct);
    backright.spin(forward,Controller1.Axis2.value(),pct);
  }
}

int tankcontrol2()
{
  while(true)
  {
    if(abs(Controller1.Axis3.value()) > deadzone)
    {
      frontleft.spin(forward,Controller1.Axis3.value(),pct);
      backleft.spin(forward, Controller1.Axis3.value(),pct);
    }
    else
    {
      frontleft.stop();
      backleft.stop();
    }
    if(abs(Controller1.Axis2.value()) > deadzone)
    {
      backright.spin(forward,Controller1.Axis2.value(),pct);
      frontright.spin(forward,Controller1.Axis2.value(),pct);
    }
    else
    {
      backright.stop();
      frontright.stop();
    }
  }
}

int arcadecontrol()
{
  while(true)
  {
    frontleft.spin(forward,Controller1.Axis3.value() + Controller1.Axis4.value(),pct);
    frontright.spin(forward,Controller1.Axis3.value() - Controller1.Axis4.value(),pct);
    backleft.spin(forward,Controller1.Axis3.value() + Controller1.Axis4.value(),pct);
    backright.spin(forward,Controller1.Axis3.value() - Controller1.Axis4.value(),pct);
  }
}

int arcadecontrol2()
{
  while(true)
  {
    if((abs(Controller1.Axis3.value()) || abs(Controller1.Axis4.value())) > deadzone)
    {
      frontleft.spin(forward,Controller1.Axis3.value() + Controller1.Axis4.value(),pct);
      frontright.spin(forward,Controller1.Axis3.value() - Controller1.Axis4.value(),pct);
      backleft.spin(forward,Controller1.Axis3.value() + Controller1.Axis4.value(),pct);
      backright.spin(forward,Controller1.Axis3.value() - Controller1.Axis4.value(),pct);
    }
    else
    {
      backright.stop();
      frontright.stop();
      frontleft.stop();
      backleft.stop();
    }
  }
}

//Oct 2 
//got robot to drive forward at a certain speed, and stop at a cetain time
// robot drives forward based on motor encoder values  
int forwardtime(int times, int speed)
{
  Brain.Timer.reset();
  while(Brain.timer(msec) < times)
  {
    frontright.spin(directionType::fwd,speed,velocityUnits::pct);
    frontleft.spin(directionType::fwd,speed,velocityUnits::pct);
     backright.spin(directionType::fwd,speed,velocityUnits::pct);
    backleft.spin(directionType::fwd,speed,velocityUnits::pct);
  }
  
  frontleft.stop(coast);
  frontright.stop(coast);
  backleft.stop(coast);
  backright.stop(coast);

  return(0);
}

int forwardencoder(int encodervalue, int speed)
{
  frontleft.resetRotation();
  while(frontleft.rotation(deg) < encodervalue)
  {
    frontright.spin(directionType::fwd,speed,velocityUnits::pct);
    frontleft.spin(directionType::fwd,speed,velocityUnits::pct);
     backright.spin(directionType::fwd,speed,velocityUnits::pct);
    backleft.spin(directionType::fwd,speed,velocityUnits::pct);
  }
  {
  frontleft.stop(coast);
  frontright.stop(coast);
  backleft.stop(coast);
  backright.stop(coast);
  }
  return(0);
}

int movedistance(int encodervalue, int speed)
{
  frontleft.resetRotation();
  while(frontleft.rotation(deg) < encodervalue)
  {
    frontright.spin(directionType::fwd,speed,velocityUnits::pct);
    frontleft.spin(directionType::fwd,speed,velocityUnits::pct);
     backright.spin(directionType::fwd,speed,velocityUnits::pct);
    backleft.spin(directionType::fwd,speed,velocityUnits::pct);
  }
  
  frontleft.stop(coast);
  frontright.stop(coast);
  backleft.stop(coast);
  backright.stop(coast);
  return(0);
}

int movedistanceback(int encodervalue, int speed)
{
  frontleft.resetRotation();
  while(frontleft.rotation(deg) > -encodervalue)
  {
    frontright.spin(directionType::fwd,speed,velocityUnits::pct);
    frontleft.spin(directionType::fwd,speed,velocityUnits::pct);
     backright.spin(directionType::fwd,speed,velocityUnits::pct);
    backleft.spin(directionType::fwd,speed,velocityUnits::pct);
  }
  
  frontleft.stop(coast);
  frontright.stop(coast);
  backleft.stop(coast);
  backright.stop(coast);
  return(0);
}

//Oct 9
int movedistanceboth(int encodervalue, int speed) // code that has both forwards and backwards movement by changing speed to either negative or positive
{
  frontleft.resetRotation();



  if(speed < 0) //backward
  {
    while(frontleft.rotation(deg) > -encodervalue)
    {
      frontright.spin(directionType::fwd,speed,velocityUnits::pct);
      frontleft.spin(directionType::fwd,speed,velocityUnits::pct);
      backright.spin(directionType::fwd,speed,velocityUnits::pct);
      backleft.spin(directionType::fwd,speed,velocityUnits::pct);
    }
  }


 if(speed > 0) //forward
  {
    while(frontleft.rotation(deg) < encodervalue)
    {
      frontright.spin(directionType::fwd,speed,velocityUnits::pct);
      frontleft.spin(directionType::fwd,speed,velocityUnits::pct);
      backright.spin(directionType::fwd,speed,velocityUnits::pct);
      backleft.spin(directionType::fwd,speed,velocityUnits::pct);
    }
  }
  
  frontleft.stop(coast);
  frontright.stop(coast);
  backleft.stop(coast);
  backright.stop(coast);

    return(0);

}

int moverotate(int encodervalue, int speed) // code that has both forwards and backwards movement by changing speed to either negative or positive
{
  frontleft.resetRotation();



  if(speed < 0) //backward
  {
    while(frontleft.rotation(deg) > -encodervalue)
    {
      frontright.spin(directionType::rev,speed,velocityUnits::pct);
      frontleft.spin(directionType::fwd,speed,velocityUnits::pct);
      backright.spin(directionType::rev,speed,velocityUnits::pct);
      backleft.spin(directionType::fwd,speed,velocityUnits::pct);
    }
  }


 if(speed > 0) //forward
  {
    while(frontleft.rotation(deg) < encodervalue)
    {
      frontright.spin(directionType::rev,speed,velocityUnits::pct);
      frontleft.spin(directionType::fwd,speed,velocityUnits::pct);
      backright.spin(directionType::rev,speed,velocityUnits::pct);
      backleft.spin(directionType::fwd,speed,velocityUnits::pct);
    }
  }
  
  frontleft.stop(brake);
  frontright.stop(brake);
  backleft.stop(brake);
  backright.stop(brake);

    return(0);

}

int printVals()
{
  while(true)
  {
    Brain.Screen.clearLine(1,black);
    Brain.Screen.setCursor(1,0);
    Brain.Screen.print("L: %f deg", Leftencoder.rotation(deg));
    Brain.Screen.clearLine(2,black);
    Brain.Screen.setCursor(2,0);
    Brain.Screen.print("R: %f deg", Rightencoder.rotation(deg));
    Brain.Screen.clearLine(3,black);
    Brain.Screen.setCursor(3,0);
    Brain.Screen.print("B: %f deg", Backencoder.rotation(deg));
    Brain.Screen.clearLine(4,black);
    Brain.Screen.setCursor(4,0);
    Brain.Screen.print("Diff: %f deg", Leftencoder.rotation(deg)-Rightencoder.rotation(deg));
    Brain.Screen.clearLine(5,black);
    Brain.Screen.setCursor(5,0);
    Brain.Screen.print("Bumper Swith: %f value", LimitSwitchA.pressing());
  }
}

int moverotateEncoder(int encodervalue) // code that has both forwards and backwards movement by changing speed to either negative or positive
{

  
  if((Leftencoder.rotation(deg)-Rightencoder.rotation(deg)) < encodervalue)
  {
    while((Leftencoder.rotation(deg)-Rightencoder.rotation(deg)) < encodervalue)
    { 
        frontright.spin(directionType::rev,50,velocityUnits::pct);
        frontleft.spin(directionType::fwd,50,velocityUnits::pct);
        backright.spin(directionType::rev,50,velocityUnits::pct);
        backleft.spin(directionType::fwd,50,velocityUnits::pct);
    }  
  }

  else if((Leftencoder.rotation(deg)-Rightencoder.rotation(deg)) > encodervalue)
  {
    while((Leftencoder.rotation(deg)-Rightencoder.rotation(deg)) > encodervalue)
    {
        frontright.spin(directionType::fwd,50,velocityUnits::pct);
        frontleft.spin(directionType::rev,50,velocityUnits::pct);
        backright.spin(directionType::fwd,50,velocityUnits::pct);
        backleft.spin(directionType::rev,50,velocityUnits::pct);
    }
  }
  
  frontleft.stop(brake);
  frontright.stop(brake);
  backleft.stop(brake);
  backright.stop(brake);

  return(0);

}

int LimitSwitchVal = 0;

int bumperswitchrepeat()
{
  while(true)
  {
    if (LimitSwitchA.pressing())
    {
      LimitSwitchVal = 0;
    }
    else
    {
      LimitSwitchVal = 1; 
    }
    
  }
}

//int LimitSwitchVal = 0;

//int bumperswitchrepeat()
//{
 // while(true)
 // {
   // if (LimitSwitchA.pressing()&& LimitSwitchVal == 0)
   // {
     // LimitSwitchVal = 0;
    //}
   // if (LimitSwitchA.pressing() && LimitSwitchVal ==1)
    //{
     // LimitSwitchVal = 1; 
    //}
  //}
//}


int main()
{ 
  vexcodeInit();
}

 





