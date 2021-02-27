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
// backright            motor         20              
// frontright           motor         10              
// backleft             motor         11              
// frontleft            motor         1               
// Back                 encoder       C, D            
// right                encoder       E, F            
// left                 encoder       G, H            
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;



int main(){ 
vexcodeInit();
while(true){
frontleft.spin(forward,Controller1.Axis3.value(),pct);
frontright.spin(forward,Controller1.Axis2.value(),pct);
backleft.spin(forward,Controller1.Axis3.value(),pct);
backright.spin(forward,Controller1.Axis2.value(),pct);
}

}


