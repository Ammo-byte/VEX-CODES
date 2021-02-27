#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor pushOut = motor(PORT5, ratio36_1, false);
motor leftFront = motor(PORT20, ratio18_1, false);
motor leftBack = motor(PORT10, ratio18_1, false);
motor rightFront = motor(PORT11, ratio18_1, true);
motor rightBack = motor(PORT1, ratio18_1, true);
motor lift = motor(PORT7, ratio36_1, true);
motor intakeLeft = motor(PORT17, ratio18_1, false);
motor intakeRight = motor(PORT14, ratio18_1, true);
controller Controller1 = controller(primary);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}