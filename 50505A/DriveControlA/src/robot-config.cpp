#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor pushOut = motor(PORT5, ratio36_1, false);
motor leftFront = motor(PORT20, ratio18_1, false);
motor leftBack = motor(PORT10, ratio18_1, false);
motor rightFront = motor(PORT11, ratio18_1, true);
motor rightBack = motor(PORT1, ratio18_1, true);
motor lift = motor(PORT6, ratio36_1, false);
motor intakeLeft = motor(PORT17, ratio18_1, false);
motor intakeRight = motor(PORT14, ratio18_1, true);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}