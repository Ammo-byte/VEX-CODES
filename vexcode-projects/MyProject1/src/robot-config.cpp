#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor backright = motor(PORT20, ratio18_1, true);
motor frontright = motor(PORT10, ratio18_1, true);
motor backleft = motor(PORT11, ratio18_1, false);
motor frontleft = motor(PORT1, ratio18_1, false);
encoder Back = encoder(Brain.ThreeWirePort.C);
encoder right = encoder(Brain.ThreeWirePort.E);
encoder left = encoder(Brain.ThreeWirePort.G);
controller Controller1 = controller(primary);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}