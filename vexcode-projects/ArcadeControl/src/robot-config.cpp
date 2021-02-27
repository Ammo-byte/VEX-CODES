#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor frontleft = motor(PORT1, ratio18_1, false);
motor frontright = motor(PORT10, ratio18_1, true);
motor backleft = motor(PORT11, ratio18_1, false);
motor backright = motor(PORT20, ratio18_1, true);
controller Controller1 = controller(primary);
encoder Backencoder = encoder(Brain.ThreeWirePort.C);
encoder Leftencoder = encoder(Brain.ThreeWirePort.E);
encoder Rightencoder = encoder(Brain.ThreeWirePort.G);
limit LimitSwitchA = limit(Brain.ThreeWirePort.A);
optical Optical = optical(PORT6);

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