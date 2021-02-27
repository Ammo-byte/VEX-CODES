using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor pushOut;
extern motor leftFront;
extern motor leftBack;
extern motor rightFront;
extern motor rightBack;
extern motor lift;
extern motor intakeLeft;
extern motor intakeRight;
extern controller Controller1;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );