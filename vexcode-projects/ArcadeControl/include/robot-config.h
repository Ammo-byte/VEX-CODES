using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor frontleft;
extern motor frontright;
extern motor backleft;
extern motor backright;
extern controller Controller1;
extern encoder Backencoder;
extern encoder Leftencoder;
extern encoder Rightencoder;
extern limit LimitSwitchA;
extern optical Optical;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );