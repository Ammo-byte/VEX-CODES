using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor backright;
extern motor frontright;
extern motor backleft;
extern motor frontleft;
extern encoder Back;
extern encoder right;
extern encoder left;
extern controller Controller1;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );