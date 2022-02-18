using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor_group FrontMotors;
extern motor_group BackMotors;
extern controller Controller1;
extern motor ArmMotor;
extern motor ClawRotate;
extern motor ClawGrip;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );