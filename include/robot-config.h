using namespace vex;

extern brain Brain;

using signature = vision::signature;

// VEXcode devices
extern controller Controller1;
extern motor FrontLeftMotor;
extern motor FrontRightMotor;
extern motor BackLeftMotor;
extern motor BackRightMotor;
extern motor GrabMotor;
extern sonar LeftRangeFinder;
extern sonar RightRangeFinder;
extern line LeftLineTracker;
extern line RightLineTracker;
extern signature Vision__PIZZABOX;
extern signature Vision__GOLDENPIZZA;
extern signature Vision__SIG_3;
extern signature Vision__SIG_4;
extern signature Vision__SIG_5;
extern signature Vision__SIG_6;
extern signature Vision__SIG_7;
extern vision Vision;
extern motor_group MotorGroup5;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );