#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor FrontMotorsMotorA = motor(PORT1, ratio18_1, false);
motor FrontMotorsMotorB = motor(PORT2, ratio18_1, true);
motor_group FrontMotors = motor_group(FrontMotorsMotorA, FrontMotorsMotorB);
motor BackMotorsMotorA = motor(PORT3, ratio18_1, false);
motor BackMotorsMotorB = motor(PORT4, ratio18_1, true);
motor_group BackMotors = motor_group(BackMotorsMotorA, BackMotorsMotorB);
controller Controller1 = controller(primary);
motor ArmMotor = motor(PORT5, ratio18_1, false);
motor ClawRotate = motor(PORT6, ratio18_1, false);
motor ClawGrip = motor(PORT7, ratio18_1, false);

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