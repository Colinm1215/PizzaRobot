#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor FrontLeftMotor = motor(PORT1, ratio18_1, true);
motor FrontRightMotor = motor(PORT2, ratio18_1, false);
motor BackLeftMotor = motor(PORT3, ratio18_1, false);
motor BackRightMotor = motor(PORT4, ratio18_1, true);
motor LeftLiftMotor = motor(PORT5, ratio18_1, false);
motor RightLiftMotor = motor(PORT6, ratio18_1, true);
motor GrabMotor = motor(PORT7, ratio18_1, false);
sonar LeftRangeFinder = sonar(Brain.ThreeWirePort.A);
sonar RightRangeFinder = sonar(Brain.ThreeWirePort.C);
line LeftLineTracker = line(Brain.ThreeWirePort.E);
line RightLineTracker = line(Brain.ThreeWirePort.F);
/*vex-vision-config:begin*/
signature Vision__PIZZABOX = signature (1, 2433, 2985, 2709, -2243, -1645, -1944, 1.5, 0);
signature Vision__GOLDENPIZZA = signature (2, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision__SIG_3 = signature (3, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision__SIG_4 = signature (4, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision__SIG_5 = signature (5, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision__SIG_6 = signature (6, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision__SIG_7 = signature (7, 0, 0, 0, 0, 0, 0, 3, 0);
vision Vision = vision (PORT8, 36, Vision__PIZZABOX, Vision__GOLDENPIZZA, Vision__SIG_3, Vision__SIG_4, Vision__SIG_5, Vision__SIG_6, Vision__SIG_7);
/*vex-vision-config:end*/

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