/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\camre                                            */
/*    Created:      Thu Feb 17 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FrontMotors          motor_group   1, 2            
// BackMotors           motor_group   3, 4            
// Controller1          controller                    
// ArmMotor             motor         5               
// ClawRotate           motor         6               
// ClawGrip             motor         7               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  // Do not put code here that will power any motors;  
  // it will not run
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
void autonomous(void) {
  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
void usercontrol(void) {
  //This will run the auto function but it will end when any controller button is pressed
  //autonomous();
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.
    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    // You can also call functions from here; keep in mind they will loop
    wait(20, msec);

    /*
    I'm guessing you guys are making an X holonomic system :

                 FL /   \ FR
                 BL \   / BR

                 FrontMotors is FL+BR
                 BackMotors is FR+BL

                  I'll draw out a better schematic of what I'm thinking
    */

    // int32_t is 4 byte signed integer value
    int32_t Y = Controller1.Axis3.value() / 2;
    int32_t X = Controller1.Axis4.value() / 2;
    int32_t R = Controller1.Axis1.value() / 2;

    int32_t frontMotorVal = Y + X + R;
    int32_t backMotorVal = -Y + X + R;

    FrontMotors.setVelocity(frontMotorVal, vex::velocityUnits::pct);
    BackMotors.setVelocity(backMotorVal, vex::percentUnits::pct);



    bool armUp = Controller1.ButtonUp.pressing();
    bool armDown = Controller1.ButtonDown.pressing();

    bool clawRotateUp = Controller1.ButtonX.pressing();
    bool clawRotateDown = Controller1.ButtonB.pressing();

    bool closeClaw = Controller1.ButtonR1.pressing();
    bool openClaw = Controller1.ButtonR2.pressing();

    int armCase = (armUp) ? ((armDown) ? -1 : 1) : ((armDown) ? 2 : -1);
    int clawRotateCase = (clawRotateUp) ? ((clawRotateDown) ? -1 : 1) : ((clawRotateDown) ? 2 : -1);
    int clawCase = (closeClaw) ? ((openClaw) ? -1 : 1) : ((openClaw) ? 2 : -1);

    switch (armCase) {
      case 1:
        // move arm up
        break;
      case 2:
        // move arm down
        break;
      default: 
        // do nothing
        break;
    }

    switch (clawRotateCase) {
      case 1:
        // move claw up
        break;
      case 2:
        // move claw down
        break;
      default: 
        // do nothing
        break;
    }

    switch (clawCase) {
      case 1:
        // close claw
        break;
      case 2:
        // open claw
        break;
      default: 
        // do nothing
        break;
    }
  }
}

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  // Run the pre-autonomous function.
  pre_auton();
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}