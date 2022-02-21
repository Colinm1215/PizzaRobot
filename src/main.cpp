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
// Controller1          controller                    
// FrontLeftMotor       motor         1               
// FrontRightMotor      motor         2               
// BackLeftMotor        motor         3               
// BackRightMotor       motor         4               
// LeftLiftMotor        motor         5               
// RightLiftMotor       motor         6               
// GrabMotor            motor         7               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <iostream>;

using namespace vex;
using namespace std;

// A global instance of competition
competition Competition;

int gearRatio = 1;
bool calibration = true;

class armState {
  const float start = 210;
  const float opened = 80;
  const float closed = 50;
};

const float liftState[] = {-600, -500, -380, -300, -220, -640};
int currentLiftState;
const float armState[] = {50, 80};
int currentArmState;

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
  cout << "setup\n";
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  // Do not put code here that will power any motors;  
  // it will not run
}

void autoStraight(float rotationSpeed, bool forward) {
  int direction = 1;
  if (forward == false) direction = -1;
  FrontLeftMotor.setVelocity(direction*rotationSpeed*gearRatio, velocityUnits::rpm);
  FrontLeftMotor.spin(directionType::fwd);
  FrontRightMotor.setVelocity(direction*rotationSpeed*gearRatio, velocityUnits::rpm);
  FrontRightMotor.spin(directionType::fwd);
  BackLeftMotor.setVelocity(direction*rotationSpeed, velocityUnits::rpm);
  BackLeftMotor.spin(directionType::fwd);
  BackRightMotor.setVelocity(direction*rotationSpeed, velocityUnits::rpm);
  BackRightMotor.spin(directionType::fwd);
}

void autoTurn(float turnDegrees, bool left) {
  int direction = 1;
  if (left == false) direction = -1;
  //FrontLeftMotor.rotateFor(90, rotationUnits::rev);
  FrontLeftMotor.rotateFor(-direction*turnDegrees*gearRatio, rotationUnits::rev);
  FrontRightMotor.rotateFor(direction*turnDegrees*gearRatio, rotationUnits::rev);
  BackLeftMotor.rotateFor(-direction*turnDegrees, rotationUnits::rev);
  BackRightMotor.rotateFor(direction*turnDegrees, rotationUnits::rev);
}

void autoLift(int level) {
  currentLiftState = level;
  if (LeftLiftMotor.velocity(velocityUnits::rpm) == 0) {
    LeftLiftMotor.setVelocity(-100, velocityUnits::rpm);
    RightLiftMotor.setVelocity(-100, velocityUnits::rpm);
    LeftLiftMotor.spinTo(liftState[currentLiftState], rotationUnits::deg, false);
    RightLiftMotor.spinTo(liftState[currentLiftState], rotationUnits::deg, false);
  }
  /*if (level == 0) {
    LeftLiftMotor.setVelocity(0, velocityUnits::rpm);
    RightLiftMotor.setVelocity(0, velocityUnits::rpm);
  }*/
}

void autoGrab(float armPosition) {
  currentArmState = armPosition;
}

void controlRobot() {
  float straightSpeed = Controller1.Axis3.value()*2;
  float turnSpeed = Controller1.Axis4.value();
  FrontLeftMotor.setVelocity(straightSpeed*gearRatio + turnSpeed*gearRatio, velocityUnits::rpm);
  FrontLeftMotor.spin(directionType::fwd);
  FrontRightMotor.setVelocity(straightSpeed*gearRatio - turnSpeed*gearRatio, velocityUnits::rpm);
  FrontRightMotor.spin(directionType::fwd);
  BackLeftMotor.setVelocity(straightSpeed + turnSpeed, velocityUnits::rpm);
  BackLeftMotor.spin(directionType::fwd);
  BackRightMotor.setVelocity(straightSpeed - turnSpeed, velocityUnits::rpm);
  BackRightMotor.spin(directionType::fwd);
}

void controlLift() {
  LeftLiftMotor.setVelocity(-100, velocityUnits::rpm);
  RightLiftMotor.setVelocity(-100, velocityUnits::rpm);
  LeftLiftMotor.spinTo(liftState[currentLiftState], rotationUnits::deg);
  RightLiftMotor.spinTo(liftState[currentLiftState], rotationUnits::deg);

  float liftSpeed = Controller1.Axis2.value();
}

void controlGrab() {
  float grabSpeed = Controller1.Axis1.value()/2;
  GrabMotor.setVelocity(grabSpeed, velocityUnits::rpm);
  GrabMotor.spin(directionType::fwd);
}

void calibrateLift() {
  cout << "Start Calibrate\n";
  LeftLiftMotor.setVelocity(100, velocityUnits::rpm);
  LeftLiftMotor.spin(directionType::fwd);
  RightLiftMotor.setVelocity(100, velocityUnits::rpm);
  RightLiftMotor.spin(directionType::fwd);
  cout << "Motor Move\n";

  wait(100, msec);
  cout << LeftLiftMotor.current();
  while(LeftLiftMotor.velocity(velocityUnits::rpm) > 20) {} cout << LeftLiftMotor.current();
  cout << "Motor Loop ended\n";
  wait(1000, msec);
  LeftLiftMotor.resetPosition();
  LeftLiftMotor.resetRotation();
  RightLiftMotor.resetPosition();
  RightLiftMotor.resetRotation();
  LeftLiftMotor.setVelocity(0, velocityUnits::rpm);
  LeftLiftMotor.spin(directionType::fwd);
  RightLiftMotor.setVelocity(0, velocityUnits::rpm);
  RightLiftMotor.spin(directionType::fwd);
  cout << "Motor Stop\n";
  
}

void calibrateArm() {
  cout << LeftLiftMotor.position(rotationUnits::deg);
  cout << "Calibrate Arm\n";
  autoLift(1);
  while(LeftLiftMotor.isSpinning() == true) {}
  GrabMotor.setVelocity(100, velocityUnits::rpm);
  GrabMotor.spin(directionType::fwd);
  wait(100, msec);
  while(GrabMotor.velocity(velocityUnits::rpm) > 25) {}

  GrabMotor.setVelocity(0, velocityUnits::rpm);
  GrabMotor.spin(directionType::fwd);
  GrabMotor.resetRotation();
  GrabMotor.resetPosition();
  wait(1000, msec);
  autoLift(4);
  calibration = false;
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
  cout << "auto\n";
  calibrateLift();
  calibrateArm();
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
  //pre_auton();
  //This will run the auto function but it will end when any controller button is pressed
  autonomous();
  //calibrateLift();
  cout << "teleop\n";
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
    /*// int32_t is 4 byte signed integer value
    int32_t Y = Controller1.Axis3.value() / 2;
    int32_t X = Controller1.Axis4.value() / 2;
    int32_t R = Controller1.Axis1.value() / 2;

    int32_t frontMotorVal = Y + X + R;
    int32_t backMotorVal = -Y + X + R;

    //FrontMotors.setVelocity(frontMotorVal, vex::velocityUnits::pct);
    //BackMotors.setVelocity(backMotorVal, vex::percentUnits::pct);

    FrontMotors.setVelocity(5, vex::velocityUnits::rpm);
    BackMotors.setVelocity(5, vex::velocityUnits::rpm);


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
    }*/
    if (calibration == false) {
      controlRobot();
      controlLift();
      controlGrab();
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