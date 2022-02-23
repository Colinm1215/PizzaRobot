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
// LeftRangeFinder      sonar         A, B            
// RightRangeFinder     sonar         C, D            
// LeftLineTracker      line          E               
// RightLineTracker     line          F               
// Vision               vision        8               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <iostream>

using namespace vex;
using namespace std;

// A global instance of competition
competition Competition;

//sonar LeftRangeFinder = sonar(Brain.ThreeWirePort.A);
//sonar RightRangeFinder = sonar(Brain.ThreeWirePort.C);

bool calibration = true;
bool buttonAPushed = false;
bool liftActive = false;

const float lineTrackingWhiteKP = 0.05;
const int avgLineFollowVel = 50;
const float lineTrackingWhiteKD = 1;
float prevLTWError = 0;

const float lineTrackingBlackKP = 0.05;
const float lineTrackingBlackKD = 1;
float prevLTBError = 0;

const float liftState[] = {40, 80, 240, 380, 500, 640};
int currentLiftState;
const float armState[] = {-120, -280};
int currentArmState;

int prevRightLineVal;
int prevLeftLineVal;

enum {RAMP, WAITING, PIZZASEARCH, DORMSEARCH, APPROACHING, LIFTING, GRABBING, RELEASING};

int currentDorm = 0;

int currentState = RAMP;

bool reverseMovement = false;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void nothing() {}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  cout << "setup\n";
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  // Do not put code here that will power any motors;  
  // it will not run
}

void lineTracking(bool darkLine) {
  int curRightTrackerVal = 100-RightLineTracker.reflectivity();
  int curLeftTrackerVal = 100-LeftLineTracker.reflectivity();
  if (darkLine == true) {
    curRightTrackerVal = 100-RightLineTracker.reflectivity();
    curLeftTrackerVal = 100-LeftLineTracker.reflectivity();
  }
  else {
    curRightTrackerVal = RightLineTracker.reflectivity();
    curLeftTrackerVal = LeftLineTracker.reflectivity();
  }
  
  int error = curRightTrackerVal-curLeftTrackerVal;
  float effort = lineTrackingKP*error;
  float newVelRight = avgLineFollowVel + effort;
  float newVelLeft = avgLineFollowVel - effort;

  FrontRightMotor.setVelocity(newVelRight, velocityUnits::rpm);
  BackRightMotor.setVelocity(newVelRight, velocityUnits::rpm);
  FrontLeftMotor.setVelocity(newVelLeft, velocityUnits::rpm);
  BackLeftMotor.setVelocity(newVelLeft, velocityUnits::rpm);
}

void lineTrackingBlack() {
  int curRightTrackerVal = RightLineTracker.reflectivity();
  int curLeftTrackerVal = LeftLineTracker.reflectivity();

  int error = curRightTrackerVal-curLeftTrackerVal;

  float effort = lineTrackingBlackKP*error + lineTrackingBlackKD*prevLTBError;
  prevLTBError = error;

  float newVelRight = avgLineFollowVel + effort;
  float newVelLeft = avgLineFollowVel - effort;

  FrontRightMotor.setVelocity(newVelRight*gearRatio, velocityUnits::rpm);
  BackRightMotor.setVelocity(newVelRight, velocityUnits::rpm);

  FrontLeftMotor.setVelocity(newVelLeft*gearRatio, velocityUnits::rpm);
  BackLeftMotor.setVelocity(newVelLeft, velocityUnits::rpm);
}

bool checkLineTrack() {
  bool retVal = false;
  if (currentState == PIZZASEARCH || currentState == RAMP) {
    retVal = true;
  }
  return retVal;
}

void handleLineTrack() {
  if (currentState == PIZZASEARCH) lineTrackingWhite();
  else if (currentState == RAMP) lineTrackingBlack();
}

void autoStraight(float rotationSpeed, bool forward) {
  int direction = 1;
  if (forward == false) direction = -1;
  FrontLeftMotor.setVelocity(direction*rotationSpeed, velocityUnits::rpm);
  FrontLeftMotor.spin(directionType::fwd);
  FrontRightMotor.setVelocity(direction*rotationSpeed, velocityUnits::rpm);
  FrontRightMotor.spin(directionType::fwd);
  BackLeftMotor.setVelocity(direction*rotationSpeed, velocityUnits::rpm);
  BackLeftMotor.spin(directionType::fwd);
  BackRightMotor.setVelocity(direction*rotationSpeed, velocityUnits::rpm);
  BackRightMotor.spin(directionType::fwd);
}

void autoTurn(float turnDegrees, bool left) {
  int direction = 1;
  if (left == false) direction = -1;
  FrontLeftMotor.rotateFor(-direction*turnDegrees, rotationUnits::rev);
  FrontRightMotor.rotateFor(direction*turnDegrees, rotationUnits::rev);
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
}

void autoGrab(float armPosition) {
  currentArmState = armPosition;
  GrabMotor.setVelocity(-100, velocityUnits::rpm);
  GrabMotor.spinTo(armState[currentArmState], rotationUnits::deg, false);
}

void centerRobot() {
  bool buttonX = Controller1.ButtonX.pressing();
  if (buttonX == true) {
    LeftLiftMotor.stop();
    RightLiftMotor.stop();
    const float kp = 0.2;
    float speed = 2.0;
    float leftDistance = LeftRangeFinder.distance(distanceUnits::cm)/100;
    float rightDistance = RightRangeFinder.distance(distanceUnits::cm)/100;
    float effort = leftDistance - rightDistance;
    LeftLiftMotor.setVelocity(effort, velocityUnits::rpm);
    RightLiftMotor.setVelocity(-effort, velocityUnits::rpm);
  }
}

void handleGrab() {
  int armPosition = (armState[currentArmState] != armState[0]) ? 0 : ((armState[currentArmState] != armState[1] ? 1 : -1));
  if (armPosition != -1) {
    currentArmState = armPosition;
    GrabMotor.setVelocity(-100, velocityUnits::rpm);
    GrabMotor.spinTo(armState[currentArmState], rotationUnits::deg, false);
  }
}

bool checkGrabAuto() {
  bool retVal = false;

  Vision.takeSnapshot(Vision__PIZZABOX);
  if (Vision.objectCount > 0) retVal = true;

  Vision.takeSnapshot(Vision__GOLDENPIZZA);
  if (Vision.objectCount > 0) retVal = true;

  return retVal;
}

void handleRobotControl() {
  float straightSpeed = Controller1.Axis3.value();
  float turnSpeed = Controller1.Axis4.value();
  FrontLeftMotor.setVelocity(straightSpeed + turnSpeed, velocityUnits::rpm);
  FrontLeftMotor.spin(directionType::fwd);
  FrontRightMotor.setVelocity(straightSpeed - turnSpeed, velocityUnits::rpm);
  FrontRightMotor.spin(directionType::fwd);
  BackLeftMotor.setVelocity(straightSpeed + turnSpeed, velocityUnits::rpm);
  BackLeftMotor.spin(directionType::fwd);
  BackRightMotor.setVelocity(straightSpeed - turnSpeed, velocityUnits::rpm);
  BackRightMotor.spin(directionType::fwd);
}

bool checkRobotControl() {
  float straightSpeed = Controller1.Axis3.value();
  float turnSpeed = Controller1.Axis4.value();
  bool retVal = false;

  if (straightSpeed != 0 || turnSpeed != 0) retVal = true;

  return retVal;
}

void handleLiftControl() {
  LeftLiftMotor.setVelocity(-100, velocityUnits::rpm);
  RightLiftMotor.setVelocity(-100, velocityUnits::rpm);
  LeftLiftMotor.spinTo(liftState[currentLiftState], rotationUnits::deg, false);
  RightLiftMotor.spinTo(liftState[currentLiftState], rotationUnits::deg, false);
}

bool checkLiftControl() {
  int retVal = false;
  if (Controller1.Axis2.value() != 0 && liftActive == false) {
    if (Controller1.Axis2.value() > 0 && currentLiftState < 6 - 1) {
      currentLiftState++;
    }
    else if (Controller1.Axis2.value() < 0 && currentLiftState > 0) {
      currentLiftState--;
    }
    liftActive = true;
    retVal = true;
    cout << currentLiftState;
    cout << " \n";
  }
  else if (Controller1.Axis2.value() == 0 && liftActive == true) {
    liftActive = false;
  }
  return retVal;
}

bool checkGrabControl() {
  bool buttonA = Controller1.ButtonA.pressing();
  bool retVal = false;
  if (buttonA == true && buttonAPushed == false) {
    buttonAPushed = true;
    retVal = true;
  } else if (buttonA == false && buttonAPushed == true) {
    buttonAPushed = false;
  }
  return retVal;
}

void calibrateArm() {
  cout << LeftLiftMotor.position(rotationUnits::deg);
  cout << "Calibrate Arm\n";
  autoLift(3);
  wait(500, msec);
  while(LeftLiftMotor.velocity(velocityUnits::rpm) > 20) {}
  GrabMotor.setVelocity(100, velocityUnits::rpm);
  GrabMotor.spin(directionType::fwd);
  wait(500, msec);
  while(GrabMotor.velocity(velocityUnits::rpm) > 2) {}
  cout << "Arm reset\n";
  GrabMotor.resetRotation();
  GrabMotor.resetPosition();
  GrabMotor.setVelocity(0, velocityUnits::rpm);
  GrabMotor.spin(directionType::fwd);
  cout << "ending calibration\n";
  autoLift(0);
  autoGrab(1);
}

bool movementChecker() {
  return true;
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
  //calibrateArm();
  calibration = false;

  while (!complete) {
    if (movementChecker()) lineTracking(true);
  }
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
    
    if (calibration == false) {
      if (checkRobotControl()) handleRobotControl();
      if (checkLiftControl()) handleLiftControl();
      if (checkGrabControl()) handleGrab();
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