// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// FrontLeftMotor       motor         1               
// FrontRightMotor      motor         2               
// BackLeftMotor        motor         3               
// BackRightMotor       motor         4               
// GrabMotor            motor         7               
// LeftRangeFinder      sonar         A, B            
// RightRangeFinder     sonar         C, D            
// LeftLineTracker      line          E               
// RightLineTracker     line          F               
// Vision               vision        8               
// LiftMotors           motor_group   5, 6            
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// FrontLeftMotor       motor         1               
// FrontRightMotor      motor         2               
// BackLeftMotor        motor         3               
// BackRightMotor       motor         4               
// GrabMotor            motor         7               
// LeftRangeFinder      sonar         A, B            
// RightRangeFinder     sonar         C, D            
// LeftLineTracker      line          E               
// RightLineTracker     line          F               
// Vision               vision        8               
// LiftMotors          motor_group   5, 6            
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <iostream>

using namespace vex;
using namespace std;

// A global instance of competition
competition Competition;

//sonar LeftRangeFinder = sonar(Brain.ThreeWirePort.A);
//sonar RightRangeFinder = sonar(Brain.ThreeWirePort.C);

bool autoComplete = true;
bool buttonAPushed = false;
bool liftActive = false;

const float lineTrackingKP = 0.5;
const int avgLineFollowVel = 20;
const float lineTrackingKD = 0.05;
float prevLTError = 0;

const float liftState[] = {27, 80, 250, 400, 540, 690};
int currentLiftState;
int transitionLiftState;
const float armState[] = {-110, -280};
int currentArmState;

int prevRightLineVal;
int prevLeftLineVal;

enum {RAMP, WAITING, PIZZASEARCH, ATPIZZA, DORMSEARCH, APPROACHING, LIFTING, GRABBING, RELEASING, ATDORM};

int currentDorm = 0;

int currentState = RAMP;

bool reverseMovement = false;

bool hasPizza = false;

bool complete = false;

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

void lineTracking(bool darkLine) {
  wait(0.5, sec);
  int curRightTrackerVal;
  int curLeftTrackerVal;
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
  prevLTError = error;
  float newVelRight = avgLineFollowVel + effort;
  float newVelLeft = avgLineFollowVel - effort;

  cout << curRightTrackerVal << " RIGHT \n";
  cout << curLeftTrackerVal << " LEFT \n";

  FrontRightMotor.setVelocity(newVelRight, velocityUnits::rpm);
  BackRightMotor.setVelocity(newVelRight, velocityUnits::rpm);
  FrontLeftMotor.setVelocity(newVelLeft, velocityUnits::rpm);
  BackLeftMotor.setVelocity(newVelLeft, velocityUnits::rpm);

  FrontRightMotor.spin(directionType::fwd);
  BackRightMotor.spin(directionType::fwd);
  FrontLeftMotor.spin(directionType::fwd);
  BackLeftMotor.spin(directionType::fwd);
}

bool checkLineTrack() {
  bool retVal = false;
  if (currentState == PIZZASEARCH || currentState == RAMP) {
    retVal = true;
  }
  return retVal;
}

void handleLineTrack() {
  if (currentState == PIZZASEARCH) lineTracking(true);
  else if (currentState == RAMP) lineTracking(false);
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
  int prevState = currentState;
  currentState = LIFTING;
  currentLiftState = level;
  if (LiftMotors.velocity(velocityUnits::rpm) == 0) {
    LiftMotors.setVelocity(100, velocityUnits::rpm);
    LiftMotors.spinTo(liftState[currentLiftState], rotationUnits::deg, false);
  }
  currentState = prevState;
}

void autoGrab(float armPosition) {
  int prevState = currentState;
  currentState = (armPosition == 0) ? GRABBING : RELEASING;
  currentArmState = armPosition;
  GrabMotor.setVelocity(-100, velocityUnits::rpm);
  GrabMotor.spinTo(armState[currentArmState], rotationUnits::deg, false);
  currentState = prevState;
}

void centerRobot() {
  bool buttonX = Controller1.ButtonX.pressing();
  if (buttonX == true) {
    LiftMotors.stop();
    const float kp = 0.2;
    float speed = 2.0;
    float leftDistance = LeftRangeFinder.distance(distanceUnits::cm)/100;
    float rightDistance = RightRangeFinder.distance(distanceUnits::cm)/100;
    float effort = leftDistance - rightDistance;
    LiftMotors.setVelocity(effort, velocityUnits::rpm);
    //RightLiftMotor.setVelocity(-effort, velocityUnits::rpm);
  }
}

void handleGrab() {
  int armPosition = (armState[currentArmState] != armState[0]) ? 0 : ((armState[currentArmState] != armState[1] ? 1 : -1));
  if (armPosition != -1) {
    currentArmState = armPosition;
    GrabMotor.setVelocity(-100, velocityUnits::rpm);
    GrabMotor.spinTo(armState[currentArmState], rotationUnits::deg, false);
    complete = true;
  }
}

void handleGrabAuto() {
  int armPosition = (armState[currentArmState] != armState[0]) ? 0 : ((armState[currentArmState] != armState[1] ? 1 : -1));
  if (armPosition != -1) {
    if (armPosition == 0) task::sleep(500);
    currentArmState = armPosition;
    GrabMotor.setVelocity(-100, velocityUnits::rpm);
    GrabMotor.spinTo(armState[currentArmState], rotationUnits::deg, false);
    complete = true;
  }
}

bool checkGrabAutoCam() {
  bool retVal = false;

  //cout << "Looking for PIZZA" << endl;

  if (currentState == PIZZASEARCH) {
  Vision.takeSnapshot(Vision__PIZZABOX);
  if (Vision.objectCount > 0) {
    if (Vision.largestObject.centerY >= 105) retVal = true;
  }
  } else if (currentState == ATDORM) {
    retVal = true;
  }

  complete = retVal;
  return retVal;
}

bool checkGrabAuto() {
  bool retVal =  false;

  if (currentState == ATDORM || currentState == ATPIZZA) retVal = true;

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
  else {
  FrontLeftMotor.setVelocity(0, velocityUnits::rpm);
  FrontRightMotor.setVelocity(0, velocityUnits::rpm);
  BackLeftMotor.setVelocity(0, velocityUnits::rpm);
  BackRightMotor.setVelocity(0, velocityUnits::rpm);
  }

  return retVal;
}

void handleLiftControl() {
  /*int vel = Controller1.Axis2.value();
  LiftMotors.setVelocity(vel, percentUnits::pct);
  //RightLiftMotor.setVelocity(-100, velocityUnits::rpm);
  LiftMotors.spin(directionType::fwd);
  //RightLiftMotor.spinTo(liftState[currentLiftState], rotationUnits::deg, false);*/
  int direction = Controller1.Axis2.value();
  cout << "checking" << endl;
  if (transitionLiftState < 5 && direction > 0) {
    cout << "up" << endl;
    transitionLiftState++;
  }
  else if (transitionLiftState > 0 && direction < 0) {
    transitionLiftState--;
    cout << "down" << endl;
  }
  LiftMotors.setVelocity(100, velocityUnits::rpm);
  LiftMotors.spinTo(liftState[transitionLiftState], rotationUnits::deg, false);
}

bool checkLiftControl() {
  currentLiftState = LiftMotors.position(rotationUnits::deg);
  int retVal = false;
  if (Controller1.Axis2.value() != 0 && currentLiftState >= liftState[transitionLiftState]-5 && currentLiftState <= liftState[transitionLiftState]+5) {
    retVal = true;

  }
  else if (currentLiftState >= liftState[transitionLiftState]-5 && currentLiftState <= liftState[transitionLiftState]+5) {}
  else {
    LiftMotors.setVelocity(0, percentUnits::pct);
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
  cout << LiftMotors.position(rotationUnits::deg) << endl;
  cout << "Calibrate Arm\n";
  /* autoLift(3);
  wait(500, msec);
  while(LiftMotors.velocity(velocityUnits::rpm) > 20) {}
  autoGrab(0);
  wait(500, msec);
  while(GrabMotor.velocity(velocityUnits::rpm) > 2) {}
  cout << "Arm reset\n";
  GrabMotor.resetRotation();
  GrabMotor.resetPosition();
  GrabMotor.setVelocity(0, velocityUnits::rpm);
  GrabMotor.spin(directionType::fwd);
  cout << "ending autoComplete\n";
  autoLift(0);
  autoGrab(1); */
  autoLift(2);
  while (!LiftMotors.isDone()) {}
  cout << "step 1" << endl;
  GrabMotor.setVelocity(100, velocityUnits::rpm);
  GrabMotor.spin(directionType::fwd);
  wait(500, msec);
  while(GrabMotor.velocity(velocityUnits::rpm) > 2) {
    cout << GrabMotor.velocity(velocityUnits::rpm) << "\n";
    wait(20, msec);
  }
  cout << "ended\n";
  GrabMotor.setVelocity(0, velocityUnits::rpm);
  GrabMotor.resetRotation();
  GrabMotor.resetPosition();
  autoGrab(1);
  while (!GrabMotor.isDone()) {}
  cout << "step 2" << endl;
  autoGrab(0);
  while (!GrabMotor.isDone()) {}
  cout << "step 3" << endl;
  autoGrab(1);
  while (!GrabMotor.isDone()) {}
  cout << "fin" << endl;

  autoLift(1);
  while (!LiftMotors.isDone()) {}
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
  calibrateArm();
  cout << "auto\n";
  //currentState = PIZZASEARCH;

  while (true) {
    lineTracking(false);
  }
  wait(3000, msec);
  autoLift(2);
  transitionLiftState = 2;
  autoComplete = false;
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
    
    if (autoComplete == false) {
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