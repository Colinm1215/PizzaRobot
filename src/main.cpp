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

#include "vex.h"
#include <cmath>
#include <iostream>

using namespace vex;
using namespace std;

// A global instance of competition
competition Competition;

bool autoComplete = true;
bool buttonAPushed = false;
bool liftActive = false;
bool buttonXPushed = false;

const float lineTrackingKP = 0.1;
const int avgLineFollowVel = 20;
const float lineTrackingKD = 0.05;
float prevLTError = 0;
bool lineTrackingFinished = false;

const float liftState[] = {27, 130, 250, 405, 540, 690, 360};
int currentLiftState;
int transitionLiftState;
const float armState[] = {-110, -240};
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

bool wallClose() {
    task::sleep(100);
    float rightDist = RightRangeFinder.distance(distanceUnits::cm);
    float leftDist = LeftRangeFinder.distance(distanceUnits::cm);
    float avgDist = 0;
    if (rightDist > 0) avgDist += rightDist;
    if (leftDist > 0) avgDist += leftDist;
    avgDist /= 2.0;
    return (avgDist <= 25.0);
}

void moveBackToLine() {
  bool atLine = false;
  int curRightTrackerVal = 100-RightLineTracker.reflectivity();
  int curLeftTrackerVal = 100-LeftLineTracker.reflectivity();
  if (curRightTrackerVal < 80 && curLeftTrackerVal < 80) {
    atLine = true;
  }

}

void lineTracking(bool darkLine) {
  while (lineTrackingFinished == false) {
    wait(20, msec);
    int curRightTrackerVal;
    int curLeftTrackerVal;
    if (darkLine == false) {
      curRightTrackerVal = 100-RightLineTracker.reflectivity();
      curLeftTrackerVal = 100-LeftLineTracker.reflectivity();
      if (curRightTrackerVal < 80 && curLeftTrackerVal < 80) {
        lineTrackingFinished = true;
      FrontLeftMotor.stop();
      FrontRightMotor.stop();
      BackLeftMotor.stop();
      BackRightMotor.stop();
      }
    }
    else {
      curRightTrackerVal = RightLineTracker.reflectivity();
      curLeftTrackerVal = LeftLineTracker.reflectivity();
      if (curRightTrackerVal < 10 && curLeftTrackerVal < 10) {
        lineTrackingFinished = true;
      FrontLeftMotor.stop();
      FrontRightMotor.stop();
      BackLeftMotor.stop();
      BackRightMotor.stop();
      }
    }
    if (!lineTrackingFinished) {
    int error = curRightTrackerVal-curLeftTrackerVal;
    float effort = lineTrackingKP*error;
    prevLTError =  effort;
    float newVelRight = avgLineFollowVel - effort;
    float newVelLeft = avgLineFollowVel + effort;
    
    cout << curRightTrackerVal << " RIGHT \n";
    cout << curLeftTrackerVal << " LEFT \n";
   // Brain.Screen.clearScreen();
    /* Brain.Screen.print(curRightTrackerVal);
    Brain.Screen.print(" RIGHT ");
    Brain.Screen.newLine();
    Brain.Screen.print(curLeftTrackerVal);
    Brain.Screen.print(" LEFT ");
    Brain.Screen.newLine(); */

     Brain.Screen.clearLine(1,color::black);
    Brain.Screen.clearLine(2,color::black);
        Brain.Screen.setCursor(1,0);
        Brain.Screen.print("RIGHT %d",curRightTrackerVal);
        Brain.Screen.setCursor(2,0);
        Brain.Screen.print("LEFT %d",curLeftTrackerVal);
    Brain.Screen.render();

    FrontRightMotor.setVelocity(newVelRight, velocityUnits::rpm);
    BackRightMotor.setVelocity(newVelRight, velocityUnits::rpm);
    FrontLeftMotor.setVelocity(newVelLeft, velocityUnits::rpm);
    BackLeftMotor.setVelocity(newVelLeft, velocityUnits::rpm);
    /* FrontRightMotor.setVelocity(newVelRight, velocityUnits::rpm);
    BackRightMotor.setVelocity(avgLineFollowVel, velocityUnits::rpm);
    FrontLeftMotor.setVelocity(avgLineFollowVel, velocityUnits::rpm);
    BackLeftMotor.setVelocity(avgLineFollowVel, velocityUnits::rpm); */

    FrontRightMotor.spin(directionType::fwd);
    BackRightMotor.spin(directionType::fwd);
    FrontLeftMotor.spin(directionType::fwd);
    BackLeftMotor.spin(directionType::fwd);
    }
    if (wallClose() && currentState == PIZZASEARCH && currentLiftState > 1) {
      lineTrackingFinished = true;
      FrontLeftMotor.stop();
      FrontRightMotor.stop();
      BackLeftMotor.stop();
      BackRightMotor.stop();
    }
  }
  lineTrackingFinished = false;
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

void autoStraight(float distanceToTravel, bool forward) {
  int direction = 1;
  if (forward == false) direction = -1;
  float revolutions = direction * distanceToTravel / (2 * 2 * M_PI);
  FrontLeftMotor.spinFor(revolutions, vex::rev, 50, rpm, false);
  FrontRightMotor.spinFor(revolutions, vex::rev, 50, rpm, false);
  BackLeftMotor.spinFor(revolutions, vex::rev, 50, rpm, false);
  BackRightMotor.spinFor(revolutions, vex::rev, 50, rpm, true);
}

void autoTurn(float turnDegrees, bool left) {
  autoStraight(1, false);
  wait(500, msec);
  int direction = 1;
  if (left == false) direction = -1;
  //11.5 is wheel diameter
  float revolutions = (direction*turnDegrees * 11.5) / (360*4);
  FrontLeftMotor.spinFor(-revolutions, rotationUnits::rev, 25, rpm, false);
  FrontRightMotor.spinFor(revolutions, rotationUnits::rev, 25, rpm, false);
  BackLeftMotor.spinFor(-revolutions, rotationUnits::rev, 25, rpm,  false);
  BackRightMotor.spinFor(revolutions, rotationUnits::rev, 25, rpm,  true);
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

bool checkCenterRobot() {
  bool retVal = false;
  if (Controller1.ButtonX.pressing() == true && buttonXPushed == false) {
    retVal = true;
  }
  else if (Controller1.ButtonX.pressing() == false && buttonXPushed == true) {
    buttonXPushed = false;
  }
  return retVal;
}

void handleCenterRobot() {
  wait(1000, msec);
  autoLift(3);
  wait(2000, msec);
  const float kp = 0.4;
  float speed = 3;
  float leftDistance = LeftRangeFinder.distance(distanceUnits::mm);
  float rightDistance = RightRangeFinder.distance(distanceUnits::mm);
  cout << "left sensor distance " << leftDistance << endl;
  cout << "right sensor distance " << rightDistance << endl;
  float effort = leftDistance - rightDistance;
  
  while ((leftDistance != rightDistance || leftDistance == 0)) {
    wait(20, msec);
    cout << "left sensor distance " << leftDistance << endl;
    cout << "right sensor distance " << rightDistance << endl;
    leftDistance = LeftRangeFinder.distance(distanceUnits::mm);
    rightDistance = RightRangeFinder.distance(distanceUnits::mm);
    effort = kp * (leftDistance - rightDistance);
    if (effort < 30) {
      float leftSpeed = speed + effort;
      float rightSpeed = speed - effort;
      FrontLeftMotor.setVelocity(leftSpeed, velocityUnits::rpm);
      BackLeftMotor.setVelocity(leftSpeed, velocityUnits::rpm);
      FrontRightMotor.setVelocity(rightSpeed, velocityUnits::rpm);
      BackRightMotor.setVelocity(rightSpeed, velocityUnits::rpm);
      FrontRightMotor.spin(directionType::fwd);
      BackRightMotor.spin(directionType::fwd);
      FrontLeftMotor.spin(directionType::fwd);
      BackLeftMotor.spin(directionType::fwd);
    }
  }
  FrontLeftMotor.setVelocity(0, velocityUnits::rpm);
  BackLeftMotor.setVelocity(0, velocityUnits::rpm);
  FrontRightMotor.setVelocity(0, velocityUnits::rpm);
  BackRightMotor.setVelocity(0, velocityUnits::rpm);
  FrontRightMotor.spin(directionType::fwd);
  BackRightMotor.spin(directionType::fwd);
  FrontLeftMotor.spin(directionType::fwd);
  BackLeftMotor.spin(directionType::fwd);
  cout << "left sensor distance " << leftDistance << endl;
  cout << "right sensor distance " << rightDistance << endl;
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
  float straightSpeed = Controller1.Axis3.value()/2;
  float turnSpeed = Controller1.Axis4.value()/4;

  if (straightSpeed < 0) straightSpeed /= 2;
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
  
  if (Controller1.Axis2.value() != 0 && LiftMotors.isDone() == true) {
    retVal = true;
    cout << "check lift" << endl;
  }
  else if (currentLiftState >= liftState[transitionLiftState]-5 && currentLiftState <= liftState[transitionLiftState]+5) {}
  else {
    
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

void moveToDistance(float distanceToSet) {
  float leftDistance = round(LeftRangeFinder.distance(distanceUnits::in));
  float rightDistance = round(RightRangeFinder.distance(distanceUnits::in));
  FrontLeftMotor.setVelocity(0, velocityUnits::rpm);
  BackLeftMotor.setVelocity(0, velocityUnits::rpm);
  FrontRightMotor.setVelocity(0, velocityUnits::rpm);
  BackRightMotor.setVelocity(0, velocityUnits::rpm);
  FrontRightMotor.spin(directionType::fwd);
  BackRightMotor.spin(directionType::fwd);
  FrontLeftMotor.spin(directionType::fwd);
  BackLeftMotor.spin(directionType::fwd);

  while(leftDistance > distanceToSet && rightDistance > distanceToSet) {
    leftDistance = round(LeftRangeFinder.distance(distanceUnits::in));
    rightDistance = round(RightRangeFinder.distance(distanceUnits::in));
    FrontLeftMotor.setVelocity(20, velocityUnits::rpm);
    BackLeftMotor.setVelocity(20, velocityUnits::rpm);
    FrontRightMotor.setVelocity(20, velocityUnits::rpm);
    BackRightMotor.setVelocity(20, velocityUnits::rpm);
  }
  while(leftDistance < distanceToSet && rightDistance < distanceToSet) {
    leftDistance = round(LeftRangeFinder.distance(distanceUnits::mm));
    rightDistance = round(RightRangeFinder.distance(distanceUnits::mm));
    FrontLeftMotor.setVelocity(-20, velocityUnits::rpm);
    BackLeftMotor.setVelocity(-20, velocityUnits::rpm);
    FrontRightMotor.setVelocity(-20, velocityUnits::rpm);
    BackRightMotor.setVelocity(-20, velocityUnits::rpm);
  }

  FrontLeftMotor.setVelocity(0, velocityUnits::rpm);
  BackLeftMotor.setVelocity(0, velocityUnits::rpm);
  FrontRightMotor.setVelocity(0, velocityUnits::rpm);
  BackRightMotor.setVelocity(0, velocityUnits::rpm);
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
  autoGrab(0);
  while (!GrabMotor.isDone()) {}
  autoGrab(1);
  while (!GrabMotor.isDone()) {}
  autoLift(3);
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
  
  cout.flush();
  calibrateArm();
  cout << "auto\n";
  //currentState = PIZZASEARCH;

  //Replace these values if testing on the other side
  bool normalTrue = true;
  bool normalFalse = false;

  wait(1000, msec);
  
  autoStraight(20, true);
  autoLift(1);
  lineTracking(true);
  autoLift(1);
  lineTracking(false);
  autoTurn(90, normalFalse);

  //lifting level normally starts at 2, 6 for testing over speed bump
  for (int liftingLevel = 2; liftingLevel < 7; liftingLevel++) {
    if (liftingLevel == 4) {
      liftingLevel = 5;
    }
    //move towards pizzaria
    wait(500, msec);
    lineTracking(false);
    //turn into pizzaria
      //for testing far side
      //autoStraight(2, true);
      //for testing ramp side
      //autoStraight(1, true);
    autoTurn(90, normalFalse);
    currentState = PIZZASEARCH;
    //move to get pizza
    handleCenterRobot();
    wait(500, msec);
    autoLift(3);
    while (!LiftMotors.isDone()) {}
    autoStraight(7, true);
    wait(5000, msec);
    autoGrab(0);
    while (!GrabMotor.isDone()) {}
    //move backwards with pizza
    autoStraight(7, false);
    wait(500, msec);
    autoLift(1);
    while (!LiftMotors.isDone()) {}
    autoTurn(90, normalFalse);
    wait(500,msec);
    cout << "should turn" << endl;
    //move forwards towards dorms
    //autoLift(1);
    //lineTracking(false);
    autoStraight(35, true);
    if (liftingLevel < 6) {
      //turn to dorm and adjust
      autoTurn(90, normalFalse);
      autoStraight(7, false);
      wait(500, msec);
      autoLift(3);
      while (!LiftMotors.isDone()) {}
      handleCenterRobot();
      wait(500, msec);
      autoLift(liftingLevel);
      while (!LiftMotors.isDone()) {}
      //move arm into dorm
      //moveToDistance(8);
      //autoStraight(8, true);
      autoStraight(9, true);
      if (liftingLevel > 3) autoStraight(2, true);
      wait(500, msec);
      autoGrab(1);
      while (!GrabMotor.isDone()) {}
      //retreat from dorm with pizza delivered
      autoStraight(10, false);
      wait(500, msec);
      autoLift(1);
      while (!LiftMotors.isDone()) {}
      lineTracking(false);
      autoTurn(90, normalFalse);
    }
    else {
      lineTracking(false);
      autoLift(1);
      while (!LiftMotors.isDone()) {}
      autoTurn(90, normalFalse);
      //move over speed bump
      autoStraight(40, true);
      lineTracking(false);
      //move towards inside building
      autoStraight(12, false);
      //align robot to smaller dorm room
      autoTurn(90, normalFalse);
      handleCenterRobot();
      //deposit pizza
      wait(1000, msec);
      autoLift(1);
      while (!LiftMotors.isDone()) {}
      autoStraight(10, true);
      autoGrab(1);
    }
  }
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
  //This will run the auto function but it will end when any controller button is pressed
  autonomous();
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
      if (checkCenterRobot()) handleCenterRobot();
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