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

//determines whether the auto functions are complete
bool autoComplete = false;
//variable for whether buttons are pushed
bool buttonAPushed = false;
bool buttonXPushed = false;
//variable for if the lifting mechanism is currently moving
bool liftActive = false;


const float lineTrackingKP = 0.1;
const int avgLineFollowVel = 20;
float prevLTError = 0;
bool lineTrackingFinished = false;

//lift state variables for the degrees for the motor to turn, the current lift state, and the state the lift motors are moving to
const float liftState[] = {27, 130, 250, 405, 540, 690};
int currentLiftState;
int transitionLiftState;
//arm state variables for the degrees for the motor to turn and the current arm state
const float armState[] = {-110, -240};
int currentArmState;

//stores the previous line tracker values
int prevRightLineVal;
int prevLeftLineVal;


/*enum {RAMP, WAITING, PIZZASEARCH, ATPIZZA, DORMSEARCH, APPROACHING, LIFTING, GRABBING, RELEASING, ATDORM};

int currentState = RAMP;*/

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

//fuction for using the line trackers
void lineTracking(bool darkLine) {
  while (lineTrackingFinished == false) {
    wait(20, msec);
    int curRightTrackerVal;
    int curLeftTrackerVal;
    //runs if the robot follows the white line
    if (darkLine == false) {
      curRightTrackerVal = 100-RightLineTracker.reflectivity();
      curLeftTrackerVal = 100-LeftLineTracker.reflectivity();
      //determines if the robot senses a white line on both sensors
      if (curRightTrackerVal < 80 && curLeftTrackerVal < 80) {
        lineTrackingFinished = true;
      }
    }
    //runs if the robot follows the black line
    else {
      curRightTrackerVal = RightLineTracker.reflectivity();
      curLeftTrackerVal = LeftLineTracker.reflectivity();
      //determines if the robot senses a black line on both sensors
      if (curRightTrackerVal < 10 && curLeftTrackerVal < 10) {
        lineTrackingFinished = true;
      }
    }
    //calculates the error and effort needed for the motors
    int error = curRightTrackerVal-curLeftTrackerVal;
    float effort = lineTrackingKP*error;
    prevLTError = error;
    float newVelRight = avgLineFollowVel - effort;
    float newVelLeft = avgLineFollowVel + effort;

    //if following the dark line, set the velocity to the kp velocities
    if (darkLine) {
      FrontRightMotor.setVelocity(newVelRight, velocityUnits::rpm);
      BackRightMotor.setVelocity(newVelRight, velocityUnits::rpm);
      FrontLeftMotor.setVelocity(newVelLeft, velocityUnits::rpm);
      BackLeftMotor.setVelocity(newVelLeft, velocityUnits::rpm);
    }
    //if following the white line, set the velocity to the average line following velocity
    else {
      FrontRightMotor.setVelocity(avgLineFollowVel, velocityUnits::rpm);
      BackRightMotor.setVelocity(avgLineFollowVel, velocityUnits::rpm);
      FrontLeftMotor.setVelocity(avgLineFollowVel, velocityUnits::rpm);
      BackLeftMotor.setVelocity(avgLineFollowVel, velocityUnits::rpm);
    }

    FrontRightMotor.spin(directionType::fwd);
    BackRightMotor.spin(directionType::fwd);
    FrontLeftMotor.spin(directionType::fwd);
    BackLeftMotor.spin(directionType::fwd);
  }
  lineTrackingFinished = false;
}

//uses the ultrasonic sensors to move a set distance away from the structure
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

//makes the robot travel straight for a set distance in inches
void autoStraight(float distanceToTravel, bool forward) {
  int direction = 1;
  if (forward == false) direction = -1;
  float revolutions = direction * distanceToTravel / (2 * 2 * M_PI);
  FrontLeftMotor.spinFor(revolutions, vex::rev, 50, rpm, false);
  FrontRightMotor.spinFor(revolutions, vex::rev, 50, rpm, false);
  BackLeftMotor.spinFor(revolutions, vex::rev, 50, rpm, false);
  BackRightMotor.spinFor(revolutions, vex::rev, 50, rpm, true);
  cout << "auto straight" << endl;
}

//makes the robot turn for an amount of degrees
void autoTurn(float turnDegrees, bool left) {
  autoStraight(1, false);
  wait(500, msec);
  int direction = 1;
  if (left == false) direction = -1;
  float revolutions = (direction*turnDegrees * 11.5) / (360*4);
  FrontLeftMotor.spinFor(-revolutions, rotationUnits::rev, 25, rpm, false);
  FrontRightMotor.spinFor(revolutions, rotationUnits::rev, 25, rpm, false);
  BackLeftMotor.spinFor(-revolutions, rotationUnits::rev, 25, rpm, false);
  BackRightMotor.spinFor(revolutions, rotationUnits::rev, 25, rpm, true);
}

//sets the lift mechanism to one of the preset levels
void autoLift(int level) {
  //int prevState = currentState;
  //currentState = LIFTING;
  currentLiftState = level;
  if (LiftMotors.velocity(velocityUnits::rpm) == 0) {
    LiftMotors.setVelocity(100, velocityUnits::rpm);
    LiftMotors.spinTo(liftState[currentLiftState], rotationUnits::deg, false);
  }
  //currentState = prevState;
}

//sets the arm mechanism to one of the preset levels
void autoGrab(float armPosition) {
  //int prevState = currentState;
  //currentState = (armPosition == 0) ? GRABBING : RELEASING;
  currentArmState = armPosition;
  GrabMotor.setVelocity(-100, velocityUnits::rpm);
  GrabMotor.spinTo(armState[currentArmState], rotationUnits::deg, false);
  //currentState = prevState;
}

//checks if the button for centering the robot is pressed
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

//centers the robot
void handleCenterRobot() {
  wait(1000, msec);
  //sets the lift to level 3 to give the sonar sensors a clear view
  autoLift(3);
  wait(2000, msec);
  const float kp = 0.4;
  float speed = 3;
  float leftDistance = LeftRangeFinder.distance(distanceUnits::mm);
  float rightDistance = RightRangeFinder.distance(distanceUnits::mm);
  float effort = leftDistance - rightDistance;
  //runs until the sonar sensor reads the same value
  while ((leftDistance != rightDistance || leftDistance == 0)) {
    wait(20, msec);
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
}

//check if the grabbing button is pressed
bool checkGrabControl() {
  bool buttonA = Controller1.ButtonA.pressing();
  bool retVal = false;
  if (buttonA == true && buttonAPushed == false) {
    buttonAPushed = true;
    retVal = true;
  }
  else if (buttonA == false && buttonAPushed == true) {
    buttonAPushed = false;
  }
  return retVal;
}

//changes grab state based on existing state
void handleGrab() {
  int armPosition = (armState[currentArmState] != armState[0]) ? 0 : ((armState[currentArmState] != armState[1] ? 1 : -1));
  if (armPosition != -1) {
    currentArmState = armPosition;
    GrabMotor.setVelocity(-100, velocityUnits::rpm);
    GrabMotor.spinTo(armState[currentArmState], rotationUnits::deg, false);
  }
}

//checks if the left controller joystick are being pushed
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

//controls robot based on controller input
void handleRobotControl() {
  float straightSpeed = Controller1.Axis3.value()/2;
  float turnSpeed = Controller1.Axis4.value()/4;
  //speeds up robot if the button is held
  if (Controller1.ButtonL1.pressing()) {
    straightSpeed *= 4;
  }

  if (straightSpeed < 0) straightSpeed /= 2;
  FrontLeftMotor.setVelocity(straightSpeed + turnSpeed, velocityUnits::rpm);
  FrontRightMotor.setVelocity(straightSpeed - turnSpeed, velocityUnits::rpm);
  BackLeftMotor.setVelocity(straightSpeed + turnSpeed, velocityUnits::rpm);
  BackRightMotor.setVelocity(straightSpeed - turnSpeed, velocityUnits::rpm);
  FrontRightMotor.spin(directionType::fwd);
  BackRightMotor.spin(directionType::fwd);
  FrontLeftMotor.spin(directionType::fwd);
  BackLeftMotor.spin(directionType::fwd);
}

//check if the lift has moved to the appropriate level
bool checkLiftControl() {
  currentLiftState = LiftMotors.position(rotationUnits::deg);
  int retVal = false;
  if (Controller1.Axis2.value() != 0 && LiftMotors.isDone() == true) {
    retVal = true;
  }
  else if (currentLiftState >= liftState[transitionLiftState]-5 && currentLiftState <= liftState[transitionLiftState]+5) {}
  return retVal;
}

//raises and lowers the state of the lift mechansim
void handleLiftControl() {
  int direction = Controller1.Axis2.value();
  if (transitionLiftState < 5 && direction > 0) {
    transitionLiftState++;
  }
  else if (transitionLiftState > 0 && direction < 0) {
    transitionLiftState--;
  }
  LiftMotors.setVelocity(100, velocityUnits::rpm);
  LiftMotors.spinTo(liftState[transitionLiftState], rotationUnits::deg, false);
}

//calibrates the arm at the beginning of startup
void calibrateArm() {
  autoLift(2);
  while (!LiftMotors.isDone()) {}
  GrabMotor.setVelocity(100, velocityUnits::rpm);
  GrabMotor.spin(directionType::fwd);
  wait(500, msec);
  while(GrabMotor.velocity(velocityUnits::rpm) > 2) {
    wait(20, msec);
  }
  GrabMotor.setVelocity(0, velocityUnits::rpm);
  GrabMotor.resetRotation();
  GrabMotor.resetPosition();
  autoGrab(1);
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

//main demo day code that drives up the ramp, places pizzas on the first, second, and top floor of messenger and then grabs a fourth pizza to go into
//the construction zone to deliver a pizza on the ground floor of faraday
void demoDay() {
  calibrateArm();

  //Replace these values if testing on the opposite side of the ramp
  bool normalTrue = true;
  bool normalFalse = false;

  wait(1000, msec);
  
  autoStraight(20, true);
  autoLift(1);
  lineTracking(true);
  lineTracking(false);
  autoTurn(90, normalFalse);

  //lifting level normally starts at 2, 6 for testing over speed bump
  for (int liftingLevel = 2; liftingLevel < 7; liftingLevel++) {
    if (liftingLevel == 4) {
      liftingLevel = 5;
    }
    //move towards pizzaria
    wait(1000, msec);
    autoLift(3);
    lineTracking(false);
    //turn into pizzaria
      //for testing far side
      //autoStraight(2, true);
      //for testing ramp side
      autoStraight(1, true);
    wait(1000, msec);
    autoTurn(90, normalFalse);
    //move to get pizza
    autoStraight(7, true);
    wait(1000, msec);
    autoGrab(0);
    wait(1000, msec);
    //move backwards with pizza
    autoStraight(5 + liftingLevel - 2, false);
    wait(2000, msec);
    autoTurn(90, normalFalse);
    //move forwards towards dorms
    autoLift(1);
    lineTracking(false);
    autoStraight(35, true);
    if (liftingLevel < 6) {
      //turn to dorm and adjust
      autoTurn(90, normalFalse);
      autoStraight(7, false);
      wait(2000, msec);
      handleCenterRobot();
      wait(2000, msec);
      autoLift(liftingLevel);
      wait(1000, msec);
      //move arm into dorm
      if (liftingLevel == 2) {
        moveToDistance(9);
      }
      else if (liftingLevel == 3) {
        moveToDistance(9.5);
      }
      else if (liftingLevel == 5) {
        moveToDistance(5);
      }
      autoGrab(1);
      //retreat from dorm with pizza delivered
      autoStraight(15, false);
      autoLift(1);
      wait(1000, msec);
      lineTracking(false);
      autoTurn(90, normalFalse);
    }
    else {
      lineTracking(false);
      autoLift(1);
      autoTurn(90, normalFalse);
      //move over speed bump
      autoStraight(30, true);
      lineTracking(false);
      //move towards inside building
      autoStraight(12, false);
      //align robot to smaller dorm room
      autoTurn(90, normalFalse);
      handleCenterRobot();
      //deposit pizza
      wait(1000, msec);
      autoLift(1);
      wait(1000, msec);
      autoStraight(10, true);
      autoGrab(1);
    }
  }
}

//grabs a pizza from the pizzaria to put on the first floor of messenger
void demoDayExtraFunctionality() {
  //Replace these values if testing on the opposite side of the ramp
  bool normalTrue = true;
  bool normalFalse = false;
  autoLift(3);
  autoTurn(90, true);
  //move to get pizza
  autoStraight(7, true);
  wait(1000, msec);
  autoGrab(0);
  wait(1000, msec);
  //move backwards with pizza
  autoStraight(5, false);
  wait(2000, msec);
  autoTurn(90, normalFalse);
  cout << "should turn" << endl;
  //move forwards towards dorms
  autoLift(1);
  lineTracking(false);
  autoStraight(35, true);
  //turn to dorm and adjust
  autoTurn(90, normalFalse);
  autoStraight(7, false);
  wait(2000, msec);
  handleCenterRobot();
  wait(2000, msec);
  autoLift(2);
  wait(1000, msec);
  //move arm into dorm
  moveToDistance(9);
  autoGrab(1);
  //retreat from dorm with pizza delivered
  autoStraight(15, false);
  autoLift(1);
  wait(1000, msec);
  lineTracking(false);
  autoTurn(90, normalFalse);
}

//grabs a pizza from the pizzaria and then goes into the construction zone to put a pizza on the first floor of faraday
void competitionDay() {
  //These values are based on being on ramp side
  bool normalTrue = true;
  bool normalFalse = false;
  autoLift(3);
  autoTurn(90, normalTrue);
  if (LeftRangeFinder.distance(distanceUnits::in) > 30) {
    normalTrue = false;
    normalFalse = true;
    autoTurn(180, false);
  }
  //move to get pizza
  autoStraight(7, true);
  wait(1000, msec);
  autoGrab(0);
  wait(1000, msec);
  //move backwards with pizza
  autoStraight(5, false);
  wait(2000, msec);
  autoTurn(90, normalFalse);
  //move towards speed bump
  autoLift(1);
  lineTracking(false);
  autoStraight(45, true);
  lineTracking(false);
  //turn towards the bump
  autoLift(1);
  autoTurn(90, normalFalse);
  //move over speed bump
  autoStraight(30, true);
  lineTracking(false);
  //move towards inside building
  autoStraight(12, false);
  //align robot to smaller dorm room
  autoTurn(90, normalFalse);
  autoStraight(4, false);
  handleCenterRobot();
  //deposit pizza
  wait(1000, msec);
  autoLift(3);
  wait(1000, msec);
  autoStraight(12, true);
  autoGrab(1);
  autoStraight(6, true);
  autoLift(0);
}

//performs the autonomous code
void autonomous(void) {

  autoComplete = true;
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
    
    if (autoComplete == true) {
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