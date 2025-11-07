
#include "vex.h"
#include <cmath>
#include<iostream>
using namespace vex;

brain Brain;
controller Controller1 = controller(primary); 

//sensors
inertial InertialSensor(PORT3); 
// ---- Device Setup ----
motor BottomIntake = motor(PORT6, ratio18_1, false);
motor UpOrDown     = motor(PORT10, ratio18_1, false);
motor UpperIntake  = motor(PORT2, ratio18_1, false);

motor leftMotor1(PORT19, ratio18_1, true);
motor leftMotor2(PORT16, ratio18_1, true);
motor leftMotor3(PORT17, ratio18_1, true);
motor rightMotor1(PORT20, ratio18_1, false);
motor rightMotor2(PORT18, ratio18_1, false);
motor rightMotor3(PORT8,  ratio18_1, false);

motor_group leftMotors(leftMotor1, leftMotor2, leftMotor3);
motor_group rightMotors(rightMotor1, rightMotor2, rightMotor3);

digital_out doinker = digital_out(Brain.ThreeWirePort.B);
// --- PID constants ---
double kP = 60;
double kI = 10;
double kD = 20;

// --- Straight correction constant ---
double kTurn = 0.05; // how much to correct difference between sides

// --- Acceleration & Deceleration control ---
double maxAccel = 2;   // % increase per loop (20ms)
double maxDecel = 2;   // % decrease per loop (20ms)

// --- Global speed limit ---
double maxSpeedGlobal = 45;  // maximum % motor output (set lower to slow everything down)

//
double global_distance_scalar=0.211;



void eat(void){
  BottomIntake.spin(forward, 100, percent);
}
void scoretop(void){
  BottomIntake.spin(forward, 100, percent);
  UpperIntake.spin(forward, 100, percent);
  UpOrDown.spin(reverse, 100, percent);
}
void scorebottom(void){
  UpOrDown.spin(forward, 100, percent);
  BottomIntake.spin(forward, 100, percent);
  UpperIntake.spin(forward, 100, percent);
}
void spill(void){
  BottomIntake.spin(reverse, 100, percent);
  UpperIntake.spin(reverse, 100, percent);
}
void stopall(void){
  BottomIntake.stop(brake);
  UpperIntake.stop(brake);
  UpOrDown.stop(brake);
}

void drivePID(double targetDistanceInches, double maxSpeed = maxSpeedGlobal, int timeout = 9999999) {
  // Reset positions
  leftMotors.resetPosition();
  rightMotors.resetPosition();

  // Start a timer for timeout tracking
  int startTime = vex::timer::system();

  // Convert inches to degrees (assuming 4" diameter wheels)
  double targetRotDeg = (targetDistanceInches / (4 * M_PI)) * 360.0;

  double error = 0, prevError = 0, derivative = 0, integral = 0;
  double currentLeftSpeed = 0;
  double currentRightSpeed = 0;

  while (true) {
    // Average position
    double leftPos = leftMotors.position(degrees);
    double rightPos = rightMotors.position(degrees);
    double avgPos = (leftPos + rightPos) / 2.0;

    // --- Main distance PID ---
    error = targetRotDeg - avgPos;
    integral += error;
    derivative = error - prevError;
    prevError = error;

    double drivePower = (error * kP) + (integral * kI) + (derivative * kD);

    // --- Straight correction ---
    double sideError = leftPos - rightPos;
    double turnCorrection = sideError * kTurn;

    double targetLeftSpeed = drivePower - turnCorrection;
    double targetRightSpeed = drivePower + turnCorrection;

    // --- Bound speeds ---
    if (targetLeftSpeed > maxSpeed) targetLeftSpeed = maxSpeed;
    if (targetLeftSpeed < -maxSpeed) targetLeftSpeed = -maxSpeed;
    if (targetRightSpeed > maxSpeed) targetRightSpeed = maxSpeed;
    if (targetRightSpeed < -maxSpeed) targetRightSpeed = -maxSpeed;

    // --- Apply acceleration limits ---
    double leftDelta = targetLeftSpeed - currentLeftSpeed;
    double rightDelta = targetRightSpeed - currentRightSpeed;

    if (leftDelta > maxAccel) leftDelta = maxAccel;
    if (leftDelta < -maxDecel) leftDelta = -maxDecel;
    if (rightDelta > maxAccel) rightDelta = maxAccel;
    if (rightDelta < -maxDecel) rightDelta = -maxDecel;

    currentLeftSpeed += leftDelta;
    currentRightSpeed += rightDelta;

    // Apply to motors
    leftMotors.spin(forward, currentLeftSpeed, pct);
    rightMotors.spin(forward, currentRightSpeed, pct);

    // Exit condition: target reached OR timeout exceeded
    if (fabs(error) < 10) break; // within 10 degrees of target

    int elapsed = vex::timer::system() - startTime;
    if (elapsed > timeout) {
      Brain.Screen.printAt(10, 30, "drivePID TIMEOUT (%.2f in)", targetDistanceInches);
      break;
    }

    wait(20, msec);
  }

  leftMotors.stop(brake);
  rightMotors.stop(brake);
}


double turnThreshold = 1.0; // degrees tolerance
void turnPID(double targetDegrees) {
  // PID constants — tune these!
  // 0.7, 0.005, 4.5
  double kP = 0.63; 
  double kI = 0.005;
  double kD = 3.8;
  if (std::abs(targetDegrees)<145){
    kP=0.65;
    kD=3.8;
  }
  if (std::abs(targetDegrees)<100){
    kP=0.75;
    kD=5;
  }
  double error = 0;
  double previousError = 0;
  double derivative = 0;
  double integral = 0;

  double threshold = turnThreshold;
  int timeout = 500;     // max time (ms)
  int startTime = vex::timer::system();

  // Reset sensor
  InertialSensor.resetRotation();
  while (InertialSensor.isCalibrating()) {
    vex::wait(20, vex::msec);
  }

  // Target absolute angle (relative to current heading)
  double targetAngle = InertialSensor.rotation() + targetDegrees;

  while (true) {
    double currentAngle = InertialSensor.rotation();
    error = targetAngle - currentAngle;
    derivative = error - previousError;
    integral += error;

    // Prevent integral windup
    if (fabs(error) < 5) {
      integral = 0;
    }

    // PID output
    double turnPower = (kP * error) + (kI * integral) + (kD * derivative);

    // Cap power to safe limits
    if (turnPower > 100) turnPower = 100;
    if (turnPower < -100) turnPower = -100;

    // Apply turn power (opposite directions)
    leftMotors.spin(fwd, turnPower, pct);
    rightMotors.spin(reverse, turnPower, pct);

    // Exit when close enough or timed out
    if (fabs(error) < threshold || vex::timer::system() - startTime > timeout) {
      break;
    }

    previousError = error;
    vex::wait(20, vex::msec);
  }

  // Stop motors after done
  leftMotors.stop(brakeType::brake);
  rightMotors.stop(brakeType::brake);
}


void pid(double distance, int timeout=9999999){
  drivePID(distance*global_distance_scalar,maxSpeedGlobal, timeout);
}
void drive(double forward, double turn) {
  double leftSpeed = forward + turn*0.65;
  double rightSpeed = forward - turn*0.65;

  if (leftSpeed > 100) leftSpeed = 100;
  if (leftSpeed < -100) leftSpeed = -100;
  if (rightSpeed > 100) rightSpeed = 100;
  if (rightSpeed < -100) rightSpeed = -100;
  
  leftMotors.spin(directionType::fwd, leftSpeed, velocityUnits::pct);
  
  rightMotors.spin(directionType::fwd, rightSpeed, velocityUnits::pct);

}

void toggleDoinker() {
  static bool doinkerState = false;
  doinkerState = !doinkerState;
  doinker.set(doinkerState);
}

// ---- Exponential Curve ----
double expoCurve(double input, double exponent) {
  double normalized = input / 100.0; // -1..1
  double curved = pow(fabs(normalized), exponent) * (normalized >= 0 ? 1 : -1);
  return curved * 100.0; // back to -100..100
}

void usercontrol(void) {  
  bool buttonBPressed = false;
  bool upPressed = false;
  bool downPressed = false;

  // Adjustable variables
  double sensitivity = 0.8;   // overall drive power
  double expoForward = 1.5;   // forward/backward curve steepness
  double expoTurn = 1.3;      // turning curve steepness

  Brain.Screen.clearScreen();
  Brain.Screen.printAt(10, 20, "Starting driver control...");

  while (true) {
    // ---- Adjust sensitivity with up/down arrows ----
    if (Controller1.ButtonUp.pressing() && !upPressed) {
      sensitivity += 0.05;
      if (sensitivity > 1.2) sensitivity = 1.2;
      upPressed = true;
    } else if (!Controller1.ButtonUp.pressing()) upPressed = false;

    if (Controller1.ButtonDown.pressing() && !downPressed) {
      sensitivity -= 0.05;
      if (sensitivity < 0.4) sensitivity = 0.4;
      downPressed = true;
    } else if (!Controller1.ButtonDown.pressing()) downPressed = false;

    // ---- Adjust exponential steepness live ----
    if (Controller1.ButtonY.pressing()) expoForward += 0.05;
    if (Controller1.ButtonA.pressing()) expoForward -= 0.05;
    if (Controller1.ButtonX.pressing()) expoTurn += 0.05;
    if (Controller1.ButtonB.pressing()) expoTurn -= 0.05;

    // Clamp exponents
    if (expoForward < 1.0) expoForward = 1.0;
    if (expoForward > 3.0) expoForward = 3.0;
    if (expoTurn < 1.0) expoTurn = 1.0;
    if (expoTurn > 3.0) expoTurn = 3.0;

    // ---- Read controller joysticks ----
    double forward = expoCurve(Controller1.Axis3.position(), expoForward);
    double turn = expoCurve(Controller1.Axis1.position(), expoTurn);

    forward *= sensitivity;
    turn *= sensitivity * 0.40;

    // ---- Drive motors ----
    double deadband = 5;
    if (fabs(forward) < deadband && fabs(turn) < deadband) {
      leftMotors.stop(brake);
      rightMotors.stop(brake);
    } else {
      leftMotors.setVelocity(forward + turn, percent);
      rightMotors.setVelocity(forward - turn, percent);
      leftMotors.spin(fwd);
      rightMotors.spin(fwd);
    }

    // ---- Intake + Lift ----
    if (Controller1.ButtonR1.pressing()) eat();
    else if (Controller1.ButtonR2.pressing()) spill();
    else if (Controller1.ButtonL1.pressing()) scoretop();
    else if (Controller1.ButtonL2.pressing()) scorebottom();
    else stopall();

    // ---- Doinker toggle ----
    if (Controller1.ButtonB.pressing() && !buttonBPressed) {
      toggleDoinker();
      buttonBPressed = true;
    } else if (!Controller1.ButtonB.pressing()) {
      buttonBPressed = false;
    }

    // ---- Display Info ----
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(10, 20, "Sens: %.2f | F-Expo: %.2f | T-Expo: %.2f", sensitivity, expoForward, expoTurn);

    wait(20, msec);
  }
}


int main() {
  Brain.Screen.print("Calibrating Inertial...");
  InertialSensor.calibrate();
  while (InertialSensor.isCalibrating()) {
    wait(100, msec);
  }
  wait(500, msec);
  Brain.Screen.clearScreen();
  Brain.Screen.print("Calibration Done");

  // auton
  double time=Brain.timer(timeUnits::msec);
  int auton = 1;
  if (auton==1){
    //testing
    pid(50);
    turnPID(90);
    pid(50);
    turnPID(90);
    pid(50);
    turnPID(90);
    pid(50);
    turnPID(90);
  }
  else if (auton==2){
    // starting from right scoring bottom
    pid(17);
    turnPID(60);
    eat();
    maxSpeedGlobal=20;
    pid(50);
    wait(500,msec);
    maxSpeedGlobal=65;
    turnPID(-112);
    pid(25);
    stopall();
    spill();
    wait(2500,msec);
    stopall();
    spill();
    wait(2500,msec);
    stopall();
    pid(-10);
    pid(10);
    spill();
    wait(2500,msec);
    stopall();
  }
  else if (auton==3){
    // starting from left scoring middle
    pid(17);
    turnPID(-60);
    eat();
    maxSpeedGlobal=20;
    pid(50);
    wait(500,msec);
    maxSpeedGlobal=65;
    turnPID(112);
    pid(25);
    stopall();
    scorebottom();
    wait(2500,msec);
    stopall();
    scorebottom();
    wait(2500,msec);
    stopall();
    pid(-10);
    pid(10);
    scorebottom();
    wait(2500,msec);
    stopall();
  }
  std::cout<<"auton time: "<<Brain.timer(timeUnits::msec)-time<<std::endl;


  // driver control for testing
  usercontrol();
}
