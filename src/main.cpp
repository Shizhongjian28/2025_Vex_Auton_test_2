#include "vex.h"
#include "iostream"
using namespace vex;

// --- Devices (adjust ports as needed) ---
brain Brain;

//sensors
inertial InertialSensor(PORT3); // adjust port as needed


// Intake motors
motor BottomIntake = motor(PORT6, ratio18_1, false);
motor UpOrDown     = motor(PORT10, ratio18_1, false);
motor UpperIntake  = motor(PORT2, ratio18_1, false);

// Driver motors
motor leftMotor1(PORT19, ratio18_1, true);
motor leftMotor2(PORT16, ratio18_1, true);
motor leftMotor3(PORT17, ratio18_1, true);
motor rightMotor1(PORT20, ratio18_1, false);
motor rightMotor2(PORT18, ratio18_1, false);
motor rightMotor3(PORT8,  ratio18_1, false);

// Motor groups
motor_group leftMotors(leftMotor1, leftMotor2, leftMotor3);
motor_group rightMotors(rightMotor1, rightMotor2, rightMotor3);

// --- PID constants ---
double kP = 20;
double kI = 0.1;
double kD = 10;

// --- Straight correction constant ---
double kTurn = 0.05; // how much to correct difference between sides

// --- Acceleration & Deceleration control ---
double maxAccel = 1;   // % increase per loop (20ms)
double maxDecel = 0.5;   // % decrease per loop (20ms)

// --- Global speed limit ---
double maxSpeedGlobal = 30;  // maximum % motor output (set lower to slow everything down)

//
double global_distance_scalar=0.211;


void drivePID(double targetDistanceInches, double maxSpeed = maxSpeedGlobal) {
  // Reset positions
  leftMotors.resetPosition();
  rightMotors.resetPosition();

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

    // Exit condition
    if (fabs(error) < 10) break; // within 10 degrees of target
    wait(20, msec);
  }

  leftMotors.stop(brake);
  rightMotors.stop(brake);
}
void turnPID(double targetDegrees) {
  // PID constants — tune these!
  double kP = 0.5; 
  double kI = 0.001;
  double kD = 0.2;

  double error = 0;
  double previousError = 0;
  double derivative = 0;
  double integral = 0;

  double threshold = 1.0; // degrees tolerance
  int timeout = 3000;     // max time (ms)
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


void pid(double distance){
  drivePID(distance*global_distance_scalar,maxSpeedGlobal);
}

// user driving control
controller Controller1 = controller(primary);

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

void usercontrol(void) {  
  while(true){
    leftMotors.setVelocity(Controller1.Axis3.position()+Controller1.Axis1.position()*0.8, percent);
    rightMotors.setVelocity(Controller1.Axis3.position()-Controller1.Axis1.position()*0.8, percent);
    leftMotors.spin(forward);
    rightMotors.spin(forward);
    wait(5, msec);
  }
}

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
  int auton = 3;
  if (auton==1){
    eat();
    pid(100);
    turnPID(10);
    wait(1000, msec);
    turnPID(-10);
    pid(-30);
    stopall();
    turnPID(-30);
    pid(10);
    wait(1000, msec);
  }
  else if (auton==2){
    turnPID(-30);
    pid(50);
    turnPID(85);
    eat();
    pid(20);
    turnPID(30);
    pid(10);
  }
  else if (auton==3){
    eat();
    pid(100);
    wait(1000,msec);
    pid(-25);
    turnPID(-45);
    pid(15);
    stopall();
    spill();
    wait(1000,msec);
    stopall();
  }


  // driver control for testing
  usercontrol();
}
