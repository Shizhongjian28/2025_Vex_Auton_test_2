#include "vex.h"

using namespace vex;

// --- Devices (adjust ports as needed) ---
brain Brain;
//intake motors
motor BottomIntake = motor(PORT6, ratio18_1, false);
motor UpOrDown = motor(PORT10, ratio18_1, false);
motor UpperIntake = motor(PORT2, ratio18_1, false);


// driver motors
motor leftMotor1(PORT19, ratio18_1, true);
motor leftMotor2(PORT16, ratio18_1, true);
motor leftMotor3(PORT17, ratio18_1, true);
motor rightMotor1(PORT20, ratio18_1, false);
motor rightMotor2(PORT18, ratio18_1, false);
motor rightMotor3(PORT8,  ratio18_1, false);
// driver Motor groups
motor_group leftMotors(leftMotor1, leftMotor2, leftMotor3);
motor_group rightMotors(rightMotor1, rightMotor2, rightMotor3);

// --- PID constants ---
double kP = 0.5;
double kI = 0.0;
double kD = 0.05;

// --- Straight correction constant ---
double kTurn = 0.05; // how much to correct difference between sides

// --- Acceleration & Deceleration control ---
double maxAccel = 3.0;   // maximum % increase per loop (20ms)
double maxDecel = 3.0;   // maximum % decrease per loop (20ms)

void drivePID(double targetDistanceInches, double maxSpeed = 80) {
  // Reset positions
  leftMotors.resetPosition();
  rightMotors.resetPosition();

  // Convert inches to degrees (assuming 4" diameter wheels)
  double targetRotDeg = (targetDistanceInches / (4 * M_PI)) * 360.0;

  double error = 0, prevError = 0, derivative = 0, integral = 0;

  double currentLeftSpeed = 0;
  double currentRightSpeed = 0;

  while (true) {
    // Get average motor position
    double leftPos = leftMotors.position(degrees);
    double rightPos = rightMotors.position(degrees);
    double avgPos = (leftPos + rightPos) / 2.0;

    // --- Main distance PID ---
    error = targetRotDeg - avgPos;
    integral += error;
    derivative = error - prevError;
    prevError = error;

    double drivePower = (error * kP) + (integral * kI) + (derivative * kD);

    // --- Straight correction using left/right difference ---
    double sideError = leftPos - rightPos;
    double turnCorrection = sideError * kTurn;

    double targetLeftSpeed = drivePower - turnCorrection;
    double targetRightSpeed = drivePower + turnCorrection;

    // --- Manual bounding (no std::clamp) ---
    if (targetLeftSpeed > maxSpeed) targetLeftSpeed = maxSpeed;
    if (targetLeftSpeed < -maxSpeed) targetLeftSpeed = -maxSpeed;
    if (targetRightSpeed > maxSpeed) targetRightSpeed = maxSpeed;
    if (targetRightSpeed < -maxSpeed) targetRightSpeed = -maxSpeed;

    // --- Apply acceleration/deceleration limits ---
    double leftDelta = targetLeftSpeed - currentLeftSpeed;
    double rightDelta = targetRightSpeed - currentRightSpeed;

    if (leftDelta > maxAccel) leftDelta = maxAccel;
    if (leftDelta < -maxDecel) leftDelta = -maxDecel;
    if (rightDelta > maxAccel) rightDelta = maxAccel;
    if (rightDelta < -maxDecel) rightDelta = -maxDecel;

    currentLeftSpeed += leftDelta;
    currentRightSpeed += rightDelta;

    // Apply motor velocity
    leftMotors.spin(forward, currentLeftSpeed, pct);
    rightMotors.spin(forward, currentRightSpeed, pct);

    // Exit condition
    if (fabs(error) < 10) break; // within 10 deg of target
    wait(20, msec);
  }

  // Stop motors
  leftMotors.stop(brake);
  rightMotors.stop(brake);
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
  eat();
  drivePID(24);
  stopall();
  //turn
  drivePID(3);
  scorebottom();
  wait(1000, msec);
  stopall();
  drivePID(-5);
  //turn
  eat();
  drivePID(24);
  stopall();
  drivePID(-24);
  //turn
  drivePID(5);
  //turn
  drivePID(20);
  scoretop();
  wait(1000, msec);
  stopall();
  eat();
  drivePID(-20);
  wait(1000, msec);
  stopall();
  drivePID(5);
  //end


  // driver control for testing
  usercontrol();
}
