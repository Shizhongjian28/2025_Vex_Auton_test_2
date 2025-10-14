#include "vex.h"

using namespace vex;

// --- Devices (adjust ports as needed) ---
brain Brain;
// Left motors (some reversed if necessary)
motor leftMotor1(PORT19, ratio18_1, true);
motor leftMotor2(PORT16, ratio18_1, true);
motor leftMotor3(PORT17, ratio18_1, true);

// Right motors
motor rightMotor1(PORT20, ratio18_1, false);
motor rightMotor2(PORT18, ratio18_1, false);
motor rightMotor3(PORT8,  ratio18_1, false);

// Motor groups
motor_group leftMotors(leftMotor1, leftMotor2, leftMotor3);
motor_group rightMotors(rightMotor1, rightMotor2, rightMotor3);
// --- PID constants ---
double kP = 0.5;
double kI = 0.0;
double kD = 0.05;

// --- Straight correction constants ---
double kTurn = 0.05; // how much to correct difference between sides

void drivePID(double targetDistanceInches, double maxSpeed = 80) {
  // Reset positions
  leftMotors.resetPosition();
  rightMotors.resetPosition();

  // Convert inches to degrees (assuming 4" diameter wheels)
  double targetRotDeg = (targetDistanceInches / (4 * M_PI)) * 360.0;

  double error = 0, prevError = 0, derivative = 0, integral = 0;

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

    double leftSpeed = drivePower - turnCorrection;
    double rightSpeed = drivePower + turnCorrection;

    // --- Manual bounding (no std::clamp) ---
    if (leftSpeed > maxSpeed) leftSpeed = maxSpeed;
    if (leftSpeed < -maxSpeed) leftSpeed = -maxSpeed;
    if (rightSpeed > maxSpeed) rightSpeed = maxSpeed;
    if (rightSpeed < -maxSpeed) rightSpeed = -maxSpeed;

    // Apply motor velocity
    leftMotors.spin(forward, leftSpeed, pct);
    rightMotors.spin(forward, rightSpeed, pct);

    // Exit condition
    if (fabs(error) < 10) break; // within 10 degrees of target
    wait(20, msec);
  }

  // Stop motors
  leftMotors.stop(brake);
  rightMotors.stop(brake);
}

int main() {

  // Example: drive forward 24 inches
  drivePID(12);
}
