/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       driver_control.cpp                                        */
/*    Author:       michael driving testing                                   */
/*    Description:  Driver Control with Independent Exponential Scaling       */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>
using namespace vex;

brain Brain;
controller Controller1 = controller(primary); 

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

// ---- Helper Functions ----
void eat() {
  BottomIntake.spin(forward, 100, percent);
}

void scoretop() {
  BottomIntake.spin(forward, 100, percent);
  UpperIntake.spin(forward, 80, percent);
  UpOrDown.spin(reverse, 100, percent);
}

void scorebottom() {
  UpOrDown.spin(forward, 100, percent);
  BottomIntake.spin(forward, 100, percent);
  UpperIntake.spin(forward, 80, percent);
}

void spill() {
  BottomIntake.spin(reverse, 100, percent);
  UpperIntake.spin(reverse, 100, percent);
}

void stopall() {
  BottomIntake.stop(brake);
  UpperIntake.stop(brake);
  UpOrDown.stop(brake);
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

// ---- Driver Control ----
int main() {
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
    if (Controller1.ButtonLeft.pressing() && !buttonBPressed) {
      toggleDoinker();
      buttonBPressed = true;
    } else if (!Controller1.ButtonLeft.pressing()) {
      buttonBPressed = false;
    }

    // ---- Display Info ----
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(10, 20, "Sens: %.2f | F-Expo: %.2f | T-Expo: %.2f", sensitivity, expoForward, expoTurn);

    wait(20, msec);
  }
}
