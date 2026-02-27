
#include "vex.h"
#include <cmath>
#include<iostream>
using namespace vex;

brain Brain;
controller Controller1 = controller(primary); 

//sensors
inertial InertialSensor(PORT3); 
// ---- Device Setup ----

motor L3 = motor(PORT11, ratio6_1, true);
motor L2 = motor(PORT12, ratio6_1, true);
motor L1 = motor(PORT13, ratio6_1, true);

motor R1 = motor(PORT18, ratio6_1, false);
motor R2 = motor(PORT19, ratio6_1, false);
motor R3 = motor(PORT20, ratio6_1, false);

motor upperIntake = motor(PORT16, ratio6_1, false);
motor lowerIntake = motor(PORT1, ratio6_1, false);

motor_group leftMotors = motor_group(L1, L2, L3);
motor_group rightMotors = motor_group(R1, R2, R3);

digital_out hook = digital_out(Brain.ThreeWirePort.A);
digital_out tounge = digital_out(Brain.ThreeWirePort.B);


digital_out doinker = digital_out(Brain.ThreeWirePort.B);
// --- PID constants ---
double kP = 0.2;
double kI = 0.001;
double kD = 2;
double kT=60;
// drive good 20260107: 0.2, 0.001, 2



// --- Straight correction constant ---
double kTurn = 0.05; // how much to correct difference between sides

// --- Acceleration & Deceleration control ---
double maxAccel = 1.1;   // % increase per loop (20ms)
double maxDecel = 1.1;   // % decrease per loop (20ms)

// --- Global speed limit ---
double maxSpeedGlobal = 40;  // maximum % motor output (set lower to slow everything down)

//
double global_distance_scalar=0.5;



void scoreUp(){
  upperIntake.spin(directionType::fwd, -100, velocityUnits::pct);
  lowerIntake.spin(directionType::fwd, 100, velocityUnits::pct);
}

void scoreMiddle(){
  lowerIntake.spin(directionType::fwd, 70, velocityUnits::pct);
}

void store(){
  lowerIntake.spin(directionType::fwd, 100, velocityUnits::pct);
  //upperIntake.spin(directionType::rev, 10, velocityUnits::pct);
}

void dump(){
  upperIntake.spin(directionType::fwd, 100, velocityUnits::pct);
  lowerIntake.spin(directionType::rev, 100, velocityUnits::pct);
}

void stop(){
  upperIntake.stop(brakeType::hold);
  lowerIntake.stop(brakeType::hold);
}

void hookSet(bool state){
  hook.set(state);
}

void toungeSet(bool state){
  tounge.set(state);
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
    if (fabs(error) < 20) break;

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
  // past pid
  // 0.7, 0.005, 4.5
  // 0.63, 0.005, 3.8

  // new pid values
  // for 45: 0.5, 0.005, 1
  // for 90: 0.6, 0.005, 3.5
  double kP = 0.6; 
  double kI = 0.005;
  double kD = 3.5;
  // if (std::abs(targetDegrees)<145){
  //   kP=0.65;
  //   kD=3.8;
  // }
  // if (std::abs(targetDegrees)<100){
  //   kP=0.75;
  //   kD=5;
  // }
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
  double turningSensativity = 0.4;

  // deadband (important)
  if (fabs(forward) < 5) forward = 0;
  if (fabs(turn) < 5) turn = 0;

  // If no input at all → BRAKE
  if (forward == 0 && turn == 0) {
    leftMotors.stop(brakeType::brake);   // or hold
    rightMotors.stop(brakeType::brake);
    return;
  }

  double leftSpeed = forward + turn * turningSensativity;
  double rightSpeed = forward - turn * turningSensativity;

  if (leftSpeed > 100) leftSpeed = 100;
  if (leftSpeed < -100) leftSpeed = -100;
  if (rightSpeed > 100) rightSpeed = 100;
  if (rightSpeed < -100) rightSpeed = -100;

  leftMotors.spin(fwd, leftSpeed, pct);
  rightMotors.spin(fwd, rightSpeed, pct);
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
  double currentForward = 0.0;
  double currentTurn = 0.0;
  
  int displayWidth = 32; 
  int scrollDelay = 200; 
  
  int scrollPosition = 0;
  int lastScrollTime = 0;
  int lastStatusUpdate = 0;
  int statusUpdateDelay = 500; 

  bool tigersHeart = false; 
  bool emmaVibrationEnabled = true; 
  bool textScrollEnabled = true;
  int leftButtonCounter = 0;
  int upButtonCounter = 0;
  bool buttonLeftWasPressed = false;
  bool buttonUpWasPressed = false;
  int lastBrainUpdate = 0;
  int brainUpdateDelay = 1000;
  int tigerKeepSpammingTheButtonLMAO = 67;

  int badTemperature = 50;

  bool hookStatus = false;
  bool toungeStatus = false;
  int lastHookToggleTime = 0; 
  int lastToungeToggleTime = 0;
  bool buttonBWasPressed = false; 
  bool buttonDownWasPressed = false;

  Brain.Screen.clearScreen();

  // User control code here, inside the loop
  while (1) {
    if (Brain.Timer.time(msec) - lastBrainUpdate >= brainUpdateDelay) {
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.print("Grants9thRobot - DRIVER    ");
      
      Brain.Screen.setCursor(2, 1);
      Brain.Screen.print("Bat:%d%% %.1fC %.1fV     ", 
        Brain.Battery.capacity(),
        Brain.Battery.temperature(temperatureUnits::celsius),
        Brain.Battery.voltage());
      
      motor* motors[] = {&L1, &L2, &L3, &R1, &R2, &R3, &upperIntake, &lowerIntake};
      char* names[] = {"L1", "L2", "L3", "R1", "R2", "R3", "UI", "LI"};
      
      for (int i = 0; i < 8; i++) {
        Brain.Screen.setCursor(3 + i, 1);
        Brain.Screen.print("%s:%dv %dA %dW %dNm %d%% %dC  ",
          names[i],
          (int)motors[i]->velocity(velocityUnits::pct),
          (int)motors[i]->current(currentUnits::amp),
          (int)motors[i]->power(powerUnits::watt),
          (int)motors[i]->torque(torqueUnits::Nm),
          (int)motors[i]->efficiency(percentUnits::pct),
          (int)motors[i]->temperature(temperatureUnits::celsius));
      }
      
      lastBrainUpdate = Brain.Timer.time(msec);
    }
    
    if (Brain.Timer.time(msec) - lastStatusUpdate >= statusUpdateDelay) {
      Controller1.Screen.setCursor(1, 1);
      int totalSeconds = Brain.Timer.time(msec) / 1000; 
      int minutes = totalSeconds / 60;
      int seconds = totalSeconds % 60;
      int battery = Brain.Battery.capacity();
      Controller1.Screen.print("%d:%02d %d%% %.1fC  ", minutes, seconds, battery);
      
      Controller1.Screen.setCursor(3, 1);
      std::string warning = "";
      if (L1.temperature(temperatureUnits::celsius) > badTemperature) {
        warning += "L1 ";
      }
      if (L2.temperature(temperatureUnits::celsius) > badTemperature) {
        warning += "L2 ";
      }
      if (L3.temperature(temperatureUnits::celsius) > badTemperature) {
        warning += "L3 ";
      }
      if (R1.temperature(temperatureUnits::celsius) > badTemperature) {
        warning += "R1 ";
      }
      if (R2.temperature(temperatureUnits::celsius) > badTemperature) {
        warning += "R2 ";
      }
      if (R3.temperature(temperatureUnits::celsius) > badTemperature) {
        warning += "R3 ";
      }
      if (upperIntake.temperature(temperatureUnits::celsius) > badTemperature) {
        warning += "UI ";
      }
      if (lowerIntake.temperature(temperatureUnits::celsius) > badTemperature) {
        warning += "LI ";
      }
      
      if (!warning.empty()) {
        Controller1.Screen.print("HOT:%s", warning.c_str());
      } else {
        Controller1.Screen.print("                         ");
      }
      
      lastStatusUpdate = Brain.Timer.time(msec);
    }                                                                                              
    
    
    if (Controller1.ButtonUp.pressing() && !buttonUpWasPressed) {
      buttonUpWasPressed = true;
      upButtonCounter++;
      if (upButtonCounter >= tigerKeepSpammingTheButtonLMAO) {
        textScrollEnabled = !textScrollEnabled;
        upButtonCounter = 0;
      }
    } 
    else if (!Controller1.ButtonUp.pressing()) {
      buttonUpWasPressed = false;
    }

    if (Controller1.ButtonLeft.pressing() && !buttonLeftWasPressed) {
      buttonLeftWasPressed = true;
      leftButtonCounter++;
      if (leftButtonCounter >= tigerKeepSpammingTheButtonLMAO) {
        emmaVibrationEnabled = !emmaVibrationEnabled;
        leftButtonCounter = 0;
      }
    } 
    else if (!Controller1.ButtonLeft.pressing()) {
      buttonLeftWasPressed = false;
    }

    double targetForward = Controller1.Axis3.position(percentUnits::pct);
    double targetTurn = Controller1.Axis1.position(percentUnits::pct);

    // DEADZONE to stop turning drift
    int deadband = 5;

    if (fabs(targetTurn) < deadband) {
      targetTurn = 0;
    }

    if (fabs(targetForward) < deadband) {
      targetForward = 0;
    }

    currentForward = targetForward;
    currentTurn = targetTurn;

    if (Controller1.ButtonR1.pressing()) {
      store();
    } 
    else if (Controller1.ButtonR2.pressing()) {
      scoreUp();
    } 
    else if (Controller1.ButtonL1.pressing()) {
      dump();
    } 
    else if (Controller1.ButtonL2.pressing()) {
      scoreMiddle();
    } 
    else if (Controller1.ButtonDown.pressing()) {
      buttonBWasPressed = true;
    } 
    else if (Controller1.ButtonB.pressing()) {
      buttonDownWasPressed = true;
    }
    else if (buttonBWasPressed) {
      int now = Brain.Timer.time(msec);
      if (now - lastHookToggleTime >= 500) {
        hookStatus = !hookStatus;
        hookSet(hookStatus);
        lastHookToggleTime = now;
      }
      buttonBWasPressed = false;
    }
    else if (buttonDownWasPressed) {
      int now = Brain.Timer.time(msec);
      if (now - lastHookToggleTime >= 500) {
        toungeStatus = !toungeStatus;
        toungeSet(toungeStatus);
        lastToungeToggleTime = now;
      }
      buttonDownWasPressed = false;
    }
    else {
      stop();
    }

    drive(currentForward, currentTurn);

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
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
  int auton=2;
  int ttest=0;
  if (ttest==1){
    if (auton==0){
      pid(-95);
      wait(1000,msec);
      pid(80);
    }
    else if (auton==1){
      pid(kT);
      wait(1000,msec);
      turnPID(90);
      wait(1000,msec);
      turnPID(-90);
      wait(1000,msec);
      pid(-kT);
    }
    else if (auton==2){
      turnPID(45);
      wait(1000,msec);
      turnPID(-45);
      wait(1000,msec);
      turnPID(90);
      wait(1000,msec);
      turnPID(-90);
      wait(1000,msec);
      turnPID(180);
      wait(1000,msec);
      turnPID(-180);
      wait(1000,msec);
      turnPID(145);
      wait(1000,msec);
      turnPID(-145);
      wait(1000,msec);
    }
    else if (auton==3){
      pid(41);
      turnPID(136);
      store();
      pid(139);
      stop();
      dump();
      wait(1000,msec);
      stop();
      pid(-35);
      turnPID(-48);
      store();
      maxSpeedGlobal=40;
      pid(135);
      wait(500,msec);
      turnPID(-55);
      pid(-30);
      scoreMiddle();
    }
  }
  else if (ttest==0) {
    if (auton==1){// solo awp
      store();
      maxSpeedGlobal=40;
      pid(35);
      maxSpeedGlobal=65;
      pid(-155);
      stop();
      wait(250,msec);
      turnPID(-100);
      toungeSet(true);
      store();
      pid(45);
      wait(500,msec);
      pid(-85, 1500);
      stop();
      scoreUp();
      toungeSet(false);
      wait(1500,msec);
      stop();
      pid(39);
      turnPID(138);
      store();
      pid(130);
      stop();
      dump();
      wait(1000,msec);
      stop();
      pid(-35);
      turnPID(-48);
      store();
      maxSpeedGlobal=40;
      pid(135);
      wait(500,msec);
      turnPID(-55);
      pid(-30);
      scoreUp();
      wait(1500,msec);
      stop();


    }
    else if (auton==2){// starting from right scoring bottom
      pid(30);
      turnPID(60);
      // store();
      toungeSet(true);
      pid(64);
      wait(500,msec);
      toungeSet(false);
      turnPID(-108);
      maxSpeedGlobal=65;
      pid(39);
      stop();
      dump();
      wait(1000,msec);
      stop();
      maxDecel=100;
      pid(-120);
      wait(500,msec);
      turnPID(-140);
      wait(500,msec);
      pid(-50);
      pid(-10,1000);
      wait(500,msec);
      toungeSet(true);
      pid(120);
      store();
      pid(100);
      wait(1000,msec);
      pid(-120);
      stop();
      scoreUp();
      wait(2500,msec);
    }
    else if (auton==3){// starting from left scoring middle
      pid(30);
      turnPID(-60);
      store();
      pid(43);
      toungeSet(true);
      maxSpeedGlobal=20;
      pid(20);
      wait(1000,msec);
      toungeSet(false);
      turnPID(-87);
      maxSpeedGlobal=65;
      pid(-40);
      stop();
      scoreUp();
      wait(2500,msec);
      stop();
      pid(140);
      wait(500,msec);
      turnPID(-45);
      toungeSet(true);
      store();
      pid(100);
      wait(1000,msec);
      pid(-120);
      stop();
      scoreUp();
      wait(2500,msec);
    }
    else if (auton==4){
      pid(17);
      turnPID(60);
      store();
      maxSpeedGlobal=20;
      pid(50);
      wait(500,msec);
      maxSpeedGlobal=65;
      turnPID(-112);
      pid(25);
      stop();
      dump();
      wait(1500,msec);
      stop();
      pid(-8);
      wait(100,msec);
      turnPID(100);
      maxSpeedGlobal=40;
      pid(30);
      wait(100,msec);
      turnPID(40);
      wait(100,msec);
      maxSpeedGlobal=25;
      store();
      pid(30);
      wait(100,msec);
      maxSpeedGlobal=65;
      pid(-10);
      turnPID(-105);
      stop();
      pid(-60);
      turnPID(-95);
      pid(-20);
      turnPID(-88);
      pid(-30);
      maxSpeedGlobal=45;
      pid(-10, 1000);
      dump();
      wait(3000,msec);
      stop();
    }
    else if (auton==5){// starting from right all score long goal
      pid(32);
      turnPID(35);
      store();
      pid(44);
      toungeSet(true);
      maxSpeedGlobal=20;
      pid(20);
      wait(500,msec);
      toungeSet(false);
      turnPID(70);
      maxSpeedGlobal=65;
      pid(90);
      wait(500,msec);
      turnPID(38);
      toungeSet(true);
      store();
      pid(40);
      wait(1000,msec);
      pid(-130,2000);
      stop();
      dump();
      wait(300,msec);
      scoreUp();
      wait(2500,msec);
      pid(30);
      wait(500,msec);
      pid(-30);
    }
    else if (auton==6){
      maxSpeedGlobal=40;
      scoreUp();
      pid(1000000);
    }
  }
  std::cout<<"auton time: "<<Brain.timer(timeUnits::msec)-time<<std::endl;


  // driver control for testing
  usercontrol();
}
