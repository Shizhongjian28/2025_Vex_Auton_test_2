#include "vex.h"
#include "math.h"
#include "iostream"
#include "algorithm"
using namespace vex;
using namespace std;

// ======================
// --- Constants ---
// ======================
const double WHEEL_DIAMETER = 100.0;      // mm
const double TRACK_WIDTH = 280.0;         // mm between left/right wheels
const double TICKS_PER_REV = 360.0;
const double MM_PER_TICK = (M_PI * WHEEL_DIAMETER) / TICKS_PER_REV;
const double LOOP_DT = 0.02;              // control loop period (s)

const double MAX_LINEAR_SPEED = 500.0;    // mm/s
const double MAX_WHEEL_RPM = 200.0;
const double MAX_ACCELERATION = 150.0;    // mm/s²
const double SLEW_RATE = 30.0;            // rpm per loop
const double pos_tolerance = 15.0;        // mm
const double angle_tolerance = 0.05;      // radians (~3°)

// ======================
// --- Hardware ---
// ======================
motor leftMotor(PORT1, ratio18_1, false);
motor rightMotor(PORT10, ratio18_1, true);
brain Brain;
encoder leftEncoder(Brain.ThreeWirePort.A);
encoder rightEncoder(Brain.ThreeWirePort.C);

// ======================
// --- Helper Functions ---
// ======================
double clamp(double val, double minVal, double maxVal) {
    return fmin(fmax(val, minVal), maxVal);
}

double mmPerSecToRpm(double mmps) {
  double revsPerSec = mmps / (M_PI * WHEEL_DIAMETER);
  return revsPerSec * 60.0;
}

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2*M_PI;
    while (angle < -M_PI) angle += 2*M_PI;
    return angle;
}

double slewLimit(double target, double prev, double rate) {
    double diff = target - prev;
    diff = clamp(diff, -rate, rate);
    return prev + diff;
}

// ======================
// --- PID Controller ---
// ======================
struct PID {
    double kP, kI, kD;
    double integral, prevError;

    PID(double P, double I, double D) : kP(P), kI(I), kD(D), integral(0), prevError(0) {}

    void reset() {
        integral = 0;
        prevError = 0;
    }

    double step(double error, double dt) {
        integral += error * dt;
        double derivative = (error - prevError) / dt;
        prevError = error;
        return kP * error + kI * integral + kD * derivative;
    }
};

// ======================
// --- Odometry ---
// ======================
struct Pose { double x, y, theta; };
struct Waypoint { double x, y; };

class Odometry {
private:
    encoder &leftEnc, &rightEnc;
    Pose pose;
    double prevLeft, prevRight;

    public:
    Odometry(encoder &l, encoder &r) : leftEnc(l), rightEnc(r) {
        reset({0, 0, 0});
    }

    void reset(Pose start) {
        pose = start;
        leftEnc.resetRotation();
        rightEnc.resetRotation();
        prevLeft = 0;
        prevRight = 0;
    }

    void update() {
        double leftTicks = leftEnc.rotation(deg);
        double rightTicks = rightEnc.rotation(deg);
        double dL = (leftTicks - prevLeft) * MM_PER_TICK;
        double dR = (rightTicks - prevRight) * MM_PER_TICK;
        prevLeft = leftTicks;
        prevRight = rightTicks;

        double dCenter = (dL + dR) / 2.0;
        double dTheta = (dR - dL) / TRACK_WIDTH;

        pose.x += dCenter * cos(pose.theta + dTheta / 2.0);
        pose.y += dCenter * sin(pose.theta + dTheta / 2.0);
        pose.theta = normalizeAngle(pose.theta + dTheta);
    }

    Pose getPose() { return pose; }
};

// ======================
// --- Motion Functions ---
// ======================
void goToPoint(Waypoint target, Odometry &odom, PID &pidLinear, PID &pidAngular) {
    pidLinear.reset();
    pidAngular.reset();

    double prevL = 0.0, prevR = 0.0;
    double currentLinearVel = 0.0;

  const double slowdown_radius = 200.0;   // start slowing within 200 mm
  const double min_speed = 40.0;          // mm/s
  const double max_speed = 500.0;         // mm/s

    while (true) {
        odom.update();
        Pose p = odom.getPose();

        double dx = target.x - p.x;
        double dy = target.y - p.y;
        double dist = sqrt(dx*dx + dy*dy);
        double targetAngle = atan2(dy, dx);
        double angleErr = normalizeAngle(targetAngle - p.theta);

        if (dist < pos_tolerance && fabs(angleErr) < angle_tolerance) {
        leftMotor.stop(brakeType::hold);
        rightMotor.stop(brakeType::hold);
        break;
    }

    double desiredLinearVel = pidLinear.step(dist, LOOP_DT);
    double angularVel = pidAngular.step(angleErr, LOOP_DT);

    // --- Soft Stop ---
    double softScale = clamp(dist / slowdown_radius, 0.0, 1.0);
    double softLimitedVel = max(min_speed, softScale * max_speed);
    desiredLinearVel = clamp(desiredLinearVel, -softLimitedVel, softLimitedVel);

    // --- Acceleration limiting ---
    double maxVelChange = MAX_ACCELERATION * LOOP_DT;
    double velDiff = desiredLinearVel - currentLinearVel;
    velDiff = clamp(velDiff, -maxVelChange, maxVelChange);
    currentLinearVel += velDiff;

    // --- Combine linear + angular ---
    double v = currentLinearVel;
    double omega = angularVel;

    double vL_mm_s = v - omega * (TRACK_WIDTH / 2.0);
    double vR_mm_s = v + omega * (TRACK_WIDTH / 2.0);

    double rpmL = mmPerSecToRpm(vL_mm_s);
    double rpmR = mmPerSecToRpm(vR_mm_s);

    rpmL = slewLimit(rpmL, prevL, SLEW_RATE);
    rpmR = slewLimit(rpmR, prevR, SLEW_RATE);

    prevL = rpmL;
    prevR = rpmR;

    leftMotor.spin(fwd, rpmL, velocityUnits::rpm);
    rightMotor.spin(fwd, rpmR, velocityUnits::rpm);

    task::sleep((int)(LOOP_DT * 1000));
    }

    leftMotor.stop(brakeType::hold);
    rightMotor.stop(brakeType::hold);
}

void turnToAngle(double targetAngle, Odometry &odom, PID &pidAngular) {
    pidAngular.reset();
    double prevL = 0, prevR = 0;

    const double slowdown_radius = 0.5; // radians before soft stop
    const double min_speed = 30.0;
    const double max_speed = 300.0;

    while (true) {
        odom.update();
        Pose p = odom.getPose();
        double angleErr = normalizeAngle(targetAngle - p.theta);

        if (fabs(angleErr) < angle_tolerance) break;

        double angularVel = pidAngular.step(angleErr, LOOP_DT);

        // Soft stop for turning
        double softScale = clamp(fabs(angleErr) / slowdown_radius, 0.0, 1.0);
        double softLimited = max(min_speed, softScale * max_speed);
        angularVel = clamp(angularVel, -softLimited, softLimited);

        double vL_mm_s = -angularVel * (TRACK_WIDTH / 2.0);
        double vR_mm_s = angularVel * (TRACK_WIDTH / 2.0);

        double rpmL = mmPerSecToRpm(vL_mm_s);
        double rpmR = mmPerSecToRpm(vR_mm_s);

        rpmL = slewLimit(rpmL, prevL, SLEW_RATE);
        rpmR = slewLimit(rpmR, prevR, SLEW_RATE);

        prevL = rpmL;
        prevR = rpmR;

        leftMotor.spin(fwd, rpmL, velocityUnits::rpm);
        rightMotor.spin(fwd, rpmR, velocityUnits::rpm);

        task::sleep((int)(LOOP_DT * 1000));
    }

    leftMotor.stop(brakeType::hold);
    rightMotor.stop(brakeType::hold);
}

// ======================
// --- Example Actions ---
// ======================
void grabObject() {
    Brain.Screen.printAt(10, 100, "Grabbing object...");
    task::sleep(500);
}

void dropObject() {
    Brain.Screen.printAt(10, 120, "Dropping object...");
    task::sleep(500);
}

// ======================
// --- Main Routine ---
// ======================
int main() {

    Odometry odom(leftEncoder, rightEncoder);
    PID pidLinear(1.5, 0.0, 0.1);  // tune these
    PID pidAngular(3.0, 0.0, 0.2); // tune these
    odom.reset({0, 0, 0});

    // Autonomous path
    goToPoint({600, 0}, odom, pidLinear, pidAngular);
    grabObject();

    goToPoint({600, 600}, odom, pidLinear, pidAngular);
    turnToAngle(M_PI / 2, odom, pidAngular);

    goToPoint({0, 600}, odom, pidLinear, pidAngular);
    dropObject();

    Brain.Screen.printAt(10, 40, "Autonomous complete!");
}
