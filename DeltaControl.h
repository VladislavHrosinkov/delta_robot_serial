#ifndef MY_HEADER_H
#define MY_HEADER_H

// Structure for storing cartesian coordinates
typedef struct {
  double x, y, z, rotator_degrees;
  double grabber_degrees;
} pose;

// Structure for storing inverse kinematics solutions
typedef struct {
  double motor_ticks[3];
  double rotator_degrees;
  double grabber_degrees;
} solution;

// Number of motors
#define MOTORS 3
#define SERVOS 2

// Pins for the rotator and gripper
#define ROTATOR_PIN 8
#define GRIPPER_PIN 7

// Indices for the rotator and gripper
#define ROTATOR_SERVO 0
#define GRIPPER_SERVO 1

// Constants used in DELTA kinematics, in mm
const double sqrt_3 = sqrt(3);
// From the center of the stationary platform to the motor
const double wB = 72;
// Distance from the center of the moving platform to the line between two parallelogram mechanisms
const double wP = 19;
// From the center of the moving platform to the center of the parallelogram mechanisms
const double uP = 39;
// Distance between two parallelogram mechanisms, on the mobile platform.
const double sP = 67;
const double a = wB - uP;
const double b = (sP - sqrt_3 * wB) / 2.0;
const double c = wP - wB / 2.0;
// Length of the upper arm
const double L = 120;
// Length of the lower arm
const double l = 200;

// Height limits of the manipulator
const double delta_height_min = 50;
const double delta_height_max = 300;

// Angle of the arms in radians (downwards from the horizontal)
const double rest_angles[MOTORS] = {0, 0, 0};

// Initial horizontal angle of the arms in ticks, to be discovered experimentally
const double rest_ticks[MOTORS] = {785, 695, 685};

// Constants for converting between encoder ticks and degrees
const double degreesPerTick = 0.3;
const double radiansPerTick = degreesPerTick * PI / 180.0;

double inline radianstoTicks(double angle);
void motorDegrees(int* degrees);
void initMotors();
void movetoPose(pose &p);
void deactivateMotors();
void replayMemorized(int degrees[]);
void restartMotors();
void setMotorDegrees(int degrees[]);
solution inline solveInverse(pose &d);

#endif
