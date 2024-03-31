#include <Arduino.h>
#include "DeltaControl.h"
#include <Herkulex.h>
#include <Servo.h>

Servo servos[SERVOS];
int servo_positions[SERVOS];

void deactivateMotors(){
   for (int i = 1; i <= MOTORS; ++i) {
    Herkulex.clearError(i);
    Herkulex.torqueOFF(i);
  }
  delay(500);
}

// Function that calculates the engine's state from a given angle
double inline radianstoTicks(double angle, int i){
  return rest_ticks[i] + (angle - rest_angles[i]) / radiansPerTick;
}

// Function that returns the motor angles in units specific to the motor
void motorDegrees(int* degrees){
  for (int i = 1; i <= MOTORS; ++i) {
    degrees[i - 1] = int(Herkulex.getAngle(i));
  }
}


// Function for calculating delta robot inverse kinematics
solution inline solveInverse(pose &d) {
 
  // Limiting the value of z to fall within the permissible range
  d.z = constrain((int) d.z, -delta_height_max, -delta_height_min);
  solution sol = {0, 0, 0, d.rotator_degrees, d.grabber_degrees};

  double E[] = {2.0*L*(a + d.y),
  -L*(sqrt_3*(d.x + b) + d.y + c),
  L*(sqrt_3*(d.x - b) - d.y - c)};

  double F = 2.0*d.z*L;

  double xyz2 = d.x*d.x + d.y*d.y + d.z*d.z + L*L - l*l;
  double bc2 = b*b + c*c;
  double G[] = {xyz2 + a*a + 2.0*d.y*a,
  xyz2 + bc2 + 2.0*(d.x*b + d.y*c),
  xyz2 + bc2 + 2.0*(-d.x*b + d.y*c)};

  double angle_radians[MOTORS];
  for (int i = 0; i < MOTORS; i++) {
   
    // Choosing one of the two solutions of the quadratic equation
    double denom = G[i] - E[i];
    double descrim = sqrt(E[i]*E[i] + F*F - G[i]*G[i]);

    double num = descrim - F;
    double angle1 = 2.0 * atan2(num, denom);
    num = -(descrim + F);
    double angle2 = 2.0 * atan2(num, denom);

    angle_radians[i] = angle2;
    if (-PI/4 < angle1 && angle1 < PI/2){
      angle_radians[i] = angle1;
    }
  }

  for (int i = 0; i < MOTORS; ++i) {
    sol.motor_ticks[i] = radianstoTicks(angle_radians[i], i);
  }

  return sol;
}

// Completely restarts the motors
void restartMotors(){   

  for (int i = 1; i <= MOTORS; ++i) {
    Herkulex.reboot(i);
  }

  delay(500);

 
  // Initializing Herkulex motors
  Herkulex.initialize();
  delay(500);
 
  // Activating all motors
  for (int i = 1; i <= MOTORS; ++i) {
    Herkulex.clearError(i);
    Herkulex.torqueON(i);
  }
  delay(500);
}

// Initializes the motors
void initMotors() {

   
  // Initializing the built-in LED pin as output
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

 
  // Initializing serial communication with Herkulex motors
  Herkulex.beginSerial1(115200);

  restartMotors();

  // Moving all motors to their initial positions
  for (int i = 1; i <= MOTORS; ++i) {
    Herkulex.moveOne(i, rest_ticks[i - 1], 2000, LED_GREEN);
  }
  delay(500);

 
  // Setting initial positions for the rotator and gripper
  servos[ROTATOR_SERVO].attach(ROTATOR_PIN);
  servos[GRIPPER_SERVO].attach(GRIPPER_PIN);
  servo_positions[ROTATOR_SERVO] = 180;
  servo_positions[GRIPPER_SERVO] = 90;
  servos[ROTATOR_SERVO].write(servo_positions[ROTATOR_SERVO]);
  servos[GRIPPER_SERVO].write(servo_positions[GRIPPER_SERVO]);
  delay(500);

 
  // Turning off the servos after setting the initial positions
  servos[ROTATOR_SERVO].detach();
  servos[GRIPPER_SERVO].detach();
}

// Moves to a position in Cartesian coordinates
void movetoPose(pose &p) {
  
  solution sol = solveInverse(p);
  for (int i = 1; i <= MOTORS; ++i) {
    Herkulex.clearError(i);
    Herkulex.torqueON(i);
    Herkulex.moveAll(i, sol.motor_ticks[i - 1], LED_GREEN);
  }

  Herkulex.actionAll(2000);
  delay(500);
}

// Setting motor angles in degrees
void setMotorDegrees(int degrees[]) {

 
  // Moving Herkulex motors to the desired positions
  for (int i = 1; i <= MOTORS; ++i) {
    double radians = degrees[i - 1] * PI/180;
    Herkulex.clearError(i);
    Herkulex.torqueON(i);
    Herkulex.moveAll(i, radianstoTicks(radians, i - 1), LED_GREEN);
  }
  Herkulex.actionAll(2000);
  delay(500);
}

// Repeats learned pose
void replayMemorized(int degrees[]) {
 
  for (int i = 1; i <= MOTORS; ++i) {
    Herkulex.clearError(i);
    Herkulex.moveAllAngle(i, degrees[i - 1], LED_GREEN);
  }
  Herkulex.actionAll(500);
}
