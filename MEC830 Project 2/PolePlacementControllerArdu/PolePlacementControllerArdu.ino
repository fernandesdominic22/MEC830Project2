#include <AccelStepper.h>
#include <Encoder.h>
#include <BasicLinearAlgebra.h>  // Include BasicLinearAlgebra for matrix operations

const bool inverted = true;
const int stepMode = 0;
const int stepModes[6][5] = {
  {5/3, 1, 0, 0, 0},
  {10, 2, 1, 0, 0},
  {20, 4, 0, 1, 0},
  {40, 8, 1, 1, 0},
  {80, 16, 0, 0, 1},
  {160, 32, 1, 1, 1}
};

double stepsPerMM;
int outputDir;
AccelStepper stepper(AccelStepper::DRIVER, 9, 8);
Encoder encoder(2, 3);
long prevPosition = 0;
unsigned long prevTime = 0;
double angular_velocity = 0.0;

// Define gain matrix K for 4 states using BLA
BLA::Matrix<1, 4> K;  // Explicitly use BLA namespace
double cartSetPoint = 0;  // Desired cart position

void setup() {
  if (inverted) {
    outputDir = -15;
    stepsPerMM = stepModes[stepMode][0];
  } else {
    outputDir = 1;
    stepsPerMM = stepModes[stepMode][0];
  }

  // Initialize K values
  K(0, 0) = 5.868926317073623;
  K(0, 1) = 0.845742379162432;
  K(0, 2) = -6.0e-7;
  K(0, 3) = -9.0e-4;

  stepper.setMaxSpeed(1900);
  stepper.setMinPulseWidth(5);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(6, stepModes[stepMode][2]);
  digitalWrite(7, stepModes[stepMode][3]);
  digitalWrite(8, stepModes[stepMode][4]);

  if (inverted) {
    encoder.write(-1199);
    delay(6000);
  } else {
    encoder.write(0);
  }

  prevPosition = encoder.read();
  prevTime = millis();
}

void loop() {
  int count = encoder.read();
  double ang = count * (360.0 / 2400);  // Pendulum angle in degrees

  // Calculate angular velocity (degrees per second)
  unsigned long currTime = millis();
  int deltaPosition = count - prevPosition;
  unsigned long deltaTime = currTime - prevTime;

  if (deltaTime > 0) {  // Prevent division by zero
    angular_velocity = (deltaPosition * (360.0 / 2400)) / (deltaTime / 1000.0);
  }

  // Update previous values
  prevPosition = count;
  prevTime = currTime;

  double cart_position = stepper.currentPosition();
  double cart_velocity = stepper.speed();

  // Define state vector X = [angle, angular_velocity, cart_position, cart_velocity]
  BLA::Matrix<4, 1> X;
  X(0, 0) = ang;
  X(1, 0) = angular_velocity;
  X(2, 0) = cart_position;
  X(3, 0) = cart_velocity;

  // Calculate control input U = -K * X
  BLA::Matrix<1, 1> controlInput = -K * X;

  // Apply control input to stepper motor
  stepper.setSpeed(outputDir * controlInput(0, 0));
  stepper.runSpeed();
}
