#define ENCODER_OPTIMIZE_INTERRUPTS

#include <AccelStepper.h>
#include <PID_v1.h>
#include <Encoder.h>

const bool inverted = true;
const int stepMode = 3;
const int stepModes[6][5] = {
  {5, 1, 0, 0, 0},
  {10, 2, 1, 0, 0},
  {20, 4, 0, 1, 0},
  {40, 8, 1, 1, 0},
  {80, 16, 0, 0, 1},
  {160, 32, 1, 1, 1}
};
double stepsPerMM;
int outputDir;
AccelStepper stepper(AccelStepper::DRIVER, 9, 8);

double setPoint, input, output, scaleFactor;
double kp, ki, kd;
PID pid(&input, &output, &setPoint, kp, ki, kd, DIRECT);

Encoder encoder(2, 3);

long counter = 0;

void setup() {

  if (inverted) {
    outputDir = 1;
    kp = 55.00;
    ki = 600;
    kd = 5;
    scaleFactor = -1/37.5;
  } else {
    outputDir = -1;
    kp = 25.00;
    ki = 0.00;
    kd = 0.1;
    scaleFactor = -1/100;
  }

  pid = PID(&input, &output, &setPoint, kp, ki, kd, DIRECT);
  
  stepsPerMM = stepModes[stepMode][0];
  stepper.setMaxSpeed(5000000);
  stepper.setMinPulseWidth(5);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(6, stepModes[stepMode][2]);
  digitalWrite(7, stepModes[stepMode][3]);
  digitalWrite(8, stepModes[stepMode][4]);

  setPoint = 0;
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-1950, 1950);

  if (inverted) {
    encoder.write(-1199);
    delay(6000);
  } else {
    encoder.write(0);
  }

  while(encoder.read() < 0) {
    // NOOP
  }
}

void loop() {
  while(abs(input) < 150) {
    int count = encoder.read();
    double ang = count * (360.00 / 2400);
    input = scaleFactor*stepper.currentPosition()/stepsPerMM + 200*sin(ang*(PI/180));
    pid.Compute();
    stepper.setSpeed(outputDir*output);
    stepper.runSpeed();
  }
}