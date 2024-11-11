#include <TimerOne.h>

// Pin Definitions
const int encoderPinA = 2;       // Encoder channel A
const int encoderPinB = 3;       // Encoder channel B
const int motorDirectionPin = 8; // TB6600 Direction pin
const int motorStepPin = 9;      // TB6600 Step pin

// Encoder & Motor Variables
volatile long encoderCount = 0;      // Encoder count for pendulum angle
const int ppr = 600;                 // Encoder Pulses Per Revolution
const float countsPerRevolution = ppr * 4; // Quadrature decoding (4x)

// Pendulum State Variables
float theta = 0;      // Pendulum angle (radians)
float theta_dot = 0;  // Pendulum angular velocity (rad/s)
float prevTheta = 0;  // Previous angle for velocity calculation

// Cart State Variables
float x = 0;          // Cart position (mm)
float x_dot = 0;      // Cart velocity (mm/s)
float prevX = 0;      // Previous position for velocity calculation
float mmPerStep = 40.0 / 200.0; // GT2 pulley conversion: 20 teeth * 2 mm pitch / 200 steps per revolution

// Control Parameters
const float controlFrequency = 1000.0; // Control loop frequency (Hz)
const float dt = 1.0 / controlFrequency; // Time step
float K[] = {-3.5, -3, -4.79, -12.3}; // Gains (K1, K2, K3, K4)

// Function Prototypes
void readEncoder();
void controlLoop();
void moveStepper(float controlSignal);

void setup() {
  // Encoder setup
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), readEncoder, CHANGE);

  // Motor setup
  pinMode(motorDirectionPin, OUTPUT);
  pinMode(motorStepPin, OUTPUT);

  // Serial Monitor for Debugging
  Serial.begin(9600);

  // Control Loop Timer setup
  Timer1.initialize(1000000 / controlFrequency); // Timer1 for control frequency
  Timer1.attachInterrupt(controlLoop);
}

void loop() {
  // Empty loop; control loop handled by Timer1 interrupt
}

// Interrupt Service Routine for Encoder
void readEncoder() {
  int b = digitalRead(encoderPinB);
  encoderCount += (b > 0) ? +1 : -1;
}

// Control Loop
void controlLoop() {
  // Calculate pendulum angle and angular velocity
  theta = (PI * encoderCount) / countsPerRevolution;
  theta_dot = (theta - prevTheta) / dt;
  prevTheta = theta;

  // Estimate cart position and velocity
  x = x + (theta_dot * mmPerStep); // Approximate cart displacement per control step
  x_dot = (x - prevX) / dt;
  prevX = x;

  // Control law: u = -K1*theta - K2*theta_dot - K3*x - K4*x_dot
  float controlSignal = -K[0] * theta - K[1] * theta_dot - K[2] * x - K[3] * x_dot;

  // Move the stepper based on control signal
  moveStepper(controlSignal);

  // Debug: Output state variables and control signal
  Serial.print("Theta: "); Serial.print(theta);
  Serial.print(" | Theta_dot: "); Serial.print(theta_dot);
  Serial.print(" | X: "); Serial.print(x);
  Serial.print(" | X_dot: "); Serial.print(x_dot);
  Serial.print(" | Control: "); Serial.println(controlSignal);
}

// Function to move the stepper based on control signal
void moveStepper(float controlSignal) {
  int steps = abs(controlSignal);  // Convert control signal to step count
  if (steps > 200) steps = 200;    // Cap at 200 steps to prevent overshooting

  if (controlSignal > 0) {
    digitalWrite(motorDirectionPin, HIGH); // Move in one direction
  } else {
    digitalWrite(motorDirectionPin, LOW);  // Move in opposite direction
  }

  for (int i = 0; i < steps; i++) {
    digitalWrite(motorStepPin, HIGH);
    delayMicroseconds(500); // Adjust delay for stepper speed
    digitalWrite(motorStepPin, LOW);
    delayMicroseconds(500);
  }
}
