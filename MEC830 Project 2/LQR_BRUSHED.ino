#include <Arduino.h>
#include <Encoder.h>
#include <Servo.h>

// Motor encoder pins
#define OUTPUT_A  3
#define OUTPUT_B  5

// Pendulum encoder pins
#define REF_OUT_A 2
#define REF_OUT_B 4

//Potentiometer for Fine Cart Position Adjustement
#define POT_PIN A0 // Pot Connected to Arduino

// Pendulum encoder pulses per revolution
#define PENDULUM_ENCODER_PPR 2400

// Pulley radius on encoder shaft (meters)
#define SHAFT_R 0.00637

// Motor constants
#define POSITION_LIMIT  0.145

#define A 35.98
#define B 2.22
#define C 2.79

#define Kth -185.1640
#define Kw  -180.0237
#define Kx  750.2042
#define Kv  119.5880

const float THETA_THRESHOLD = PI / 12;
const float PI2 = 2.0 * PI;

// Motor driver pins
Servo motorDriver;    // Use Servo library for PWM control
#define PWM_PIN 9

// Encoder instances
Encoder motorEncoder(OUTPUT_A, OUTPUT_B);       // Create encoder instance for the motor
Encoder pendulumEncoder(REF_OUT_A, REF_OUT_B);  // Create encoder instance for the pendulum

unsigned long now = 0L;
unsigned long lastTimeMicros = 0L;

float x, last_x, v, dt;
float theta, last_theta, w;
float control, u, mappedValue;


//Default Setpoint on Boot
int xSetpoint = 0;
int thetaSetpoint = PI;

unsigned long log_prescaler = 0;

float saturate(float v, float maxValue) {
  return constrain(v, -maxValue, maxValue);
}

float getAngle(Encoder& encoder, long ppr) {  
  int count = encoder.read();
  // Encoder position is constrained to between 0 and 1 rotation
  if (count > 2400) { 
    // Value wraps over to 0 if the max readout is exceeded
    count = 0; 
  } else if (count < 0) { 
    // Value wraps over to 2400 if the min readout is exceeded
    count = 2400; 
  }
// Return the normalized position as a fraction of a full rotation (in radians)
return (count / static_cast<float>(ppr)) * 2.0 * PI;
}

void driveMotor(float u) {
  // For motors with 1500 as neutral, and +/- 500 for direction and speed:
  // Adjust the PWM value so that it fits within the range 1000-2000 (Servo standard range)
  int pwm_value = 1500 + saturate(u, 500);  // Neutral PWM is 1500, adjust +/- 500 for speed/direction
  motorDriver.writeMicroseconds(pwm_value); // Send PWM signal to motor
}

boolean isControllable(float theta) {
  return fabs(theta) < THETA_THRESHOLD;
}

void log_state(float control, float u) {
  if (fabs(w) > 100) return;

  if (log_prescaler % 20 == 0) {
    Serial.print("Theta (rad): "); Serial.print(theta, 4); Serial.print("\t");
    Serial.print("Angular Velocity (rad/s): "); Serial.print(w, 4); Serial.print("\t");
    Serial.print("Cart Position (m): "); Serial.print(x, 4); Serial.print("\t");
    Serial.print("Cart Velocity (m/s): "); Serial.print(v, 4); Serial.print("\t");
    Serial.print("Control Signal: "); Serial.print(control, 4); Serial.print("\t");
    Serial.print("Motor Input (PWM): "); Serial.println(u, 4);
    Serial.print("Pot Setpoint (mm): "); Serial.println(mappedValue, 4);
  }
  log_prescaler++;
}

void setup() {
  motorDriver.attach(PWM_PIN);  // Attach the Servo library to the motor pin
  Serial.begin(9600);
  lastTimeMicros = micros();
}

void loop() {

  //Fine Tune Cart Positionn with Potentiometer

  int potValue = analogRead(POT_PIN); // Read potentiometer value (0 to 1023)
  
  // Map the potentiometer value to -150 to +150
  float mappedValue = map(potValue, 0, 1023, -150, 150);

  now = micros();
  dt = (now - lastTimeMicros) / 1000000.0;

  // Read positions from encoders
  long motorPos = motorEncoder.read();
  x = (motorPos / static_cast<float>(PENDULUM_ENCODER_PPR)) * 2.0 * PI * SHAFT_R; // Cart position
  v = (x - last_x) / dt; // Cart velocity

  theta = getAngle(pendulumEncoder, PENDULUM_ENCODER_PPR); // Pendulum angle
  w = (theta - last_theta) / dt; // Angular velocity

  if (isControllable(theta) && fabs(x) < POSITION_LIMIT) {
    control = (Kx * (x-xSetpoint) + Kv * v + Kth * (theta-thetaSetpoint) + Kw * w);
    u = (control + A * v + copysignf(C, v)) / B;
    u =  (u / 100) + 1500;
    driveMotor(saturate(u, 500));  // Update for the servo range
  } else {
    driveMotor(0);
  }

  last_x = x;
  last_theta = theta;
  lastTimeMicros = now;

  log_state(control, u);

  delay(5);
}
