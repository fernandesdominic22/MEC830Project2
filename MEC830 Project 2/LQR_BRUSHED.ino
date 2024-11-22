#include <Arduino.h>
#include <Encoder.h>
#include <Servo.h>

// Motor encoder pins
#define OUTPUT_A  18
#define OUTPUT_B  19

// Pendulum encoder pins
#define REF_OUT_A 2
#define REF_OUT_B 3

//Potentiometer for Fine Cart Position Adjustement
#define POT_PIN A0 // Pot Connected to Arduino

// Pendulum encoder pulses per revolution
#define PENDULUM_ENCODER_PPR 2400

// Pulley radius on encoder shaft (meters)
#define SHAFT_R 0.00637

// Motor constants
#define POSITION_LIMIT  0.15

#define A 35.98
#define B 1
#define C 1


#define Kx  -185.1640
#define Kth 314.6887
#define Kv  -108.9983
#define Kw  45.4043

float Kr = 1;


const float PI2 = 2.0 * PI;
const float THETA_THRESHOLD = PI/12 ;

// Motor driver pins
Servo motorDriver;    // Use Servo library for PWM control
#define PWM_PIN 8

// Encoder instances
Encoder motorEncoder(OUTPUT_A, OUTPUT_B);       // Create encoder instance for the motor
Encoder pendulumEncoder(REF_OUT_A, REF_OUT_B);  // Create encoder instance for the pendulum

bool encoderWaited = false;  // Flag to ensure the loop runs only once

unsigned long now = 0L;
unsigned long lastTimeMicros = 0L;

float x, last_x, v, dt;
float theta, last_theta, thetaReal, w;
float control, u, mappedValue;


//Default Setpoint on Boot
int xSetpoint = 0;
int thetaSetpoint = 0;

unsigned long log_prescaler = 0;

float saturate(float v, float maxValue) {
  return constrain(v, -maxValue, maxValue);
}

float getAngleABS(Encoder& encoder, long ppr) {  
     int count = abs(encoder.read());
  // int n; //number of rotations
  // n = floor(count / 2400;)

  // // Encoder position is constrained to between 0 and 1 rotation
  // if (abs(count) > 2400) { 
  //   // Value wraps over to 0 if the max readout is exceeded
  //   count = 0; 
  // } else if (abs(count) < 0) { 
  //   // Value wraps over to 2400 if the min readout is exceeded
  //   count = 2400; 
  // }
// Return the normalized position as a fraction of a full rotation (in radians)
return (count / static_cast<float>(ppr)) * 2.0 * PI;
}

float getAngleReal(Encoder& encoder, long ppr) {  
     int count = encoder.read();
  // int n; //number of rotations
  // n = floor(count / 2400;)

  // // Encoder position is constrained to between 0 and 1 rotation
  // if (abs(count) > 2400) { 
  //   // Value wraps over to 0 if the max readout is exceeded
  //   count = 0; 
  // } else if (abs(count) < 0) { 
  //   // Value wraps over to 2400 if the min readout is exceeded
  //   count = 2400; 
  // }
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
  if (fabs(w) > 70) return;

  if (log_prescaler % 20 == 0) {
    Serial.print("Theta (rad): "); Serial.print(thetaReal, 4); Serial.print("\t");
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

  // Check if a key is pressed in the Serial Monitor
  if (Serial.available() > 0) {
    char input = Serial.read(); // Read the input
    Serial.println("Program stopped. Press the reset button to restart.");
    while (true); // Halt execution indefinitely
  }
  

  //Fine Tune Cart Positionn with Potentiometer

  int potValue = analogRead(POT_PIN); // Read potentiometer value (0 to 1023)
  
  // Map the potentiometer value to -150 to +150
  float mappedValue = map(potValue, 0, 1023, -150, 150);

  now = micros();
  dt = (now - lastTimeMicros);

  // Read positions from encoders
  long motorPos = motorEncoder.read();
  x = (motorPos / static_cast<float>(PENDULUM_ENCODER_PPR)) * 0.040; // Cart position
  v = (x - last_x) / dt; // Cart velocity

  theta = getAngleABS(pendulumEncoder, PENDULUM_ENCODER_PPR); // Pendulum angle
  thetaReal = getAngleReal(pendulumEncoder, PENDULUM_ENCODER_PPR); // Pendulum Encoder
  w = (thetaReal - last_theta) / dt; // Angular velocity

 if (isControllable(theta) && fabs(x) < POSITION_LIMIT) {
  control = Kr*(Kx * (x) + Kv * v + Kth * (thetaReal) + Kw * w);
  u = (control + A * v + copysignf(C, v)) / B;
  u = 500*u/12.0;
  // Read the pendulum encoder value to determine direction
  driveMotor(saturate(u, 132));  // Saturate to limit motor input within range
} else {
  driveMotor(0);  // If not controllable, stop the motor
}


  last_x = x;
  last_theta = thetaReal;
  lastTimeMicros = now;

  log_state(control, u);


}
