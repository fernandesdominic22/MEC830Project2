#include <Encoder.h>
#include <AccelStepper.h>

#define ENCODER_PIN_A 2          // Encoder A channel pin
#define ENCODER_PIN_B 3          // Encoder B channel pin
#define STEPPER_STEP_PIN 9       // TB6600 step input
#define STEPPER_DIR_PIN 8        // TB6600 direction input

const float PPR = 600.0;               // Encoder pulses per revolution
const float STEPS_PER_REV = 200;       // Stepper motor steps per revolution
const float PULLEY_TOOTH_COUNT = 20;   // GT2 pulley tooth count
const float GT2_PITCH = 2.0;           // mm per tooth for GT2 belt
const float MAX_TRAVEL_MM = 200.0;     // Maximum travel distance (±200 mm)
const float SMOOTHING_FACTOR = 0.1;    // Smoothing factor for control adjustments

Encoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);
AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP_PIN, STEPPER_DIR_PIN);

long encoderPos = 0;                // Encoder position
float angleRadians = 0.0;           // Angle in radians
long stepCount = 0;                 // Step count for stepper motor
float distanceTraveled = 0.0;       // Distance in mm
float targetAcceleration = 0.0;     // Acceleration command from Python
float currentAcceleration = 0.0;    // Current acceleration being applied
long targetSteps = 0;               // Target steps for stepper motor
float lastTargetSteps = 0;          // Last target steps to smooth the transition

void setup() {
  Serial.begin(9600);             // Start serial communication
  encoder.write(0);                 // Reset encoder position
  stepper.setMaxSpeed(4000);        // Set max speed (steps/sec)
  stepper.setAcceleration(20000);   // Set acceleration (steps/sec^2)
  stepper.setCurrentPosition(0);    // Reset stepper position
}

void loop() {
  // Read encoder and calculate angle
  encoderPos = encoder.read();
  angleRadians = (PI/2 * encoderPos) / PPR;

  // Calculate distance traveled by stepper in mm
  stepCount = stepper.currentPosition();
  distanceTraveled = (stepCount / STEPS_PER_REV) * (PULLEY_TOOTH_COUNT * GT2_PITCH);

  // Receive acceleration command from Python
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    if (input.startsWith("ACCEL:")) {
      targetAcceleration = input.substring(6).toFloat();
      
      // Smoothly transition to the new target acceleration
      currentAcceleration = (SMOOTHING_FACTOR * targetAcceleration) + ((1 - SMOOTHING_FACTOR) * currentAcceleration);

      // Calculate target position based on the smoothed acceleration
      float targetDistance = distanceTraveled + (0.5 * currentAcceleration * 1 * 1) * (PULLEY_TOOTH_COUNT * GT2_PITCH);  // Calculate target distance from acceleration over a small timestep
      targetDistance = constrain(targetDistance, -MAX_TRAVEL_MM, MAX_TRAVEL_MM);  // Constrain within ±200 mm
      
      // Calculate target steps from target distance
      targetSteps = (targetDistance / (PULLEY_TOOTH_COUNT * GT2_PITCH)) * STEPS_PER_REV;

      // Smooth transition to the new target position
      targetSteps = (SMOOTHING_FACTOR * targetSteps) + ((1 - SMOOTHING_FACTOR) * lastTargetSteps);
      
      // Move stepper to the new smoothed target position
      stepper.moveTo(targetSteps);
      lastTargetSteps = targetSteps;
    }
  }

  // Run the stepper motor to the target position
  stepper.run();

  // Send encoder and stepper information over Serial
  Serial.print("Encoder Angle (radians): ");
  Serial.print(angleRadians);
  Serial.print(" | Stepper Distance (mm): ");
  Serial.println(distanceTraveled);
}
