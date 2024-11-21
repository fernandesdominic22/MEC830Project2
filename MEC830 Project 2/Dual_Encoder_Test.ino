#include <Encoder.h>

// Define pins for Encoder 1
#define ENCODER1_PIN_A 2
#define ENCODER1_PIN_B 4

// Define pins for Encoder 2
#define ENCODER2_PIN_A 3
#define ENCODER2_PIN_B 5

// Create Encoder objects
Encoder encoder1(ENCODER1_PIN_A, ENCODER1_PIN_B);
Encoder encoder2(ENCODER2_PIN_A, ENCODER2_PIN_B);

void setup() {
  // Start serial communication
  Serial.begin(9600);
  Serial.println("Encoder test using Encoder library");
}

void loop() {
  // Read encoder positions
  long encoder1Position = encoder1.read();
  long encoder2Position = encoder2.read();

  // Output encoder values
  Serial.print("Encoder 1: ");
  Serial.print(encoder1Position);
  Serial.print("\tEncoder 2: ");
  Serial.println(encoder2Position);

  delay(100); // Adjust delay for preferred update rate
}
