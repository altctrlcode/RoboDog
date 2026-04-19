#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "motorSetup.h"

// Create PWM servo driver instance
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo parameters
#define SERVOMIN  150 // Minimum pulse length count (out of 4096)
#define SERVOMAX  600 // Maximum pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Servo channels
#define NUM_LEGS 4
const uint8_t hipChannels[NUM_LEGS] = {0, 1, 2, 3}; // FL, FR, BL, BR
const uint8_t kneeChannels[NUM_LEGS] = {4, 5, 6, 7};

// Neutral angles
const int neutralHip = 90;
const int neutralKnee = 90;

// Walking parameters
const int hipForward[NUM_LEGS] = {45, 135, 45, 135}; // FL, FR, BL, BR forward positions - increased range for bigger steps
const int hipBack[NUM_LEGS] = {135, 45, 135, 45};    // FL, FR, BL, BR back positions for pushing
const int kneeLift[NUM_LEGS] = {120, 45, 120, 45};  // FL, FR, BL, BR knee lift direction
const int stepDelay = 300; // Delay between steps in ms - reduced for faster walking

// Function declarations
void setServoAngle(uint8_t channel, int angle);
void walkStep();
void processSerialCommand();

bool walking = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("Robot Dog Walking Demo");
  Serial.println("Send 'w' to walk and 's' to stop");

  // Initialize I2C
  Wire.begin();

  // Initialize PWM servo driver
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  // Set all servos to 90 degrees for setup
  setAllMotorsTo90(pwm);
  Serial.println("All 8 servos set to 90 degrees");

  delay(1000);
}

void loop() {
  processSerialCommand();

  if (walking) {
    walkStep();
  } else {
    delay(10);
  }
}

void processSerialCommand() {
  while (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'w' || command == 'W') {
      walking = true;
      Serial.println("Walking started");
    } else if (command == 's' || command == 'S') {
      walking = false;
      setAllMotorsTo90(pwm);
      Serial.println("Walking stopped, all servos set to 90 degrees");
    }
  }
}

// Function to set servo angle
void setServoAngle(uint8_t channel, int angle) {
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}

// Simple walking function - diagonal leg pairs
void walkStep() {
  // Step 1: Lift and move front left and back right legs forward
  setServoAngle(hipChannels[0], hipForward[0]); // FL hip forward
  setServoAngle(kneeChannels[0], kneeLift[0]);  // FL knee bend
  setServoAngle(hipChannels[3], hipForward[3]); // BR hip forward
  setServoAngle(kneeChannels[3], kneeLift[3]);  // BR knee bend
  delay(stepDelay);

  // Lower FL and BR legs with push back
  setServoAngle(hipChannels[0], hipBack[0]);    // FL hip back (push)
  setServoAngle(kneeChannels[0], neutralKnee);  // FL knee straight
  setServoAngle(hipChannels[3], hipBack[3]);    // BR hip back (push)
  setServoAngle(kneeChannels[3], neutralKnee);  // BR knee straight
  delay(stepDelay);

  // Return to neutral
  setServoAngle(hipChannels[0], neutralHip);
  setServoAngle(kneeChannels[0], neutralKnee);
  setServoAngle(hipChannels[3], neutralHip);
  setServoAngle(kneeChannels[3], neutralKnee);
  delay(stepDelay);

  // Step 2: Lift and move front right and back left legs forward
  setServoAngle(hipChannels[1], hipForward[1]); // FR hip forward
  setServoAngle(kneeChannels[1], kneeLift[1]);  // FR knee bend
  setServoAngle(hipChannels[2], hipForward[2]); // BL hip forward
  setServoAngle(kneeChannels[2], kneeLift[2]);  // BL knee bend
  delay(stepDelay);

  // Lower FR and BL legs with push back
  setServoAngle(hipChannels[1], hipBack[1]);    // FR hip back (push)
  setServoAngle(kneeChannels[1], neutralKnee);  // FR knee straight
  setServoAngle(hipChannels[2], hipBack[2]);    // BL hip back (push)
  setServoAngle(kneeChannels[2], neutralKnee);  // BL knee straight
  delay(stepDelay);

  // Return to neutral
  setServoAngle(hipChannels[1], neutralHip);
  setServoAngle(kneeChannels[1], neutralKnee);
  setServoAngle(hipChannels[2], neutralHip);
  setServoAngle(kneeChannels[2], neutralKnee);
  delay(stepDelay);
}
