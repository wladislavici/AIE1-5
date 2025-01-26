#include <RPLidar.h>

// Create an instance of the RPLidar driver
RPLidar lidar;

// Pin definitions for L298N motor driver
#define ENA 5  // Enable pin for Motor A (PWM)
#define IN1 8  // Input 1 for Motor A
#define IN2 7  // Input 2 for Motor A
#define ENB 6  // Enable pin for Motor B (PWM)
#define IN3 9  // Input 1 for Motor B
#define IN4 10 // Input 2 for Motor B
// RPLIDAR motor control pin
#define RPLIDAR_MOTOR 3 // PWM pin for RPLIDAR's motor speed control

void setup() {
  // Bind the RPLIDAR driver to the Arduino hardware serial
  lidar.begin(Serial);

  // Set pin modes for L298N motor driver
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set pin mode for RPLIDAR motor control
  pinMode(RPLIDAR_MOTOR, OUTPUT);

  // Initialize motor control pins
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  // Start RPLIDAR motor
  analogWrite(RPLIDAR_MOTOR, 255);
}

void loop() {
  if (IS_OK(lidar.waitPoint())) {
    // Perform data processing here...
    float distance = lidar.getCurrentPoint().distance;
    float angle = lidar.getCurrentPoint().angle;

    // Check if the object is within the 330° to 30° FOV and within 10 cm
    if (angle >= 330 || angle <= 30) {
      if (distance <= 0.10) {
      // Object detected within 10 M in the FOV, brake the motor
        brakeMotor();
      } else {
        // No object detected, continue moving
        moveMotorForward();
      }
    }
  } else {
    // Stop the RPLIDAR motor if no data is received
    analogWrite(RPLIDAR_MOTOR, 0);

    // Try to detect RPLIDAR...
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
      // Detected, start scanning again
      lidar.startScan();
      analogWrite(RPLIDAR_MOTOR, 255);
      delay(1000);
    }
  }
}

void moveMotorForward() {
  // Set motor to move forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
}

void brakeMotor() {
  // Brake the motor (stop immediately)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 255); // Full brake
  analogWrite(ENB, 255); // Full brake
}