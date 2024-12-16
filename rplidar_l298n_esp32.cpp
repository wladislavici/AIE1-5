#include <thijs_rplidar.h>

// Motor Driver Pins
#define IN1 5
#define IN2 18
#define IN3 19
#define IN4 21
#define ENA 14 // Must be a PWM-capable pin
#define ENB 27 // Must be a PWM-capable pin

// RPLIDAR Motor Pin
#define RPLIDAR_MOTOR 15

// RPLIDAR and Motor Control Settings
const int MOTOR_SPEED = 200;   // Adjust as needed (0-255)
const int SAFE_DISTANCE_CM = 35; // Safe distance in centimeters (adjust as needed)
const int LIDAR_POLL_INTERVAL = 5; // Adjust as needed

// Initialize the RPLIDAR using UART0 (Serial)
RPlidar lidar(Serial);

void setup() {
  // Initialize Motor Driver Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Initialize the RPLIDAR
  lidar.init(3, 1); // RX on GPIO3, TX on GPIO1 (UART0)
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  analogWrite(RPLIDAR_MOTOR, MOTOR_SPEED);

  delay(1000); // Give the RPLIDAR some time to initialize
}

void loop() {
    // Start the scan
  if (lidar.startStandardScan()) {
      // Set a callback function that will be called for each data point
      lidar.postParseCallback = [](RPlidar* self, uint16_t dist, uint16_t angle_q6, uint8_t newRotFlag, int8_t quality) {
          // Convert angle from 'q6' format (angle * 64) to degrees
          float angle = (float)angle_q6 / 64.0;
          // Convert distance from millimeters to centimeters
          float distance = (float)dist / 10.0;

          // Check if the object is within the safe distance
          if (distance > 0 && distance < SAFE_DISTANCE_CM) {
              // Stop the motors immediately
              stopMotors();
          } else {
              // Move the motors forward
              moveForward();
          }
      };

      // Handle data from the lidar in a loop
      while (true) {
          if (lidar.handleData() < 0) {
              // Handle data error if needed
              break;
          }
          delay(LIDAR_POLL_INTERVAL);
      }

      // Stop the scan when exiting the loop
      lidar.stopScan();
  }
}

void moveForward() {
  // Set motor direction (forward)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  // Enable motors (full speed)
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
}

void stopMotors() {
  // Short brake (both inputs HIGH or LOW)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);

  // Disable motors (optional - coast to stop)
  // analogWrite(ENA, 0);
  // analogWrite(ENB, 0);
}