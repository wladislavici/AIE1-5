// Motor driver pins
const int motorAForward = 4;
const int motorABackward = 5;
const int motorBForward = 6;
const int motorBBackward = 7;

// Ultrasonic sensor pins
const int trigPin = 9;
const int echoPin = 10;

// Variables for distance measurement
long duration;
int distance;

// Speed for forward movement (higher value for faster speed)
int motorSpeed = 150;

void setup() {
  // Set motor control pins as outputs
  pinMode(motorAForward, OUTPUT);
  pinMode(motorABackward, OUTPUT);
  pinMode(motorBForward, OUTPUT);
  pinMode(motorBBackward, OUTPUT);
  
  // Set ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Start serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Measure the distance from the ultrasonic sensor
  distance = getDistance();
  
  // Print the distance to the Serial Monitor for debugging
  Serial.print("Distance: ");
  Serial.println(distance);
  
  // If distance is below the threshold, stop the motors
  if (distance < 10) {  // 10 cm is the minimum threshold
    stopMotors();
  } else {
    moveForward();  // Change this to move forward instead of backward
  }
  
  delay(100);  // Short delay before checking again
}

// Function to move the car forward
void moveForward() {
  // Rotate motors in forward direction
  analogWrite(motorAForward, motorSpeed);   // Motor A forward
  analogWrite(motorABackward, LOW);         // Motor A backward off
  
  analogWrite(motorBForward, motorSpeed);   // Motor B forward
  analogWrite(motorBBackward, LOW);         // Motor B backward off
}

// Function to stop the motors
void stopMotors() {
  // Stop both motors
  analogWrite(motorABackward, LOW);
  analogWrite(motorAForward, LOW);
  
  analogWrite(motorBBackward, LOW);
  analogWrite(motorBForward, LOW);
}

// Function to calculate distance using the ultrasonic sensor
long getDistance() {
  // Send a pulse to trigger the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Measure the pulse duration on Echo pin
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance (in cm)
  long distance = duration * 0.034 / 2;
  
  return distance;
}
