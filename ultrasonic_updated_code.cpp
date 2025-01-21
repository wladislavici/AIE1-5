// Motor driver pins
const int ENA = 5;  // ENA on L298N
const int IN1 = 4;  // IN1 on L298N
const int IN2 = 3;  // IN2 on L298N
const int ENB = 11; // ENB on L298N
const int IN3 = 7;  // IN3 on L298N
const int IN4 = 6;  // IN4 on L298N

// Ultrasonic sensor pins
const int trigPin = 9;
const int echoPin = 10;

// Variables for distance measurement
long duration;
int distance;

// Speed for forward movement (0-255, higher value for faster speed)
int motorSpeed = 150;

void setup() {
  // Set motor control pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

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
    moveForward();
  }

  delay(100);  // Short delay before checking again
}

// Function to move the car forward
void moveForward() {
  // Rotate motors in forward direction
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, motorSpeed);  // Motor A forward with speed control

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, motorSpeed);  // Motor B forward with speed control
}

// Function to stop the motors
void stopMotors() {
  // Stop both motors (coasting)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, LOW);
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