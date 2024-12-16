#include <thijs_rplidar.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define RPLIDAR_MOTOR 15 // Example PWM pin - choose an available one (that's not 1 or 3)

// I2C address of your LCD1602 (use I2C scanner sketch if you're not sure)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Initialize the RPLIDAR using UART0 (Serial)
RPlidar lidar(Serial);

const int MOTOR_SPEED = 200;   // Adjust as needed (0-255)
const int DISPLAY_DELAY = 10; // Delay in milliseconds after updating the LCD (tune as needed)

void setup() {
  // Initialize the LCD
  Wire.begin();
  lcd.init(); // Or lcd.begin(16, 2);
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("RPLIDAR Data");

  // Initialize the RPLIDAR
  lidar.init(3, 1); // RX on GPIO3, TX on GPIO1 (UART0)
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  analogWrite(RPLIDAR_MOTOR, MOTOR_SPEED);

  delay(1000); // Give the RPLIDAR some time to initialize
  lcd.clear();

  // Print static text only once
  lcd.setCursor(0, 0);
  lcd.print("Ang:");
  lcd.setCursor(0, 1);
  lcd.print("Dis:");
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

      // Update LCD only if values have changed
      static float lastDisplayedAngle = -1.0;
      static float lastDisplayedDistance = -1.0;

      if (angle != lastDisplayedAngle) {
        lcd.setCursor(4, 0);
        lcd.print("      "); // Clear previous angle
        lcd.setCursor(4, 0);
        lcd.print(angle, 0); // Print angle with 0 decimal places
        lcd.print("deg");
        lastDisplayedAngle = angle;
      }

      if (distance != lastDisplayedDistance) {
        lcd.setCursor(4, 1);
        lcd.print("      "); // Clear previous distance
        lcd.setCursor(4, 1);
        lcd.print(distance, 0); // Print distance with 0 decimal places
        lcd.print("cm");
        lastDisplayedDistance = distance;
      }
    };

    // Handle data from the lidar in a loop
    while (true) {
      if (lidar.handleData() >= 0) {
        delay(DISPLAY_DELAY); // Small delay to avoid overwhelming the LCD
      } else {
        // Handle data error if needed
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Data Error");
        break;
      }
    }

    // Stop the scan when exiting the loop
    lidar.stopScan();
  } else {
    // Handle scan start error if needed
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Scan Start Error");
  }
}