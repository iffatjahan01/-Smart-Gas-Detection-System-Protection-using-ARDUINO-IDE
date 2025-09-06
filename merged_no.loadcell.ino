#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// Initialize the LCD with the detected I2C address (0x3F)
LiquidCrystal_I2C lcd(0x3F, 16, 2);
Servo s1;

// Sensor and actuator pins
const int MQ2pin = A3;             // Gas sensor pin
const int flameSensorPin = A0;     // Flame sensor pin
const int buzzerPin = 12;          // Buzzer pin
const int ledPin = 13;             // LED pin
const int fanPin = 6;              // Fan PWM pin
const int pumpPin = 5;             // Pump motor pin

// Threshold values
const int GAS_THRESHOLD = 350;     // Gas detection threshold
const int FLAME_THRESHOLD = 500;   // Flame detection threshold

// Variables for blinking behavior
int currentState = 0;              // State variable for LCD timing
unsigned long previousMillis = 0;  // Timer for LCD timing
unsigned long buzzerMillis = 0;    // Timer for buzzer timing
bool buzzerState = false;          // Current state of the buzzer

// Setup function
void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);

  // Initialize Servo
  s1.attach(10);
  s1.write(0);

  // Initialize pins
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(fanPin, OUTPUT);
  pinMode(pumpPin, OUTPUT);

  // Set initial states for actuators
  digitalWrite(ledPin, LOW);
  digitalWrite(buzzerPin, LOW);
  analogWrite(fanPin, 0);
  analogWrite(pumpPin, 0);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
}

// Function to handle the passive buzzer with alternating tones
void buzzerAlert() {
  unsigned long currentMillis = millis();
  if (currentMillis - buzzerMillis >= 500) { // Switch tone every 500ms
    buzzerMillis = currentMillis;
    if (buzzerState) {
      tone(buzzerPin, 1500); // Play 1500Hz tone
    } else {
      tone(buzzerPin, 2000); // Play 2000Hz tone
    }
    buzzerState = !buzzerState; // Toggle state
  }
}

// Function to stop the passive buzzer
void stopBuzzer() {
  noTone(buzzerPin); // Turn off the buzzer
}

// Loop function
void loop() {
  // Read sensor values
  float gasSensorValue = analogRead(MQ2pin);
  float flameSensorValue = analogRead(flameSensorPin);

  // Log sensor values to Serial Monitor
  Serial.print("Gas Sensor: ");
  Serial.print(gasSensorValue);
  Serial.print("  Flame Sensor: ");
  Serial.println(flameSensorValue);

  // Determine system state based on sensor readings
  bool gasDetected = gasSensorValue >= GAS_THRESHOLD;
  bool flameDetected = flameSensorValue <= FLAME_THRESHOLD;

  if (gasDetected && flameDetected) {
    // Both gas and flame detected
    handleLCDTiming("WARNING:Gas,Fire", "     Alert!    ");
    s1.write(180);
    analogWrite(fanPin, 255);
    analogWrite(pumpPin, 255);
    digitalWrite(ledPin, HIGH);
    buzzerAlert(); // Activate passive buzzer
  } else if (gasDetected) {
    // Gas detected
    handleLCDTiming("    WARNING!    ", "Gas Leakage   ");
    s1.write(180);
    analogWrite(fanPin, 255);
    digitalWrite(ledPin, HIGH);
    buzzerAlert(); // Activate passive buzzer
  } else if (flameDetected) {
    // Flame detected
    handleLCDTiming("    WARNING!    ", "   Fire Alert  ");
    s1.write(180);
    analogWrite(pumpPin, 255);
    digitalWrite(ledPin, HIGH);
    buzzerAlert(); // Activate passive buzzer
  } else {
    // Nothing detected
    lcd.setCursor(0, 0);
    lcd.print("Everything is   ");
    lcd.setCursor(0, 1);
    lcd.print("SAFE and Fine   ");
    s1.write(0);
    analogWrite(fanPin, 0);
    analogWrite(pumpPin, 0);
    digitalWrite(ledPin, LOW);
    stopBuzzer(); // Turn off the buzzer
    delay(500); // Keep the screen stable
  }

  delay(100); // Small delay for stability
}

// Function to handle first-row blinking/timing behavior
void handleLCDTiming(String firstRow, String secondRow) {
  unsigned long currentMillis = millis();

  switch (currentState) {
    case 0: // Show the first row for 1000 ms
      lcd.setCursor(0, 0);
      lcd.print(firstRow);
      lcd.setCursor(0, 1);
      lcd.print(secondRow);
      if (currentMillis - previousMillis >= 1000) { // Wait for 1000 ms
        previousMillis = currentMillis;
        lcd.setCursor(0, 0);
        lcd.print("                "); // Clear the first row
        currentState = 1;
      }
      break;

    case 1: // Turn off the first row for 300 ms
      if (currentMillis - previousMillis >= 300) { // Wait for 300 ms
        previousMillis = currentMillis;
        currentState = 2;
      }
      break;

    case 2: // Show the first row for 450 ms
      lcd.setCursor(0, 0);
      lcd.print(firstRow);
      lcd.setCursor(0, 1);
      lcd.print(secondRow);
      if (currentMillis - previousMillis >= 450) { // Wait for 450 ms
        previousMillis = currentMillis;
        lcd.setCursor(0, 0);
        lcd.print("                "); // Clear the first row
        currentState = 3;
      }
      break;

    case 3: // Turn off the first row for 200 ms
      if (currentMillis - previousMillis >= 200) { // Wait for 200 ms
        previousMillis = currentMillis;
        currentState = 4;
      }
      break;

    case 4: // Show the first row for 500 ms
      lcd.setCursor(0, 0);
      lcd.print(firstRow);
      lcd.setCursor(0, 1);
      lcd.print(secondRow);
      if (currentMillis - previousMillis >= 500) { // Wait for 500 ms
        previousMillis = currentMillis;
        lcd.setCursor(0, 0);
        lcd.print("                "); // Clear the first row
        currentState = 5;
      }
      break;

    case 5: // Turn off the first row for 300 ms
      if (currentMillis - previousMillis >= 300) { // Wait for 300 ms
        previousMillis = currentMillis;
        currentState = 0; // Reset to the first state
      }
      break;
  }
}
