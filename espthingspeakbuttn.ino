#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <ThingSpeak.h>

// Initialize the LCD with the detected I2C address (0x3F)
LiquidCrystal_PCF8574 lcd(0x3F); // Replace 0x3F with your LCD's I2C address if different
Servo s1;

// Sensor and actuator pins
const int MQ2pin = 32;             // Gas sensor pin (ADC1_4)
const int flameSensorPin = 33;     // Flame sensor pin (ADC1_5)
const int buzzerPin = 25;          // Buzzer pin  
const int ledPin = 2;              // LED pin (Built-in LED on some ESP32 boards)

// Motor driver pins
const int IN1 = 26;  // Direction pin 1 for Motor 1 (pump)
const int IN2 = 27;  // Direction pin 2 for Motor 1
const int IN3 = 14;  // Direction pin 1 for Motor 2 (exhaust fan)
const int IN4 = 12;  // Direction pin 2 for Motor 2

// Wi-Fi and ThingSpeak configuration
const char *ssid = "Cuties";           // Your Wi-Fi SSID
const char *password = "soma2100606";  // Your Wi-Fi Password
unsigned long myChannelNumber =  2863341; // Your ThingSpeak channel number
const char *myWriteAPIKey = "PABDVCG239UFORMG"; // ThingSpeak Write API Key

WiFiClient client;

// Threshold values
const int GAS_THRESHOLD = 1000;     // Gas detection threshold
const int FLAME_THRESHOLD = 4000;   // Flame detection threshold

// Variables for blinking behavior
int currentState = 0;              // State variable for LCD timing
unsigned long previousMillis = 0;  // Timer for LCD timing
unsigned long buzzerMillis = 0;    // Timer for buzzer timing
bool buzzerState = false;          // Current state of the buzzer

// Field number for manual control of the servo
const int controlField = 3; // Field 3 for controlling the servo manually

// Setup function
void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);

  // Initialize Servo
  s1.attach(13); // Attach servo to GPIO13
  s1.write(0); // Start at 0 degree position (safe state)

  // Initialize pins
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  // Motor pins as output
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set initial states for actuators
  digitalWrite(ledPin, LOW);
  digitalWrite(buzzerPin, LOW);

  // Stop both motors initially
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Initialize LCD
  lcd.begin(16, 2); // Specify 16 columns and 2 rows
  lcd.setBacklight(255); // Turn on backlight

  // Connect to Wi-Fi
  connectToWiFi();

  // Initialize ThingSpeak
  ThingSpeak.begin(client);
}

// Function to handle the passive buzzer with alternating tones
void buzzerAlert() {
  unsigned long currentMillis = millis();
  if (currentMillis - buzzerMillis >= 500) { // Switch tone every 500ms
    buzzerMillis = currentMillis;
    if (buzzerState) {
      tone(buzzerPin, 1400); // Play 700Hz tone
    } else {
      tone(buzzerPin, 1800); // Play 800Hz tone
    }
    buzzerState = !buzzerState; // Toggle state
  }
}

// Function to stop the passive buzzer
void stopBuzzer() {
  noTone(buzzerPin); // Stop tone
}

// Function to control Motor 1 (pump)
void controlPumpMotor(bool state) {
  if (state) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}

// Function to control Motor 2 (exhaust fan)
void controlFanMotor(bool state) {
  if (state) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
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

  // Update ThingSpeak with sensor values
  ThingSpeak.setField(1, gasSensorValue);    // Field 1 for Gas Sensor
  ThingSpeak.setField(2, flameSensorValue);  // Field 2 for Flame Sensor
  
  // Read the control button state from Field 3
  long controlValue = ThingSpeak.readField(myChannelNumber, controlField); // Field 3

  // Write to ThingSpeak
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if (x == 200) {
    Serial.println("Data updated successfully.");
  } else {
    Serial.println("Error updating data. HTTP error code: " + String(x));
  }

  // Check the control value and update the servo position
  if (controlValue == 1) {
    s1.write(180); // Turn servo to 180 degrees (ON)
  } else if (controlValue == 0) {
    s1.write(0);   // Turn servo to 0 degrees (OFF)
  }

  // Determine system state based on sensor readings
  bool gasDetected = gasSensorValue >= GAS_THRESHOLD;
  bool flameDetected = flameSensorValue <= FLAME_THRESHOLD;

  if (gasDetected || flameDetected) {
    // Gas or flame detected (or both)
    handleLCDTiming("WARNING! Gas/Fire", "     Alert!     ");
    s1.write(180); // Move servo to 180 degrees (danger state)
    controlFanMotor(true); // Turn on exhaust fan
    controlPumpMotor(true); // Turn on pump
    digitalWrite(ledPin, HIGH);
    buzzerAlert(); // Activate passive buzzer
  } else {
    // Nothing detected
    lcd.setCursor(0, 0);
    lcd.print("Everything is   ");
    lcd.setCursor(0, 1);
    lcd.print("SAFE and Fine   ");
    controlFanMotor(false); // Turn off exhaust fan
    controlPumpMotor(false); // Turn off pump
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

// Function to connect to Wi-Fi
void connectToWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi!");
}
