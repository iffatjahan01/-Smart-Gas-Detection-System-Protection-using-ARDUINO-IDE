#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Initialize the LCD with the detected I2C address (0x3F)
LiquidCrystal_I2C lcd(0x3F, 16, 2);

// Sensor and actuator pins
const int MQ2pin = A3;             // Gas sensor pin
const int flameSensorPin = A0;     // Flame sensor pin
const int buzzerPin = 12;          // Buzzer pin
const int ledPin = 13;             // LED pin

// Motor driver pins
const int IN1 = 5;  // Direction pin 1 for Motor 1 (pump)
const int IN2 = 9;  // Direction pin 2 for Motor 1
const int IN3 = 3;  // Direction pin 1 for Motor 2 (exhaust fan)
const int IN4 = 11; // Direction pin 2 for Motor 2

// I2C Address for Arduino Nano (Slave)
const int slaveAddress = 8;  // Address of the Arduino Nano

// Variables for blinking behavior on LCD
int currentState = 0;              // State variable for LCD timing
unsigned long previousMillis = 0;  // Timer for LCD timing
unsigned long buzzerMillis = 0;    // Timer for buzzer timing
bool buzzerState = false;          // Current state of the buzzer

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);

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
  lcd.init();
  lcd.backlight();

  // Start I2C as slave with address 8
  Wire.begin(slaveAddress);
  Wire.onRequest(requestEvent); // Register the requestEvent function
}

void loop() {
  // Nothing to do here, just waiting for requests from ESP32
}

void requestEvent() {
  int gasSensorValue = analogRead(MQ2pin);
  int flameSensorValue = analogRead(flameSensorPin);

  // Send sensor data to ESP32 (2 bytes for each sensor, gas and flame)
  Wire.write(gasSensorValue >> 8);  // Send high byte of gas value
  Wire.write(gasSensorValue & 0xFF); // Send low byte of gas value
  Wire.write(flameSensorValue >> 8);  // Send high byte of flame value
  Wire.write(flameSensorValue & 0xFF); // Send low byte of flame value
}
