#include <Servo.h>
// Define pin connections
int rainSensorPin = A0;      // Rain sensor analog input
int moistureSensorPin = A1;  // Moisture sensor analog input
int motorDriverPinA = 2;     // Motor driver pin 1 
int motorDriverPinB = 4;     // Motor driver pin 2
int motorDriverPinC = 7;     // Motor driver pin 3
int motorDriverPinD = 8;     // Motor driver pin 4
int enableAPin = 11;         // Motor driver enable A
int enableBPin = 3;          // Motor driver enable B
int servoPin = 10;           // Servo motor pin

// Define Servo motor
Servo servo;

// Define threshold values
int heavyRainThreshold = 400;
int moderateRainThreshold = 700;
int drySoilThreshold = 900;

void setup() {
  Serial.begin(9600);
  pinMode(motorDriverPinA, OUTPUT);
  pinMode(motorDriverPinB, OUTPUT);
  pinMode(motorDriverPinC, OUTPUT);
  pinMode(motorDriverPinD, OUTPUT);
  pinMode(enableAPin, OUTPUT);
  pinMode(enableBPin, OUTPUT);
  // Analog pins are inputs by default, but explicitly setting is fine
  pinMode(rainSensorPin, INPUT);
  pinMode(moistureSensorPin, INPUT);
  servo.attach(servoPin);
  servo.write(0);
  stopMotor(); // Start with the motor stopped
}

void loop() {
  // Read sensor values at the beginning of the loop
  int rainSensorValue = analogRead(rainSensorPin);
  int moistureSensorValue = analogRead(moistureSensorPin);

  // Print sensor values to the Serial Monitor
  Serial.print("Rain sensor value: ");
  Serial.println(rainSensorValue);
  Serial.print("Moisture sensor value: ");
  Serial.println(moistureSensorValue);

  // Check rain sensor value and control motor
  if (rainSensorValue < heavyRainThreshold) {
    // Heavy rain, cover the field
    Serial.println("Heavy rain, covering the field...");
    moveMotorBackward(); // Assuming backward covers the field
    stopMotor();
  } else if (rainSensorValue < moderateRainThreshold) {
    // Moderate rain, do nothing
    Serial.println("Moderate rain, no action.");
    stopMotor();
  } else {
    // No rain, open the field
    Serial.println("No rain, opening the field...");
    moveMotorForward(); // Assuming forward opens the field
    stopMotor();
  }
  
  // Check moisture sensor value and control servo motor
  if (moistureSensorValue > drySoilThreshold) {
    Serial.println("Soil is dry. Watering...");
    servo.write(180); // Open valve
  } else {
    Serial.println("Soil is moist. No water needed.");
    servo.write(0); // Close valve
  }

  delay(2000); // Wait for 2 seconds before the next reading
}

void moveMotorForward() {
  Serial.println("Motor moving forward (opening roof).");
  digitalWrite(motorDriverPinA, HIGH);
  digitalWrite(motorDriverPinB, LOW);
  digitalWrite(motorDriverPinC, LOW);
  digitalWrite(motorDriverPinD, HIGH);
  analogWrite(enableAPin, 200); // Set motor speed
  analogWrite(enableBPin, 200);
  delay(2000); // Run motor for a set duration
}

void moveMotorBackward() {
  Serial.println("Motor moving backward (closing roof).");
  digitalWrite(motorDriverPinA, LOW);
  digitalWrite(motorDriverPinB, HIGH);
  digitalWrite(motorDriverPinC, HIGH);
  digitalWrite(motorDriverPinD, LOW);
  analogWrite(enableAPin, 200); // Set motor speed
  analogWrite(enableBPin, 200);
  delay(2000); // Run motor for a set duration
}

void stopMotor() {
  digitalWrite(motorDriverPinA, LOW);
  digitalWrite(motorDriverPinB, LOW);
  digitalWrite(motorDriverPinC, LOW);
  digitalWrite(motorDriverPinD, LOW);
  analogWrite(enableAPin, 0); // Stop motor
  analogWrite(enableBPin, 0);
}


