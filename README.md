# Auotmatic-roofing-system

#include <Servo.h>
// Define pin connections
 int rainSensorPin =  A0; // Rain sensor analog input
 int moistureSensorPin = A1; // Moisture sensor analog input
 int motorDriverPinA = 2;  // Motor driver pin 1 (forward)
 int motorDriverPinB = 4;
 int motorDriverPinC = 7;
 int motorDriverPinD = 8;
int enableAPin = 11;
int enableBPin = 3;
int servoPin= 10 ;
// Define Servo motor
Servo servo;

// Define threshold values for rain sensor
 int heavyRainThreshold = 350;
 int moderateRainThreshold = 500;
void setup() {
  Serial.begin(9600);
  pinMode(motorDriverPinA, OUTPUT);
  pinMode(motorDriverPinB, OUTPUT);
  pinMode(motorDriverPinC,OUTPUT);
  pinMode(motorDriverPinD,OUTPUT);
  pinMode(enableAPin,OUTPUT);
  pinMode(enableBPin,OUTPUT);
  pinMode(rainSensorPin, INPUT);
  pinMode(moistureSensorPin, INPUT);
  servo.attach(servoPin);
  servo.write(0);
 stopMotor(); // Stop the motor initially
}

void loop() {
  int rainSensorValue = analogRead(rainSensorPin);  // Read rain sensor value
  int moistureSensorValue = analogRead(moistureSensorPin); // Read moisture sensor value
  Serial.print("Rain sensor value: ");
 Serial.println(rainSensorValue);
  Serial.print("Moisture sensor value: ");
  Serial.println(moistureSensorValue);
 delay(5000);
  // Check rain sensor value and control motor
  if (rainSensorValue && moistureSensorValue < heavyRainThreshold) 
  {
    // Heavy rain, cover the field
    Serial.println("Heavy rain, covering the field");
    moveMotorForward();
    delay(5000);
    stopMotor();
    //delay(5000);
  } 
  else if ( heavyRainThreshold < rainSensorValue && moistureSensorValue > moderateRainThreshold) 
  {
    // Moderate rain, do nothing
    Serial.println("Moderate rain, no action");
    delay(5000);
    stopMotor();
  //delay(5000);
  }
  else
  { 
    // No rain, open the field
    Serial.println("No rain, opening the field"); 
    moveMotorBackward();
    delay(5000);
    stopMotor();
  } 

  // Check moisture sensor value and control servo motor

  if (moistureSensorValue > 800)
  {

   // Serial.println("Soil is dry. Watering...");
    servo.write(180);
  } 
  else
  {
    
    Serial.println("Soil is wet. No water needed.");
    servo.write(0);
  }

}

void moveMotorForward() 
{
  Serial.println("hi");
  digitalWrite(motorDriverPinA,HIGH);
  digitalWrite(motorDriverPinB,LOW);
  digitalWrite(motorDriverPinC,LOW);
  digitalWrite(motorDriverPinD,HIGH);
  analogWrite(enableAPin, 200); // Full speed forward (255 is maximum speed)
  analogWrite(enableBPin, 200);
  delay(5000); // Open roof for 5 seconds
} 

void moveMotorBackward() {
  digitalWrite(motorDriverPinA,LOW);
  digitalWrite(motorDriverPinB,HIGH);
  digitalWrite(motorDriverPinC,HIGH);
  digitalWrite(motorDriverPinD,LOW);
  analogWrite(enableAPin, 200); 
  analogWrite(enableBPin, 200);
  delay(5000); // Open roof for 5 seconds
}


void stopMotor() {
  digitalWrite(motorDriverPinA, LOW);
  digitalWrite(motorDriverPinB, LOW);
  digitalWrite(motorDriverPinC, LOW);
  digitalWrite(motorDriverPinD, LOW);
  analogWrite(enableAPin, 0); 
  analogWrite(enableBPin, 0);
}
