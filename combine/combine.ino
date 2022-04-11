#define dirPin 26
#define stepPin 28
#define dirPin1 30
#define stepPin1 32
#define stepsPerRevolution 400
#include <Servo.h>

// SERVO
int servoPin=42;
int reading=0;
Servo Servo1;

//LINEAR ACTUATOR
const int ENA_PIN = 40; // the Arduino pin connected to the EN1 pin L298N
const int IN1_PIN = 38; // the Arduino pin connected to the IN1 pin L298N
const int IN2_PIN = 36; // the Arduino pin connected to the IN2 pin L298N

void setup() {
  Serial.begin(115200);
  // MOTOR OUTPUT PINS
  pinMode(stepPin, OUTPUT); //speed motor 1
  pinMode(dirPin, OUTPUT); //direction motor 1
  pinMode(stepPin1, OUTPUT); //speed motor 2
  pinMode(dirPin1, OUTPUT); //direction motor 2

  // ATTACH SERVO stuff
  Servo1.attach(servoPin);

  // LINEAR ACTUATOR OUTPUT PINS
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  digitalWrite(ENA_PIN, HIGH); //Driver output pin. On driver it's ENCA

  // Sanity check LED
  pinMode(LED_BUILTIN, OUTPUT);
}
void loop() {
   if (Serial.available())
    {
        // SERIAL READ
        String a = Serial.readString();
        int commaIndex = a.indexOf(',');
        int secondCommaIndex = a.indexOf(';');
        int thirdCommaIndex = a.indexOf('?');

        String x = a.substring(0, commaIndex);
        String y = a.substring(commaIndex + 1, secondCommaIndex);
        String angle = a.substring(secondCommaIndex + 1, thirdCommaIndex);
        String inject = a.substring(thirdCommaIndex);

        int rotations = x.toInt();
        int rotations1 = y.toInt();
        int end_angle = angle.toInt();

        // MOTOR DIRECTION
        if(rotations >= 0){
          digitalWrite(dirPin, LOW);
        }
        if(rotations < 0){
          digitalWrite(dirPin, LOW);
          rotations = abs(rotations);
        }
        
        if(rotations1 >= 0){
          digitalWrite(dirPin1, LOW);
        }
        if(rotations1 < 0){
          digitalWrite(dirPin1, LOW);
          rotations1 = abs(rotations1);
        }

        // SANITY CHECK
        digitalWrite(LED_BUILTIN, HIGH);

        // MOTOR 1 move
        // Spin the stepper motor revolutions fast:
        for (int i = 0; i < rotations1 * stepsPerRevolution; i++) {
          // These four lines result in 1 step:
          digitalWrite(stepPin, HIGH);
          delayMicroseconds(500);
          digitalWrite(stepPin, LOW);
          delayMicroseconds(500);
        }
        delay(500);

        // MOTOR 2 move
        // Spin the stepper motor revolutions fast:
        for (int i = 0; i < rotations * stepsPerRevolution; i++) {
          // These four lines result in 1 step:
          digitalWrite(stepPin1, HIGH);
          delayMicroseconds(500);
          digitalWrite(stepPin1, LOW);
          delayMicroseconds(500);
        }
        delay(500);

        // SERVO MOVE
        Servo1.write(end_angle);
        delay(500);

        // LINEAR ACTUATOR MOVE
        if(inject.equals('push')){
          // retracts the actuator
          digitalWrite(IN1_PIN, LOW);
          digitalWrite(IN2_PIN, HIGH);
          delay(1000);
        }
        if(inject.equals('pull')){
          // extend the actuator
          digitalWrite(IN1_PIN, HIGH);
          digitalWrite(IN2_PIN, LOW);
          delay(1000);
        }
        // SANITY CHECK
        digitalWrite(LED_BUILTIN, LOW);
    }
}
