#define dirPin 26
#define stepPin 28
#define dirPin1 30
#define stepPin1 32
#define stepsPerRevolution 400
#include <Servo.h>

int servoPin=42;
int reading=0;
Servo Servo1;

void setup() {
  Serial.begin(115200);
  // Declare pins as output:
  pinMode(stepPin, OUTPUT); //speed
  pinMode(dirPin, OUTPUT); //direction
  
  // Declare pins as output:
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);

  // Declare servo stuff
  Servo1.attach(servoPin);

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

        String x = a.substring(0, commaIndex);
        String y = a.substring(commaIndex + 1, secondCommaIndex);
        String angle = a.substring(secondCommaIndex + 1);

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

        // SANITY CHECK
        digitalWrite(LED_BUILTIN, LOW);
    }
}
