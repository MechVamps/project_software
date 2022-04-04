// Servo Motor
#include <Servo.h>
int servoPin=9;
int Led=2;
int end_angle=30;
int reading=0;
Servo Servo1;

//Linear Actuator
const int ENA_PIN = 40; // the Arduino pin connected to the EN1 pin L298N
const int IN1_PIN = 38; // the Arduino pin connected to the IN1 pin L298N
const int IN2_PIN = 36; // the Arduino pin connected to the IN2 pin L298N

void setup(){
Servo1.attach(servoPin);
pinMode(Led,OUTPUT);

// Linear Actuator.
pinMode(ENA_PIN, OUTPUT);
pinMode(IN1_PIN, OUTPUT);
pinMode(IN2_PIN, OUTPUT);

digitalWrite(ENA_PIN, HIGH);
}

void loop(){
//reading = 500;
//angle = map(reading, 0, 1023, 0, 180);
Servo1.write(0);
digitalWrite(Led, LOW);
delay(2000);
Servo1.write(end_angle);
digitalWrite(Led, HIGH);
delay(1000);
digitalWrite(Led, LOW);
//Servo1.write(0);
delay(2000);

//Linear Actuator
// extend the actuator
digitalWrite(IN1_PIN, HIGH);
digitalWrite(IN2_PIN, LOW);

delay(7000); // actuator will stop extending automatically when reaching the limit

// retracts the actuator
digitalWrite(IN1_PIN, LOW);
digitalWrite(IN2_PIN, HIGH);

delay(5000); // actuator will stop retracting automatically when reaching the limit
//exit(0);
}

//
// // Sweep from 0 to 180 degrees:
//  for (angle = 0; angle <= 180; angle += 1) {
//    Servo1.write(angle);
//    delay(15);
//  }
//  // And back from 180 to 0 degrees:
//  for (angle = 180; angle >= 0; angle -= 1) {
//    Servo1.write(angle);
//    delay(30);
//  }
//}
