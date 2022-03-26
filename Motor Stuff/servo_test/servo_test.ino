#include <Servo.h>
int servoPin=3;
//int end_angle=180;
String a = "180";//Serial.readString();
int end_angle = a.toInt();
int reading=0;
Servo Servo1;
void setup(){
Servo1.attach(servoPin);
pinMode(LED_BUILTIN,OUTPUT);
}
void loop(){
//reading = 500;
//angle = map(reading, 0, 1023, 0, 180);
Servo1.write(0);
digitalWrite(LED_BUILTIN, LOW);
delay(2000);
Servo1.write(end_angle);
digitalWrite(LED_BUILTIN, HIGH);
delay(1000);
digitalWrite(LED_BUILTIN, LOW);
//Servo1.write(0);
delay(2000);
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
