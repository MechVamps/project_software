#include <Servo.h>
int servoPin=42;
//int end_angle=180;
int reading=0;
Servo Servo1;

void setup() {
  Serial.begin(115200);
  Servo1.attach(servoPin);
  pinMode(LED_BUILTIN, OUTPUT);
}
void loop() {
   if (Serial.available())
    {
        //Servo1.write(0);
        //digitalWrite(LED_BUILTIN, LOW);
        //delay(2000);
        String a = Serial.readString();
        int end_angle = a.toInt();
        Servo1.write(end_angle);
        digitalWrite(LED_BUILTIN, HIGH);
        //delay(1000);
    }
}
