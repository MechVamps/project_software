#define dirPin 28
#define stepPin 30
#define stepsPerRevolution 400

void setup() {
  Serial.begin(115200);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}
void loop() {
   if (Serial.available())
    {
        //Servo1.write(0);
        //digitalWrite(LED_BUILTIN, LOW);
        //delay(2000);
        String a = Serial.readString();
        int rotations = a.toInt();
        digitalWrite(LED_BUILTIN, HIGH);
        //delay(1000);

        digitalWrite(dirPin, HIGH);

        // Spin the stepper motor 1 revolution quickly:
        for (int i = 0; i < rotations*stepsPerRevolution; i++) {
          // These four lines result in 1 step:
          digitalWrite(stepPin, HIGH);
          delayMicroseconds(1000);
          digitalWrite(stepPin, LOW);
          delayMicroseconds(1000);
        }
      
        delay(2000);
    }
}
