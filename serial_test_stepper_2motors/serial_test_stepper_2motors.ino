#define dirPin 26
#define stepPin 28
#define dirPin1 30
#define stepPin1 32
#define stepsPerRevolution 400

char token;
const char delimiter =",";
char strings[2];

void setup() {
  Serial.begin(115200);
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  
  // Declare pins as output:
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);

  // Sanity check LED
  pinMode(LED_BUILTIN, OUTPUT);
}
void loop() {
   if (Serial.available())
    {
        String a = Serial.readString();
        int commaIndex = a.indexOf(',');
        //int secondCommaIndex = myString.indexOf(',', commaIndex + 1);

        String x = a.substring(0, commaIndex);
        String y = a.substring(commaIndex + 1);//, secondCommaIndex);

        int rotations = x.toInt();
        int rotations1 = y.toInt();
        
        digitalWrite(LED_BUILTIN, HIGH);
        //delay(1000);
//
//        digitalWrite(dirPin, LOW);
//        // Spin the stepper motor revolutions fast:
//        for (int i = 0; i < rotations * stepsPerRevolution; i++) {
//          // These four lines result in 1 step:
//          digitalWrite(stepPin, HIGH);
//          delayMicroseconds(500);
//          digitalWrite(stepPin, LOW);
//          delayMicroseconds(500);
//        }
//        delay(500);
//        // Set the spinning direction clockwise of MOTOR2:
//        digitalWrite(dirPin1, LOW);
//        // Spin the stepper motor revolutions fast:
//        for (int i = 0; i < rotations1 * stepsPerRevolution; i++) {
//          // These four lines result in 1 step:
//          digitalWrite(stepPin1, HIGH);
//          delayMicroseconds(500);
//          digitalWrite(stepPin1, LOW);
//          delayMicroseconds(500);
//        }
//        delay(500);
        // Set the spinning direction counterclockwise  of MOTOR2:
        digitalWrite(dirPin1, HIGH);
        // Spin the stepper motor revolutions fast:
        for (int i = 0; i < rotations * stepsPerRevolution; i++) {
          // These four lines result in 1 step:
          digitalWrite(stepPin1, HIGH);
          delayMicroseconds(500);
          digitalWrite(stepPin1, LOW);
          delayMicroseconds(500);
        }
        delay(500);
        // Set the spinning direction counterclockwise of MOTOR1:
        digitalWrite(dirPin, HIGH);
        // Spin the stepper motor revolutions fast:
        for (int i = 0; i < rotations1 * stepsPerRevolution; i++) {
          // These four lines result in 1 step:
          digitalWrite(stepPin, HIGH);
          delayMicroseconds(500);
          digitalWrite(stepPin, LOW);
          delayMicroseconds(500);
        }
        delay(500);

        digitalWrite(LED_BUILTIN, LOW);
    }
}
