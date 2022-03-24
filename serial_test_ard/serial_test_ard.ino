void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
}
void loop() {
//  if(Serial.available()) {
//    char data = Serial.read();
//    char str[2];
//    str[0] = data;
//    str[1] = '\0';
//    Serial.print(str);
//  }
   if (Serial.available())
    {
        String a = Serial.readString();
        Serial.print("Received Value: ");
        Serial.println(a);
        int b = a.toInt();
        if(b==1){
          digitalWrite(LED_BUILTIN, HIGH);
        }
        if(b==0){
          digitalWrite(LED_BUILTIN, LOW);
        }
    }
}
