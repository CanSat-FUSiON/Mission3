void setup() {
  Serial.begin(115200);
  pinMode(12,OUTPUT);

  Serial.println("LOW");
  digitalWrite(12,LOW);
  delay(3000);
  
  Serial.println("HIGH");
  digitalWrite(12,HIGH);
  delay(15000);

  Serial.println("LOW");
  digitalWrite(12,LOW);
}
 
void loop() {
}
