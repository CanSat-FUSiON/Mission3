#define PIN_IN1 32
#define PIN_IN2 33
#define PIN_IN3 26
#define PIN_IN4 27


void setup() {
  pinMode(PIN_IN1,OUTPUT);
  pinMode(PIN_IN2,OUTPUT);
  pinMode(PIN_IN3,OUTPUT);
  pinMode(PIN_IN4,OUTPUT);
}

void loop() {
  
     digitalWrite(PIN_IN1,HIGH);
     digitalWrite(PIN_IN2,LOW);
     digitalWrite(PIN_IN3,HIGH);
     digitalWrite(PIN_IN4,LOW);

     delay(5000);

     digitalWrite(PIN_IN1,LOW);
     digitalWrite(PIN_IN2,HIGH);
     digitalWrite(PIN_IN3,LOW);
     digitalWrite(PIN_IN4,HIGH);

     delay(1000);
    
     
      
  }
