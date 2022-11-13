#define PIN_IN1 2
#define PIN_IN2 23

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU9250_asukiaaa.h>
#include <math.h>
#include <TinyGPS++.h>

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif

MPU9250_asukiaaa mySensor;
Adafruit_BME280 bme;
TinyGPSPlus gps;


float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;
float temp = 0;
float pres = 0;
float heig = 0;
float ang = 0;
float lonx = 0;
float latx = 0;
float lony = 0;//ゴールの値
float laty = 0;//ゴールの値
float lona = 0;
float lata = 0;
float x = 10;
float y = -1;
float z = -1;
float msec = 0;



float read_heig(float,float);
float read_ang(float,float,float,float,float,float);
void motor(int, int);


float read_heig(float temp,float pres){
  heig = ((pow(1013/pres,1/5.257)-1)*(temp+273.15))/0.0065; 
  return heig;
  }


float read_ang(float mX,float mY,float lonx,float latx,float lony,float laty){
  ang = atan(mY/mX)-atan(cos(latx)*(lony-lonx)/(laty-latx));
 return ang;
  }


void setup() {

  Serial.begin(115200);
  bool status;
  status = bme.begin(0x76);  
  while (!status) {
    Serial.println("BME280 sensorが使えません");
    delay(1000);
  }
  while(!Serial);
  Serial.println("started");

#ifdef _ESP32_HAL_I2C_H_ // For ESP32
  Wire.begin(SDA_PIN, SCL_PIN);
  mySensor.setWire(&Wire);
#endif

  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();
  pinMode(PIN_IN1,OUTPUT);
  pinMode(PIN_IN2,OUTPUT);
}

void loop() {
  
     digitalWrite(PIN_IN1,HIGH);
     digitalWrite(PIN_IN2,LOW);

     delay(1000);

     digitalWrite(PIN_IN1,LOW);
     digitalWrite(PIN_IN2,HIGH);

     delay(1000);

     lonx = gps.location.lng();
     latx = gps.location.lat();
     mX = mySensor.magX();
     mY = mySensor.magY();
     temp = bme.readTemperature();
     pres = bme.readPressure() / 100.0F;
      Serial.println(lonx);
      Serial.println(latx);
      Serial.println(mX);
      Serial.println(mY);
      Serial.println(temp);
      Serial.println(pres);
      Serial.println(read_heig(temp,pres));

     delay(2000);
     
      
  }
