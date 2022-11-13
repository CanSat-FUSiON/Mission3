#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;



float temp = 0;
float pres = 0;
float heig = 0;

float read_heig(float temp,float pres){
  heig = ((pow(1013.84/pres,1/5.257)-1)*(temp+273.15))/0.0065; 
  return heig;
  }

void setup() {
  Serial.begin(115200);
  bool status;
  status = bme.begin(0x76);  
  while (!status) {
    Serial.println("BME280 sensorが使えません");
    delay(1000);
  }
}
void loop() { 
  temp=bme.readTemperature();
  pres=bme.readPressure() / 100.0F;
  
  Serial.print("温度 ;");
  Serial.print(temp);
  Serial.println(" °C");
   
  Serial.print("気圧 ;");
  Serial.print(pres);
  Serial.println(" hPa");

  Serial.println(read_heig(temp,pres));
  Serial.println();
  delay(1000);
}
