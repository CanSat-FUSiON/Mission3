#include <MPU9250_asukiaaa.h>

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif

MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ, ang;

float mXoffset = 0;
float mYoffset = 0;
float mZoffset = 0;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("started");

#ifdef _ESP32_HAL_I2C_H_ // For ESP32
  Wire.begin(SDA_PIN, SCL_PIN);
  mySensor.setWire(&Wire);
#endif

  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  // You can set your own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
  // mySensor.magZOffset = -10;
}

void loop() {
  uint8_t sensorId;
  if (mySensor.readId(&sensorId) == 0) {
    Serial.println("sensorId: " + String(sensorId));
  } else {
    Serial.println("Cannot read sensorId");
  }

  
  if (mySensor.magUpdate() == 0) {
    mX = mySensor.magX()-mXoffset;
    mY = mySensor.magY()-mYoffset;
    mZ = mySensor.magZ()-mZoffset;
    
    if (mX,mY>0){
      ang = (atan(mX/mY))*180/3.1415;
    }else{
      if(mX>0 && mY<0){
        ang = 180 + (atan(mX/mY))*180/3.1415;
      }else{
        if(mX<0 && mY<0){
          ang = 180 + (atan(mX/mY))*180/3.1415;
        }
        }
      }
    if (mX<0 && mY>0){
      ang = 360 + (atan(mX/mY))*180/3.1415;
    }
    
    Serial.println(mX);
    Serial.println(mY);
    Serial.println(ang);

  }else{
    Serial.println("Cannot read mag values");
  

}
delay(1000);
}
