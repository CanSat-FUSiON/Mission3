#include <Wire.h>
 
#define LPS25H_ADDRESS  0x5c /* SA0 -> GND */
#define LPS25H_WHO_AM_I     0x0f
#define LPS25H_CTRL_REG1    0x20
#define LPS25H_PRESS_OUT_XL 0x28
#define LPS25H_PRESS_OUT_L  0x29
#define LPS25H_PRESS_OUT_H  0x2a
#define LPS25H_TEMP_OUT_L   0x2b
#define LPS25H_TEMP_OUT_H   0x2c

//volatile float altitude;//[m]換算した高度
volatile float gnd_T = 0;//地面の高度入れる
volatile float referencePressure = 1026.26;//高度計算に使用する基準高度での大気圧[hPa]。ここは随時変える。

void setup(void)
{
  Serial.begin(9600);
  pinMode(12,OUTPUT);
  
  Serial.begin(115200);

  Wire.begin();
   
  Serial.print("WHO_AM_I = 0x");
  Serial.println(whoAmI(), HEX);
   
  setCtrlReg1();

   delay(1000);
}

void loop(void)
{

  Serial.print("Pressure = ");
  Serial.print(getPressure());
  Serial.print(" hPa ,Altitude = ");
  Serial.print(altitude());
  Serial.print(" m , Temperature = ");
  Serial.print(getTemperature());
  Serial.println(" degree");
 
  if(altitude() > gnd_T + 10){
    Serial.println("10m rise");
  }else if(altitude() < gnd_T + 10){
    if(altitude() > 1){
      delay(5000);
    }else{
      unsigned long start = millis(); //このときの時間をstartとする
       if(millis() > start + 20000){  //Startから20秒たったら導通
      Serial.println("HIGH");
      digitalWrite(12,HIGH); 
      delay(5000);

      Serial.println("LOW");
      digitalWrite(12,LOW);
    }else{
    }
    }
   
    while(1){
    }
  }
}

int whoAmI() {
  Wire.beginTransmission(LPS25H_ADDRESS);
  Wire.write(LPS25H_WHO_AM_I);
  Wire.endTransmission();
   
  Wire.requestFrom(LPS25H_ADDRESS, 1);
  while(Wire.available() < 1) {
    ;
  }
   
  return Wire.read();
}
 
void setCtrlReg1() {
  Wire.beginTransmission(LPS25H_ADDRESS);
  Wire.write(LPS25H_CTRL_REG1);
  Wire.write(0x90);
  Wire.endTransmission();
}

float getPressure() {
  long pData = 0;
   
  for (int i = 0; i < 3; i++) {
    Wire.beginTransmission(LPS25H_ADDRESS);
    Wire.write(LPS25H_PRESS_OUT_XL + i);
    Wire.endTransmission();
 
    Wire.requestFrom(LPS25H_ADDRESS, 1);
    while(Wire.available() < 1) {
      ;
    }
     
    pData |= Wire.read() << (8 * i);
  }
   
  return pData / 4096.0;
}
 
float getTemperature() {
  short tData = 0;
   
  for (int i = 0; i < 2; i++) {
    Wire.beginTransmission(LPS25H_ADDRESS);
    Wire.write(LPS25H_TEMP_OUT_L + i);
    Wire.endTransmission();
 
    Wire.requestFrom(LPS25H_ADDRESS, 1);
    while(Wire.available() < 1) {
      ;
    }
     
    tData |= Wire.read() << (8 * i);
  }
   
  return 42.5 + tData / 480.0;
}

volatile float altitude(){
  //[m]換算した高度
  //return altitude = ((pow(referencePressure / getPressure(), 1 / 5.257) - 1)*(getTemperature() + 273.15)) / 0.0065;
    return ((pow(referencePressure / getPressure(), 1 / 5.257) - 1)*(getTemperature() + 273.15)) / 0.0065; 
   // return altitude = pow((44.331514 - getPressure()) / 11.880516, 5.255877);
//下が気温を使わない高度の出し方です。

 // double n = pow(getPressure(),1/5.255877);
 // return 44.331514 - n*11.880516;

//if(float (altitude()) < 1){
//unsigned long start = millis();
//while (millis() < start + 5000) { 
  //Serial.println("HIGH");
 // digitalWrite(12,HIGH); 
  //delay(5000);

  //Serial.println("LOW");
  //digitalWrite(12,LOW);
//} 

//else{
//Serial.println("LOW");
  //digitalWrite(12,LOW);
 // delay(2500);
//}
}
