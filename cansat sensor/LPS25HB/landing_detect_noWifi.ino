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
volatile float referencePressure = 1013.15;//高度計算に使用する基準高度での大気圧[hPa]。ここは随時変える。

// PWM出力端子設定
#define pwm1 14
#define dir1 27
#define pwm2 26
#define dir2 25

// PWM出力設定（周波数と分解能はチャンネルのペアでは同じに設定する）
#define CH1 1        // PWM出力チャンネル（0,1/ 2,3/ 4,5/ 6,7/ 8,9/ 10,11 /12,13 /14,15でペア）
#define FREQ 20000   // PWM出力周波数（最大周波数 : 20kHz / 2の「bit数」乗）
#define BIT_NUM 12  // bit数（1bit〜16bit）

#define CH2 0        // PWM出力チャンネル（0,1/ 2,3/ 4,5/ 6,7/ 8,9/ 10,11 /12,13 /14,15でペア）
#define FREQ 20000   // PWM出力周波数（最大周波数 : 20kHz / 2の「bit数」乗）
#define BIT_NUM 12  // bit数（1bit〜16bit）

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
   
   // PWM出力に使用する端子を出力設定
  pinMode(pwm1, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(dir2, OUTPUT);

  // PWM初期設定
  ledcSetup(CH1, FREQ, BIT_NUM);   // PWM設定（ﾁｬﾝﾈﾙ, 周波数, bit数）
  ledcAttachPin(dir1, CH1);
  ledcSetup(CH2, FREQ, BIT_NUM);   // PWM設定（ﾁｬﾝﾈﾙ, 周波数, bit数）
  ledcAttachPin(dir2, CH2);

  digitalWrite(pwm1, HIGH);
  digitalWrite(pwm2, HIGH);
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
  
if(float(altitude()) > float(gnd_T) + 10){
  Serial.println("over 10meters");
  delay(1000);//10mのとこから1mのところまでおちてくる時間。20秒待機した後につぎのifの判断に移るってなるはず。

  while(1){


// * while(1)でくくってるところで無限ループするので気圧とかの値が出力されなくなってしまうのでは？と思ったので実験してそうなった場合は、ここのコメントアウトはずしてください。
  Serial.print("Pressure = ");
  Serial.print(getPressure());
  Serial.print(" hPa ,Altitude = ");
  Serial.print(altitude());
  Serial.print(" m , Temperature = ");
  Serial.print(getTemperature());
  Serial.println(" degree");
//*/
    
    if(float(altitude()) < 1){
      Serial.println("under 1meter");
      delay(20000);//20秒まって導通
      Serial.println("HIGH");
      digitalWrite(12,HIGH); 
      delay(10000);

      Serial.println("LOW");
      digitalWrite(12,LOW);
      Serial.print("1");
  ledcWrite(CH1, 1000); //フル正回転
  ledcWrite(CH2, 1000); //フル正回
  delay(5000);
  Serial.print("2");
  ledcWrite(CH1, 2846); //フル逆回転4096
  ledcWrite(CH2, 2846); //フル逆回転
  delay(5000);
  Serial.print("3");
  ledcWrite(CH1, 1000);
  ledcWrite(CH2, 2048);
  delay(5000);
  Serial.print("4");
  ledcWrite(CH1, 2048);
  ledcWrite(CH2, 1000);
  delay(5000);
      while(1){
    }
   }else{
    Serial.println("over 1meter");
    delay(500);
}
}

}else{
  Serial.println("under 10meters");
  delay(500);
}
  
 
  /*if(float(altitude()) > float(gnd_T) + 10){
    Serial.println("10m rise");
  }else if(float(altitude()) < float(gnd_T) + 10){
    if(float(altitude()) > 1){
      delay(100);
    }else{
      unsigned long start = millis(); //このときの時間をstartとする
       if(start > 20000){  //Startから20秒たったら導通
      Serial.println("HIGH");
      digitalWrite(12,HIGH); 
      delay(10000);
      Serial.println("LOW");
      digitalWrite(12,LOW);
      while(1){
    }
    }
    //else{
    //}
    }
  }*/
    //while(1){
    //}
  
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
