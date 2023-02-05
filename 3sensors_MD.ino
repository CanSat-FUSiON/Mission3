
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>

//MD-----------------------------------------------

// PWM出力端子設定
#define pwm_2 26     
#define pwm_1 27
#define dir_2 25
#define dir_1 14
 

// PWM出力設定（周波数と分解能はチャンネルのペアでは同じに設定する）
#define CH 1        // PWM出力チャンネル（0,1/ 2,3/ 4,5/ 6,7/ 8,9/ 10,11 /12,13 /14,15でペア）
#define FREQ 4.9   // PWM出力周波数（最大周波数 : 20kHz / 2の「bit数」乗）
#define BIT_NUM 12  // bit数（1bit〜16bit）

// 変数設定
float ad_val;       // アナログ入力値格納用
float duty;         // Duty比
float resolution;   // 分解能
float battery;      // バッテリー残量表示用
bool state = false; // ボタン操作状態保持用

//3sensor-----------------------------------------------------

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
 
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

TinyGPSPlus gps;

float gps_lat; //緯度
float gps_longt; //経度 

int RX_PIN = 19;
int TX_PIN = 18;

void setup(void)
{
  Serial.begin(115200);

//MD---------------------------------------------
// PWM出力に使用する端子を出力設定
  pinMode(pwm_2, OUTPUT);     
  pinMode(pwm_1, OUTPUT);     
  pinMode(dir_1, OUTPUT);
  pinMode(dir_2, OUTPUT);

  // PWM初期設定
  ledcSetup(CH, FREQ, BIT_NUM);   // PWM設定（ﾁｬﾝﾈﾙ, 周波数, bit数）
  ledcAttachPin(dir_2, CH); // PWMﾁｬﾝﾈﾙ割り当て（端子番号, ﾁｬﾝﾈﾙ）
  ledcAttachPin(dir_1, CH); 

  int reading = 0;
  int output=0;
  int opposite_output=0;
  digitalWrite(pwm_1,HIGH);
  digitalWrite(pwm_2,HIGH);

  // PWM出力実行（以下の3ついずれか1つを有効にして動作確認）
  ledcWrite(1, 4096);       // チャンネルに指定した周波数、分解能で実行する場合（チャンネル、Duty比）
  delay(10000);
  ledcWrite(1, 1);
  delay(10000);
  digitalWrite(pwm_1,LOW);
  digitalWrite(pwm_2,LOW);

  //3sensor----------------------------------------

  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  
  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

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
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  printEvent(&orientationData);
  printEvent(&angVelocityData);
  printEvent(&linearAccelData);
  printEvent(&magnetometerData);
  printEvent(&accelerometerData);
  printEvent(&gravityData);

  int8_t boardTemp = bno.getTemp();
  Serial.println();
  Serial.print(F("temperature: "));
  Serial.println(boardTemp);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.println();
  Serial.print("Calibration: Sys=");
  Serial.print(system);
  Serial.print(" Gyro=");
  Serial.print(gyro);
  Serial.print(" Accel=");
  Serial.print(accel);
  Serial.print(" Mag=");
  Serial.println(mag);

  Serial.println("--");
  delay(BNO055_SAMPLERATE_DELAY_MS);

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
  

  while (Serial1.available() > 0) {
    char c = Serial1.read();
    gps.encode(c);
    if (gps.location.isUpdated()) {

      gps_lat = gps.location.lat();
      gps_longt = gps.location.lng();
      Serial.print("LAT:  "); Serial.println(gps_lat,9);
      Serial.print("LONG: "); Serial.println(gps_longt,9);
     
    }
  }
  delay(2000);
}

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
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
}



