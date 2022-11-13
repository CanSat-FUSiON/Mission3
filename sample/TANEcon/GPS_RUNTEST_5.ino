#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU9250_asukiaaa.h>
#include <math.h>
#include <SoftwareSerial.h>


Adafruit_BME280 bme;
SoftwareSerial mySerial(16, 17);
MPU9250_asukiaaa mySensor;

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif

//#define PIN_IN1 32
//#define PIN_IN2 33
//#define PIN_IN3 26
//#define PIN_IN4 27

float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ, ang;
float mXoffset = 34.44;
float mYoffset = 102.13;
float mZoffset = 0;
float lonx;
float latx;
float lony =139.736954;
float laty =35.741970;
float _lonx;
float _latx;
float _lony;
float _laty;
double aa;
double ab;
float ac;
float ad;
float XY;
float pi = 3.14159265;
float r = 6378137;//地球半径[m]表示
float dis;
float disa;
float disb;
float disc;
float disd;
float disx;
float anga;
float angb;
float angm;
float angx;
float pres = 0;
float temp = 0;
float heig = 0;
float heigx = 0;
double a1;
double a2;
int lonxx;
int latxx;
int mode=0;
int num=0;
int num2 = 0;
int num3 = 0;
float timer;


float mpu_ang();
float gps_ang();
float gps_dis();
double gps_read();
float read_heig(float, float);
void motor(int, int);
void motor2(int, int, int);
void fire(int); //ニクロム線発熱関数の宣言

//#define MOSFET_G 14
double gps_read(){
      String line = mySerial.readStringUntil('\n');
 
  if(line != ""){
    int i, index = 0, len = line.length();
    String str = "";
  
    // StringListの生成(簡易)
    String list[30];
    for (i = 0; i < 30; i++) {
      list[i] = "";
    }
 
    // 「,」を区切り文字として文字列を配列にする
    for (i = 0; i < len; i++) {
      if (line[i] == ',') {
        list[index++] = str;
        str = "";
        continue;
      }
      str += line[i];
    }
    
    // $GPGGAセンテンスのみ読み込む
    if (list[0] == "$GPGGA") {
      
      // ステータス
      if(list[6] != "0"){      
            float af = list[2].toFloat();
            int ad = af / 100;
            int am = (((af / 100.0) - ad) * 100.0) / 60;
            float as = (((((af / 100.0) - ad) * 100.0) - am) * 60) / (60 * 60);
            float latx = ad + am + as;
            
 
        // 経度
        float f = list[4].toFloat();
        int d = f / 100;
        int m = (((f / 100.0) - d) * 100.0) / 60;
        float s = (((((f / 100.0) - d) * 100.0) - m) * 60) / (60 * 60);
        float lonx = d + m + s;

        int lonxx = lonx*1000000;
        int latxx = latx*1000000;

        aa = latxx;
        aa = aa/100000000;

        aa = lonxx + aa;
      
        
        return aa;
       }
}
}
}

float read_heig(float temp,float pres){
  heig = ((pow(1008.68/pres,1/5.257)-1)*(temp+273.15))/0.0065; 
  return heig;
  }
  
void fire(int msec){
  digitalWrite(14, HIGH);
  delay(msec);
  
  digitalWrite(14, LOW);
}

float mpu_ang(){
    uint8_t sensorId;

  
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
  
    return ang;

  }
}

float gps_ang(float lonx, float latx){        
       ab = atan2((lony-lonx)*1.23,(laty-latx))*57.3+180;
       return ab;    
       }

float gps_dis(float lonx, float latx){         
  aa = sqrt(pow(lony-lonx,2)+pow(laty-latx,2))*99096.44;
  return aa; 
}

void motor(int mode, int msec){
  switch(mode){

    case 0: //直進
     digitalWrite(32,HIGH);
     ledcWrite(1,0);
     digitalWrite(26,HIGH);
     ledcWrite(3,0);

     delay(msec);

     digitalWrite(32,LOW);
     ledcWrite(1,0);
     digitalWrite(26,LOW);
     ledcWrite(3,0);


      break;

    case 1: //後退
     digitalWrite(32,LOW);
     ledcWrite(1,255);
     digitalWrite(26,LOW);
     ledcWrite(3,255);

     delay(msec);

     digitalWrite(32,LOW);
     ledcWrite(1,0);
     digitalWrite(26,LOW);
     ledcWrite(3,0);

 
     
      break;

    case 2: //右回転
     digitalWrite(32,HIGH);
     ledcWrite(1,0);
     digitalWrite(26,LOW);
     ledcWrite(3,0);

     delay(msec);

     digitalWrite(32,LOW);
     ledcWrite(1,0);
     digitalWrite(26,LOW);
     ledcWrite(3,0);


      break;

    case 3: //左回転
     digitalWrite(32,LOW);
     ledcWrite(1,0);
     digitalWrite(26,HIGH);
     ledcWrite(3,0);

     delay(msec);

     digitalWrite(32,LOW);
     ledcWrite(1,0);
     digitalWrite(26,LOW);
     ledcWrite(3,0);

      

      break;
      
  }
}



void motor2(int mode, int left, int right){
  switch(mode){

    case 0: //直進
     digitalWrite(32,HIGH);
     ledcWrite(1,0);
     digitalWrite(26,HIGH);
     ledcWrite(3,0);
      break;

    case 1: //後退
     digitalWrite(32,LOW);
     ledcWrite(1,left);
     digitalWrite(26,LOW);
     ledcWrite(3,right);
      break;

    case 2: //右回転
     digitalWrite(32,HIGH);
     ledcWrite(1,0);
     digitalWrite(26,LOW);
     ledcWrite(3,0);
      break;

    case 3: //左回転
     digitalWrite(32,LOW);
     ledcWrite(1,0);
     digitalWrite(26,HIGH);
     ledcWrite(3,0);
      break;

    case 4: //右回転強め
     digitalWrite(32,HIGH);
     ledcWrite(1,0);
     digitalWrite(26,LOW);
     ledcWrite(3,255);
      break;
      
  }
}


void setup() {
  mySerial.begin(9600);
  Serial.begin(115200);
  while(!Serial);
  
  bool status;
  status = bme.begin(0x76); 
  
#ifdef _ESP32_HAL_I2C_H_ // For ESP32
  Wire.begin(SDA_PIN, SCL_PIN);
  mySensor.setWire(&Wire);
#endif

  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  pinMode(32,OUTPUT);
  
  pinMode(33,OUTPUT);
  ledcSetup(1, 1000, 8);
  ledcAttachPin(33, 1);
  
  pinMode(26,OUTPUT);
  
  pinMode(27,OUTPUT);
  ledcSetup(3, 1000, 8);
  ledcAttachPin(27, 3);
 
  pinMode(14, OUTPUT);

}



void loop() {
  switch(num){
    case 0:
      temp=bme.readTemperature();
      pres=bme.readPressure() / 100.0F;
      heig = read_heig(temp,pres);
      

      if(heig <5){
        num = 1;
       break;
      }
     
      
      
    case 1:
        
        temp=bme.readTemperature();
        pres=bme.readPressure() / 100.0F;
        heig = read_heig(temp,pres);

        if(heigx < 3){
          digitalWrite(14,HIGH);
          delay(3000);
          digitalWrite(14,LOW);
          num = 2;
          break;
        }
        
        delay(2000);
        
        temp=bme.readTemperature();
        pres=bme.readPressure() / 100.0F;
        heigx = heig - read_heig(temp,pres);
        
      

    case 2:
    
     motor(0,5000);

    a1 = 0;
 while(a1 <1){
   a1 = gps_read();
   if(a1>1){
    a2 = a1;
    break ; 
   }
 }
  lonxx = a2;
  lonx = a2/1000000;
  latx = (a2 -lonxx)*100;
  dis = gps_dis(lonx,latx);
  
   while(dis>7){
          a1 = 0;
       while(a1 <1){
         a1 = gps_read();
         if(a1>1){
          a2 = a1;
          
          break ; 
         }
       }
       
        lonxx = a2;
        lonx = a2/1000000;
        latx = (a2 -lonxx)*100;
          angx = gps_ang(lonx, latx);
          anga = angx+15;
          angb = angx-15;
          angm = mpu_ang();
          timer = 0;
          while(angm<angb||anga<angm){
            motor2(2,0,0);
            angm = mpu_ang();
            delay(100);
            timer = timer+0.1;
            if (timer>8){
              num = 3;
              break;
            }
          }
          
          motor(0,5000);
          delay(500);
        
     a1 = 0;
   while(a1 <1){
     a1 = gps_read();
     if(a1>1){
      a2 = a1;
      break ; 
     }
 }
  lonxx = a2;
  lonx = a2/1000000;
  latx = (a2 -lonxx)*100;
  dis = gps_dis(lonx,latx);

   switch(num3){
    case 0:
     disa = dis;
     disx = abs(disa - disb);
     if(disx<20){
      num = 3;
      break;
     }
     num3 = 1;
     
    case 1:
     disb = dis;
     disx = abs(disb - disc);
     if(disx<20){
      num = 3;
      break;
     }
     num3 = 2;
     
    case 2:
     disc = dis;
     disx = abs(disc - disd);
     if(disx<20){
      num = 3;
      break;
     }
     num3 = 3;
     
    case 3:
     disd = dis;
     disx = abs(disd - disa);
     if(disx<20){
      num = 3;
      break;
     }
     num3 = 0;
     
   }
        }
      
      
         angx = gps_ang(lonx, latx);
         anga = angx+15;
         angb = angx-15;
         angm = mpu_ang();
        
          while(angm<angb||anga<angm){
           motor2(2,0,0);
           angm = mpu_ang();
          }
          
        while(20<angm){
          motor2(2,0,0);
          angm = mpu_ang();
        }
        break  ;


    case 3:
       switch(num2){
        case 0:
          motor(1,5000); 
          motor(2,1000);
          num = 2;
          num2 = 1;
          num3 = 0;
          disa = 0;
          disb = 0;
          disc = 0;
          disd = 0;
          break;

        case 1:
          motor2(1,0,255);
          delay(2000);
          motor2(1,255,255);
          delay(2000);
          motor2(1,0,0);
          motor(2,1000);
          num = 2;
          num2 = 2;
          num3 = 0;
          disa = 0;
          disb = 0;
          disc = 0;
          disd = 0;
          break;

        case 2:
          motor2(1,255,0);
          delay(2000);
          motor2(1,255,255);
          delay(2000);
          motor2(1,0,0);
          motor(2,1000);
          num = 2;
          num2 = 3;
          num3 = 0;
          disa = 0;
          disb = 0;
          disc = 0;
          disd = 0;
          break;

         case 3:
          motor2(1,255,0);
          delay(2000);
          motor2(1,0,255);
          delay(2000);
          motor2(1,255,255);
          delay(2000);
          motor2(1,0,0);
          motor(2,1000);
          num = 2;
          num2 = 4;
          num3 = 0;
          disa = 0;
          disb = 0;
          disc = 0;
          disd = 0;
          break;

         case 4:
          motor2(1,255,150);
          delay(2000);
          motor2(1,0,0);
          motor(2,1000);
          num = 2;
          num2 = 5;
          num3 = 0;
          disa = 0;
          disb = 0;
          disc = 0;
          disd = 0;
          break;

         case 5:
          motor2(1,150,255);
          delay(2000);
          motor2(1,0,0);
          motor(2,1000);
          num = 2;
          num2 = 6;
          num3 = 0;
          disa = 0;
          disb = 0;
          disc = 0;
          disd = 0;
          break;

         case 6:
          motor2(4,0,0);
          delay(2000);
          motor2(1,0,0);
          motor(1,3000);
          motor(2,1000);
          num = 2;
          num2 = 7;
          num3 = 0;
          disa = 0;
          disb = 0;
          disc = 0;
          disd = 0;
          break;

         case 7:
         for (int k=0; k<=5; k++){
          motor(0,3000);
          motor(1,5000);
         }
         motor(2,1000);
         num = 2;
         num2 = 7;
         num3 = 0;
         disa = 0;
         disb = 0;
         disc = 0;
         disd = 0;
          
       }
  
  }
  
}
