#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU9250_asukiaaa.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>

//マックスの家庭
//const char* ssid = "C42B4484CD6E-2G";
//const char* password = "xgkfxpet80ww44";
//
//const IPAddress ip(192,168,3,15);
//const IPAddress subnet(255,255,255,0);

//あべ家庭
const char* ssid = "30F772BDEE2B-2G";
const char* password = "2215092933052";

const IPAddress ip(192, 168, 3, 7);
const IPAddress gateway(192,168, 3, 1);
const IPAddress subnet(255, 255, 255, 0);

//あべスマホ
//const char* ssid = "mayaphone";
//const char* password = "astroswitch";
//
//const IPAddress ip(192,168,3,15);
//const IPAddress subnet(255,255,255,0);

WebServer server(80);

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
int a = 0;
int lonxx;
int latxx;
int mode=0;
int num=0;
int num2 = 0;
int num3 = 0;
float timer;
int stanum = 0;


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

void run_cmd(int num4, int msec){  //num4にコマンドの種類、msecに動作時間(ms)
  switch(num4){
    case 0: //ニクロム線点火
     digitalWrite(14,HIGH);
     delay(msec);
     digitalWrite(14,LOW);
     
    case 1: //前進
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

    case 2: //後退
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

    case 3: //右回転
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

    case 4: //左回転
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

void handleright() {
  if (server.hasArg("ms")) {
    int j = server.arg("ms").toInt();
    run_cmd(3,j);
  }else{
    run_cmd(3,500);  
  }
  server.send(200, "text/plain", "go right");
  Serial.println("right working");
  a = 1;
}

void handlerightlittle() {
  run_cmd(3,200);
  delay(5000);
  server.send(200, "text/plain", "go right little");
  Serial.println("right little working");
  a = 1;
}


void handleleft() {
  if (server.hasArg("ms")) {
    int j = server.arg("ms").toInt();
    run_cmd(4,j);
  }else{
    run_cmd(4,500);
  }
  server.send(200, "text/plain", "go left");
  Serial.println("left working");
  a = 1;
}


void handleforward() {
  if (server.hasArg("ms")) {
    int j = server.arg("ms").toInt();
    run_cmd(1,j);
  }else{
    run_cmd(1,500);
  }
  server.send(200, "text/plain", "go forward");
  Serial.println("forward working");
  a = 1;
}


void handleback() {
  if (server.hasArg("ms")) {
    int j = server.arg("ms").toInt();
    run_cmd(2,j);
  }else{
    run_cmd(2,500);
  }
  server.send(200, "text/plain", "go back");
  Serial.println("back working");
  a = 1;
}


void handleautomatic() {
  
  float angx;
  float angy;
  float anga;
  float angb;
  
  if (server.hasArg("a")) {
    int i = server.arg("a").toInt();
    angx = mpu_ang();
    angy = angx + i;
    if(angy<0){
      angy = angy + 360;
    }
    
    anga = angy - 5;
    angb = angy + 5;
    
    while(angx<anga||angb<angx){
      motor2(2,0,0);
      angx = mpu_ang();
    }
    motor(0,400);
    delay(5000);
  }
  server.send(200, "text/plain", "image processing");
  a = 1;
}


void automastart(){
  stanum = 1;
}


void automaend(){
  a = 1;
}


void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t j = 0; j < server.args(); j++) {
    message += " " + server.argName(j) + ": " + server.arg(j) + "\n";
  }
  server.send(404, "text/plain", message);
  a = 1;
}


void setup() {
  mySerial.begin(9600);
  Serial.begin(115200);
  while(!Serial);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
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


  server.on("/right", handleright);
  server.on("/left", handleleft);
  server.on("/forward", handleforward);
  server.on("/back", handleback);
 

  server.on("/right_little", handlerightlittle);
  
  server.on("/image_automatic", handleautomatic);

  server.on("/start", automastart);
  server.on("/end", automaend);

  server.onNotFound(handleNotFound);

  
  server.begin();

}



void loop() {
  switch(num){
    case 0:
        server.handleClient();
        delay(1000);

        if(stanum == 1){
          num = 1;
          break;
        }

        if(a == 1){
          num = 3;
          break;
        }
        
        break;
      
      
    case 1:
        
        digitalWrite(14,HIGH);
        delay(3000);
        digitalWrite(14,LOW);
        num = 2;
        break;
        
      

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

          if (angm<angb){
            while(angm<angb){
              motor2(2,0,0);
              angm = mpu_ang();
            }
          }
          if else (anga<angm){
            while(anga<angm){
              motor2(3,0,0);
              angm = mpu_ang();
            }
          }

          
          motor(0,5000);
          server.handleClient();
          if(a = 1){
            num = 3;
            break;
          }
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

        num = 3;

    case 3:
      server.handleClient();
      delay(1000);
       }
  }
