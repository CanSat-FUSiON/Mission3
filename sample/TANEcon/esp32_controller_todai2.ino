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
const char* ssid = "C42B4484CD6E-2G";
const char* password = "xgkfxpet80ww44";

const IPAddress ip(192,168,3,15);
const IPAddress subnet(255,255,255,0);

//あべ家庭
//const char* ssid = "30F772BDEE2B-2G";
//const char* password = "2215092933052";
//
//const IPAddress ip(192, 168, 3, 7);
//const IPAddress gateway(192,168, 3, 1);
//const IPAddress subnet(255, 255, 255, 0);

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


float mpu_ang();
float gps_ang();
float gps_dis();
double gps_read();
float read_heig(float, float);
void motor(int, int);
void motor2(int, int, int);
void fire(int); //ニクロム線発熱関数の宣言


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


  
void fire(int msec){
  digitalWrite(14, HIGH);
  delay(msec);
  digitalWrite(14, LOW);
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
  if (num==1){
    if (server.hasArg("ms")) {
      int j = server.arg("ms").toInt();
      run_cmd(3,j);
    }else{
      run_cmd(3,500);  
    }
    server.send(200, "text/plain", "go right");
    Serial.println("right working");
  }
}

void handlerightlittle() {
  if (num==1){
    run_cmd(3,200);
    delay(5000);
    server.send(200, "text/plain", "go right little");
    Serial.println("right little working");
  }
}


void handleleft() {
  if (num==1){
    if (server.hasArg("ms")) {
      int j = server.arg("ms").toInt();
      run_cmd(4,j);
    }else{
      run_cmd(4,500);
    }
    server.send(200, "text/plain", "go left");
    Serial.println("left working");
  }
}


void handleforward() {
  if (num==1){
    if (server.hasArg("ms")) {
      int j = server.arg("ms").toInt();
      run_cmd(1,j);
    }else{
      run_cmd(1,500);
    }
    server.send(200, "text/plain", "go forward");
    Serial.println("forward working");
  }
}


void handleback() {
  if (num==1){
    if (server.hasArg("ms")) {
      int j = server.arg("ms").toInt();
      run_cmd(2,j);
    }else{
      run_cmd(2,500);
    }
    server.send(200, "text/plain", "go back");
    Serial.println("back working");
  }
}


void handleautomatic() {
  if (num==1){
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
  }
}


void automastart(){
  if (num==0){
    fire(1500);
    num = 1;
  }
}


void automaend(){
  if (num==1){
    num = 0;
  }
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
  if (!WiFi.config(ip,ip,subnet)){
     Serial.println("Failed to configure!");
  }

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin("esp32")) {
    Serial.println("MDNS responder started");
  }

  pinMode(32,OUTPUT);
  
  pinMode(33,OUTPUT);
  ledcSetup(1, 1000, 8);
  ledcAttachPin(33, 1);
  
  pinMode(26,OUTPUT);
  
  pinMode(27,OUTPUT);
  ledcSetup(3, 1000, 8);
  ledcAttachPin(27, 3);
 
  pinMode(14,OUTPUT);


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
  server.handleClient();
  delay(100);
  }
