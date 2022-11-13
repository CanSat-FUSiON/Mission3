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

int num=0;

  
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
  if (num == 1){
    if (server.hasArg("a")) {
      int i = server.arg("a").toInt();
      Serial.println(i);
      if ( i < 0 ) {
        run_cmd(3, -i*20);
      }
      else if ( i > 0 ) {
        run_cmd(2, i*20);
      }
      else {
      }
    run_cmd(0,600);
    }
    server.send(200, "text/plain", "automatic control");
    delay(2000);
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
