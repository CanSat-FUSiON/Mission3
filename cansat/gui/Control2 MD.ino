#include <WiFi.h>

const char* ssid = "Buffalo-G-8B68";
const char* password = "byvf8jr5eierp";

WiFiServer server(80);

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

#define LPS25H_ADDRESS  0x5c /* SA0 -> GND */
#define LPS25H_WHO_AM_I     0x0f
#define LPS25H_CTRL_REG1    0x20
#define LPS25H_PRESS_OUT_XL 0x28
#define LPS25H_PRESS_OUT_L  0x29
#define LPS25H_PRESS_OUT_H  0x2a
#define LPS25H_TEMP_OUT_L   0x2b
#define LPS25H_TEMP_OUT_H   0x2c

//int ledPin1 = 13; // LED1のピン番号
//int ledPin2 = 14; // LED2のピン番号

void setup() {
  Serial.begin(115200);
// PWM出力に使用する端子を出力設定
  pinMode(pwm1, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(dir2, OUTPUT);

  pinMode(12,OUTPUT);//baro

  // PWM初期設定
  ledcSetup(CH1, FREQ, BIT_NUM);   // PWM設定（ﾁｬﾝﾈﾙ, 周波数, bit数）
  ledcSetup(CH2, FREQ, BIT_NUM);   // PWM設定（ﾁｬﾝﾈﾙ, 周波数, bit数）
  ledcAttachPin(dir1, CH1);
  ledcAttachPin(dir2, CH2);

  digitalWrite(pwm1, HIGH);
  digitalWrite(pwm2, HIGH);
  /*
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);
  */
  
  // WiFiに接続
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // サーバーを開始
  server.begin();
  Serial.println("Server started");
}

void loop() {
  // クライアントからの接続を待機
  WiFiClient client = server.available();
  if (client) {
    Serial.println("New client");
    // HTTPリクエストを読み込み
    String request = client.readStringUntil('\r');
    Serial.println(request);
    client.flush();

    // rightボタンがクリックされた場合
    if (request.indexOf("/fusion/control/right") != -1) {
      /*
      digitalWrite(ledPin1, HIGH);
      delay(5000);
      digitalWrite(ledPin1, LOW);
      */
      ledcWrite(CH1, 0); //右タイヤ正回転
      ledcWrite(CH2, 2048); //左タイヤブレーキ
      delay(1000);
    }
    
    // leftボタンがクリックされた場合
    if (request.indexOf("/fusion/control/left") != -1) {
      /*
      digitalWrite(ledPin2, HIGH);
      delay(5000);
      digitalWrite(ledPin2, LOW);
      */
      ledcWrite(CH1, 2048); //右タイヤブレーキ
      ledcWrite(CH2, 0); //左タイヤ正回転
      delay(1000);
    }

    // forwardボタンがクリックされた場合
    if (request.indexOf("/fusion/control/forward") != -1) {
      ledcWrite(CH1, 0); 
      ledcWrite(CH2, 0); 
      delay(1000);
    }

    // backボタンがクリックされた場合
    if (request.indexOf("/fusion/control/back") != -1) {
      ledcWrite(CH1, 4096); 
      ledcWrite(CH2, 4096); 
      delay(1000);
    }

    // fireボタンがクリックされた場合
    if (request.indexOf("/fusion/control/fire") != -1) {
      Serial.println("HIGH");
      digitalWrite(12,HIGH); 
      delay(1000);

      Serial.println("LOW");
      digitalWrite(12,LOW);
    }

    // HTTPレスポンスを送信
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();
    client.println("<!DOCTYPE HTML>");
    client.println("<html>");
    client.println("<body>");
    client.println("<h1>LED Control</h1>");
    client.println("<form method=\"get\" action=\"/led1on\">");
    client.println("<input type=\"submit\" value=\"Turn on LED1 for 5 sec\"><br>");
    client.println("</form>");
    client.println("<form method=\"get\" action=\"/led2on\">");
    client.println("<input type=\"submit\" value=\"Turn on LED2 for 5 sec\"><br>");
    client.println("</form>");
    client.println("</body>");
    client.println("</html>");
  }
}
