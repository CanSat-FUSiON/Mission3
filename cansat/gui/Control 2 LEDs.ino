#include <WiFi.h>

const char* ssid = "Buffalo-G-8B68";
const char* password = "byvf8jr5eierp";

WiFiServer server(80);

int ledPin1 = 13; // LED1のピン番号
int ledPin2 = 14; // LED2のピン番号

void setup() {
  Serial.begin(115200);
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);

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

    // LED1を点灯するボタンがクリックされた場合
    if (request.indexOf("/fusion/control/right") != -1) {
      digitalWrite(ledPin1, HIGH);
      delay(5000);
      digitalWrite(ledPin1, LOW);
    }
    
    // LED2を点灯するボタンがクリックされた場合
    if (request.indexOf("/fusion/control/left") != -1) {
      digitalWrite(ledPin2, HIGH);
      delay(5000);
      digitalWrite(ledPin2, LOW);
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
