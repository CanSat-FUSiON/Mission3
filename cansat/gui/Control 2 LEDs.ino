#include <WiFi.h>

const char* ssid = "Buffalo-G-8B68";
const char* password = "byvf8jr5eierp";

WiFiServer server(80);

int ledPin = 2; // LEDのピン番号
int duration = 0; // LEDが点灯する秒数

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  
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

    // URLからdurationを取得
    int pos = request.indexOf("duration=");
    if (pos != -1) {
      duration = request.substring(pos+9).toInt();
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
    client.println("<form method=\"get\">");
    client.println("<label>Duration (sec):</label><br/>");
    client.println("<input type=\"text\" name=\"duration\" value=\"\"><br/><br/>");
    client.println("<input type=\"submit\" value=\"Turn on LED\">");
    client.println("</form>");
    client.println("</body>");
    client.println("</html>");

    // LEDを点灯
    digitalWrite(ledPin, HIGH);
    delay(duration * 1000);
    // LEDを消灯
    digitalWrite(ledPin, LOW);
  }
}
