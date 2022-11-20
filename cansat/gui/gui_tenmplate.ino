/*********
  Rui Santos
  Complete project details at http://randomnerdtutorials.com  
*********/

// Wi-Fiライブラリをインポート
#include <WiFi.h>

// Wi-Fi接続情報を入力
const char* ssid     = "shiota kyohei";
const char* password = "1234567890";

// ウェブサーバーをポート80で開始
WiFiServer server(80);

// HTTPリクエストを保存しておく変数
String header;

// ピンの状態を保存する変数の宣言
String output26State = "off";
String output27State = "off";

// 使用するピンに名前を付ける
const int output26 = 26;
const int output27 = 27;

void setup() {
  Serial.begin(115200);
  // ピンを二つとも出力ピンに割り当て
  pinMode(output26, OUTPUT);
  pinMode(output27, OUTPUT);
  // ２つのピンをオフ（LEDをオフ）
  digitalWrite(output26, LOW);
  digitalWrite(output27, LOW);

  // ssidとpasswordを用いてWi-Fiに接続
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // IPアドレスを出力し、webserverをスタート
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop(){
  WiFiClient client = server.available();   //クライアント（スマホやPCなど）がつながっているかどうかをclientに出力

  if (client) {                             // クライアントが来たとき
    Serial.println("New Client.");          
    String currentLine = "";                // クライアントからくるデータを格納する変数
    while (client.connected()) {            // クライアントがつながっている間、以下をループ
      if (client.available()) {             // クライアントからデータが来ているとき
        char c = client.read();             // データを読み込み
        Serial.write(c);                    // 届いたデータをシリアルモニタに出力
        header += c;
        if (c == '\n') {                    // 届いたデータが改行コードだった時
          // もし現在の行が空白ならば、この改行コードのみ受け取る
          // つまりHTTPリクエストの終わりなので、レスポンスを返す
          if (currentLine.length() == 0) {
            // HTTPヘッダは（HTTP/1.1 200 OK)のようなステータスコードから始まる
            // 次にコンテントタイプを送信。今回はhtml形式なので以下のようにする
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // リクエストに従ってGPIOをスイッチする
            if (header.indexOf("GET /26/on") >= 0) {
              Serial.println("GPIO 26 on");
              output26State = "on";
              digitalWrite(output26, HIGH);
            } else if (header.indexOf("GET /26/off") >= 0) {
              Serial.println("GPIO 26 off");
              output26State = "off";
              digitalWrite(output26, LOW);
            } else if (header.indexOf("GET /27/on") >= 0) {
              Serial.println("GPIO 27 on");
              output27State = "on";
              digitalWrite(output27, HIGH);
            } else if (header.indexOf("GET /27/off") >= 0) {
              Serial.println("GPIO 27 off");
              output27State = "off";
              digitalWrite(output27, LOW);
            }
            
            // htmlを表示
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // ON/OFFボタンのためのCSS 
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");
            
            // ページ本体（bodyタグ内）
            client.println("<body><h1>ESP32 Web Server</h1>");
            
            // 現在のピンの状態と、オンオフ用のボタンを出力
            client.println("<p>GPIO 26 - State " + output26State + "</p>");
            // output26Stateがオフなら、ONにするボタンを表示
            if (output26State=="off") {
              client.println("<p><a href=\"/26/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/26/off\"><button class=\"button button2\">OFF</button></a></p>");
            } 
               
            // 27ピンの状態と、オンオフボタン
            client.println("<p>GPIO 27 - State " + output27State + "</p>");
            // If the output27State is off, it displays the ON button       
            if (output27State=="off") {
              client.println("<p><a href=\"/27/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/27/off\"><button class=\"button button2\">OFF</button></a></p>");
            }
            client.println("</body></html>");
            
            //　HTTPレスポンスの最後は改行で終了
            client.println();
            // whileループの終了
            break;
          } else { // 改行コードを取得したら、currentLineをリセット
            currentLine = "";
          }
        } else if (c != '\r') {  // 改行以外の何かしらのコードが来ているとき
          currentLine += c;      // currentLineに追加
        }
      }
    }
    // ヘッダーをリセット
    header = "";
    // 接続をリセット
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}
