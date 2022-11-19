// Wi-Fiライブラリをインポート
#include <WiFi.h>

// Wi-Fi接続情報を入力(**のところは自分で入力)
const char* ssid     = "********";
const char* password = "******";

// ウェブサーバーをポート80で開始
WiFiServer server(80);

// HTTPリクエストを保存しておく変数
String header;

// ピンの状態を保存する変数の宣言
String output26State = "off";
String output27State = "off";
String output25State = "off";
String output33State = "off";

// 使用するピンに名前を付ける
const int output26 = 26;
const int output27 = 27;
const int output25 = 25;
const int output33 = 33;



void setup() {
  Serial.begin(115200);
  // ピンを四つとも出力ピンに割り当て
  pinMode(output26, OUTPUT);
  pinMode(output27, OUTPUT);
  pinMode(output25, OUTPUT);
  pinMode(output33, OUTPUT);
  // ２つのピンをオフ（LEDをオフ）
  digitalWrite(output26, LOW);
  digitalWrite(output27, LOW);
  digitalWrite(output25, LOW);
  digitalWrite(output33, LOW);

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
            //前進
            if (header.indexOf("GET /alltyre/forward") >= 0) {
              Serial.println("forward");
              output26State = "on";
              output27State = "on";
              output25State = "on";
              output33State = "on";
              digitalWrite(output26, HIGH);
              digitalWrite(output27, HIGH);
              digitalWrite(output25, HIGH);
              digitalWrite(output33, HIGH);
            } 
            //後進
              else if (header.indexOf("GET /alltyre/back") >= 0) {
              Serial.println("back");
              output26State = "on";
              output27State = "off";
              output25State = "on";
              output33State = "off";
              digitalWrite(output26, HIGH);
              digitalWrite(output27, LOW);
              digitalWrite(output25, HIGH);
              digitalWrite(output33, LOW);
            } 
            //ストップ  
              else if (header.indexOf("GET /alltyre/stop") >= 0) {
              Serial.println("stop");
              output26State = "off";
              output27State = "off";
              output25State = "off";
              output33State = "off";
              digitalWrite(output26, LOW);
              digitalWrite(output27, LOW);
              digitalWrite(output25, LOW);
              digitalWrite(output33, LOW);
            } 
            
            //左旋回
              else if (header.indexOf("GET /righttyre/forward") >= 0) {
              Serial.println("leftturn");
              output26State = "on";
              output27State = "on";
              output25State = "off";
              output33State = "off";
              digitalWrite(output26, HIGH);
              digitalWrite(output27, HIGH);
              digitalWrite(output25, LOW);
              digitalWrite(output33, LOW);
            } 
            
            //右旋回
              else if (header.indexOf("GET /lefttyre/forward") >= 0) {
              Serial.println("rightturn");
              output26State = "off";
              output27State = "off";
              output25State = "on";
              output33State = "on";
              digitalWrite(output26, LOW);
              digitalWrite(output27, LOW);
              digitalWrite(output25, HIGH);
              digitalWrite(output33, HIGH);
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
            


            // 右タイヤについて現在のピンの状態と、オンオフ用のボタンを出力

            // 前進(右タイヤ・左タイヤ前進)
            client.println("<p> forward </p>");
            // 全てがオフなら、前進・後進・右旋回・左旋回を表示
            if (output26State == "off" and output27State == "off" and output25State == "off" and output33State == "off") {
              client.println("<p><a href=\"/alltyre/forward\"><button class=\"button\">Forward(Stop now)</button></a></p>");
              client.println("<p><a href=\"/alltyre/back\"><button class=\"button\">Back(Stop now)</button></a></p>");
              client.println("<p><a href=\"/righttyre/forward\"><button class=\"button\">Left turn(Stop now)</button></a></p>");
              client.println("<p><a href=\"/lefttyre/forward\"><button class=\"button\">Right turn(Stop now)</button></a></p>");
            } else {
              client.println("<p><a href=\"/alltyre/stop\"><button class=\"button button2\">Stop</button></a></p>");
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
