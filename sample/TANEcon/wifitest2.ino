#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>

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


}

void loop() {
  delay(1000);//allow the cpu to switch to other tasks

}
