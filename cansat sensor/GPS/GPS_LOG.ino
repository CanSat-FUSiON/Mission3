#include <TinyGPS++.h>
#include "Ambient.h"

TinyGPSPlus gps;

float gps_lat; //緯度
float gps_longt; //経度 

int RX_PIN = 16;
int TX_PIN = 17;

WiFiClient client;
Ambient ambient;

const char* ssid = "Buffalo-G-8B68";
const char* password = "byvf8jr5eierp";

unsigned int channelId = 62369;
const char *writeKey = "a1ae206c8214f215";

void setup()
{
  // シリアルポート開始
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);


  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());


  ambient.begin(channelId, writeKey, &client);

}

void loop() {
  // Serial.println("test"); 
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    gps.encode(c);
    if (gps.location.isUpdated()) {

      gps_lat = gps.location.lat();
      gps_longt = gps.location.lng();
      Serial.print("LAT:  "); Serial.println(gps_lat,9);
      Serial.print("LONG: "); Serial.println(gps_longt,9);
      delay(1000);

      ambient.set(9, gps_lat);
      ambient.set(10, gps_longt);
      ambient.send();   
    }
  }
  delay(1000);
}
