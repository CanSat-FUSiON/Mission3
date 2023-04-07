

#include <TinyGPS++.h>

// GPSモジュールのRXとTXピン
static const int RXPin = 4, TXPin = 3;
// GPSモジュールのボーレート
static const uint32_t GPSBaud = 9600;

float gps_lat; //現在地の緯度
float gps_longt; //現在地の経度


// TinyGPS++オブジェクト
TinyGPSPlus gps;

// 目的地の緯度と経度
static const double DestinationLat = 35.681236;
static const double DestinationLng = 139.767125;

// GPSモジュールとの通信が正常かどうかを示すフラグ
bool is_gps_connected = false;

void setup()
{
  Serial.begin(115200);
  Serial.begin(GPSBaud);
}

void loop()
{
  // GPSデータを更新する
  while (Serial.available() > 0) {
    gps.encode(Serial.read());
  }

  // GPSデータが正常に取得できた場合
  if (gps.location.isValid()) {
    is_gps_connected = true;
    
    gps_lat = gps.location.lat();
      gps_longt = gps.location.lng();
      //Serial.print(millis());
      //GPSの値取得
      Serial.print("LAT:  "); Serial.println(gps_lat, 9);
      Serial.print("LONG: "); Serial.println(gps_longt, 9);

    

    // 目的地との距離を計算する
    double distance = TinyGPSPlus::distanceBetween(
        gps_lat, gps_longt, DestinationLat, DestinationLng);

    // 目的地に近づくごとにずれる現象を修正するため、GPS精度を考慮する
    distance += gps.hdop.hdop() * 2;

    Serial.print("Distance to destination: ");
    Serial.print(distance);
    Serial.println(" meters");
  }

  // GPSモジュールとの通信が正常に行えていない場合
  if (!is_gps_connected) {
    Serial.println("GPS module not connected!");
  }

  delay(1000);
}
