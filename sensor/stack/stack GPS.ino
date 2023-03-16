
///////////////////////////////////////////
#include <TinyGPS++.h>
#include <math.h>


TinyGPSPlus gps;

double atan(double x);
//double atan2(double y, double x);

float gps_lat; //現在地の緯度
float gps_longt; //現在地の経度

int RX_PIN = 19;
int TX_PIN = 18;
int counter = 0;

/////////////////////////////////////////////

float equator = 6378.137;
float LatA = 38.9854;
float LongA = 135.9876;      //コーンの緯度・経度入力

float Angle() {
  return (90 - (atan2(sin((LatA) - (gps_lat)), (cos(gps_lat) * tan(LatA) - sin(gps_lat) * cos((LatA) - (gps_lat))))));
}
float Distance() {
  return (equator) * acos(sin(gps_lat) * sin(LatA) + cos(gps_lat) * cos(LatA) * cos((LatA) - (gps_lat)));
}

/////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
}

////////////////////////////////////////////

void loop() {
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    gps.encode(c);

    if (gps.location.isUpdated()) {
      gps_lat = gps.location.lat();
      gps_longt = gps.location.lng();
      //Serial.print(millis());
      //GPSの値取得
      Serial.print("LAT:  "); Serial.println(gps_lat, 15);
      Serial.print("LONG: "); Serial.println(gps_longt, 15);

    }
  }

  static unsigned long last_millis = 0;
  static int count = 0;
  if (millis() - last_millis > 1000) {
    if (gps.speed.kmph() < 1) {
      count++;
    } else {
      count = 0;
    }
    if (count > 5) {
      Serial.println("Stuck!");
      delay(1000);
      // スタックリカバリー処理をここに記述する
      count = 0;
    }
    last_millis = millis();
  }


  Serial.print("Direction = ");                               //目的地Aの方角(°）
  Serial.print(Angle());
  Serial.print("deg:Distance = ");                             //目的地A迄の距離(m)
  Serial.print(Distance());
  Serial.println("m");
}
