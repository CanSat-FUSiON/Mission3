// GPS Tx をメガ Rx ピン 19 に接続し、GPS Rx をメガ Tx ピン 18 に接続します。 
#include <TinyGPS++.h>
TinyGPSPlus gps;

#define HardwareSerial Serial1(1);


void setup() {
  //pinMode(19, OUTPUT);
  //pinMode(18, INPUT);
// ここにセットアップ コードを入れて、一度だけ実行します:
Serial1.begin(115200, SERIAL_8N1, 19, 18); // シリアル接続
Serial.println("GPS Signal received:");
Serial1.begin(9600); // GPS センサーを接続
}

void loop(){
// このスケッチは、新しい文が正しくエンコードされるたびに情報を表示します。
while (Serial1.available() > 0){
gps.encode(Serial1.read());
if (gps.location.isUpdated()){
   //digitalWrite(19, HIGH);
// 度単位の緯度 (double)
Serial.print("Latitude= ");
Serial.print(gps.location.lat(), 6);
// 度単位の経度 (double)
Serial.print(" Longitude= ");
Serial.println(gps.location.lng(), 6);
 //digitalWrite(19, LOW);
}
}
}
