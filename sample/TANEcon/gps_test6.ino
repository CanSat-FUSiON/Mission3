#include <SoftwareSerial.h>
 
// rxPin = 16  txPin = 17
SoftwareSerial mySerial(16, 17);
 
float lonx;
float latx;
float lony =130.223740;
float laty =33.600090;
float _lonx;
float _latx;
float _lony;
float _laty;
float aa;
float ab;
float ac;
float ad;
float XY;
float pi = 3.14159265;
float r = 6378137;//地球半径[m]表示


 
void setup() {
  mySerial.begin(9600);
  Serial.begin(115200);
}
void loop() {
  // 1つのセンテンスを読み込む
  String line = mySerial.readStringUntil('\n');
 
  if(line != ""){
    int i, index = 0, len = line.length();
    String str = "";
  
    // StringListの生成(簡易)
    String list[30];
    for (i = 0; i < 30; i++) {
      list[i] = "";
    }
 
    // 「,」を区切り文字として文字列を配列にする
    for (i = 0; i < len; i++) {
      if (line[i] == ',') {
        list[index++] = str;
        str = "";
        continue;
      }
      str += line[i];
    }
    
    // $GPGGAセンテンスのみ読み込む
    if (list[0] == "$GPGGA") {
      
      // ステータス
      if(list[6] != "0"){      
            float af = list[2].toFloat();
            int ad = af / 100;
            int am = (((af / 100.0) - ad) * 100.0) / 60;
            float as = (((((af / 100.0) - ad) * 100.0) - am) * 60) / (60 * 60);
            float latx = ad + am + as;
            
 
        // 経度
        float f = list[4].toFloat();
        int d = f / 100;
        int m = (((f / 100.0) - d) * 100.0) / 60;
        float s = (((((f / 100.0) - d) * 100.0) - m) * 60) / (60 * 60);
        float lonx = d + m + s;
        
       Serial.println(lonx, 6);
       Serial.println(latx, 6);

        ab = atan2((lony-lonx)*1.23,(laty-latx))*57.3;
        if(ab<=0){
          ac = 360  + ab;
        }else{
          ac = ab;
        }
        aa = sqrt(pow(lony-lonx,2)+pow(laty-latx,2))*99096.44;

       if(aa>7){
        Serial.println(aa,6);
        Serial.println(ac,6);
        Serial.println("");
       }else{
        Serial.print("GOAL");
       }
       
       delay(500);
       }
}
}

}
