//普通のRUN
//これもモーターでテスト済み

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>
#include <math.h>

// PWM出力端子設定
#define pwm1 14
#define dir1 27
#define pwm2 26
#define dir2 25

// PWM出力設定（周波数と分解能はチャンネルのペアでは同じに設定する）
#define CH1 1        // PWM出力チャンネル（0,1/ 2,3/ 4,5/ 6,7/ 8,9/ 10,11 /12,13 /14,15でペア）
#define FREQ 20000   // PWM出力周波数（最大周波数 : 20kHz / 2の「bit数」乗）
#define BIT_NUM 12  // bit数（1bit〜16bit）

//MD2個使うからチャンネルも二個必要かな？1こ出善さげな感じする
#define CH2 0        // PWM出力チャンネル（0,1/ 2,3/ 4,5/ 6,7/ 8,9/ 10,11 /12,13 /14,15でペア）
#define FREQ 20000   // PWM出力周波数（最大周波数 : 20kHz / 2の「bit数」乗）
#define BIT_NUM 12  // bit数（1bit〜16bit）

#define THRESHOLD 30 //閾値の設定

#define MOVING_AVG_SIZE 10

TinyGPSPlus gps;

double atan(double x);
//double atan2(double y, double x);

//float gps_lat; //現在地の緯度
//float gps_longt; //現在地の経度

int RX_PIN = 19;
int TX_PIN = 18;
int counter = 0;

/////////////////////////////////////////////

float destinationLat = 35.12345;
float destinationLng = 135.67890;

float latMovingAvg[MOVING_AVG_SIZE];
float lngMovingAvg[MOVING_AVG_SIZE];

/*
  float equator = 6378.137;
  float LatA = 30.374275208;
  float LongA = 130.960083008;     //コーンの緯度・経度入力

  float Angle() {
  return (90 - (atan2(sin((LongA) - (gps_longt)), (cos(gps_lat) * tan(LatA) - sin(gps_lat) * cos((LongA) - (gps_longt))))));
  }

  float Distance() {
  return (TinyGPSPlus::distanceBetween(gps_lat, gps_longt, LatA, LongA ));
  }
*/



/////////////////////////////////////////////


/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);



void setup() {
  // PWM出力に使用する端子を出力設定
  pinMode(pwm1, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(dir2, OUTPUT);

  // PWM初期設定
  ledcSetup(CH1, FREQ, BIT_NUM);   // PWM設定（ﾁｬﾝﾈﾙ, 周波数, bit数）
  ledcSetup(CH2, FREQ, BIT_NUM);   // PWM設定（ﾁｬﾝﾈﾙ, 周波数, bit数）
  ledcAttachPin(dir1, CH1);
  ledcAttachPin(dir2, CH2);

  digitalWrite(pwm1, HIGH);
  digitalWrite(pwm2, HIGH);

  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  for (int i = 0; i < MOVING_AVG_SIZE; i++) {
    latMovingAvg[i] = 0.0;
    lngMovingAvg[i] = 0.0;
  }

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }


}

////////////////////////////////////////////

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;

  } else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else {
    Serial.print("Unk:");
  }
  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}


void loop() {

  /*
    while (Serial1.available() > 0) {
      char c = Serial1.read();
      gps.encode(c);

      if (gps.location.isUpdated()) {
        gps_lat = gps.location.lat();
        gps_longt = gps.location.lng();
        //Serial.print(millis());
        //GPSの値取得
        Serial.print("LAT:  "); Serial.println(gps_lat, 9);
        Serial.print("LONG: "); Serial.println(gps_longt, 9);

      }
    }
  */

  while (Serial.available() > 0) {
    if (gps.encode(Serial.read())) {
      // Update moving average arrays
      for (int i = MOVING_AVG_SIZE - 1; i > 0; i--) {
        latMovingAvg[i] = latMovingAvg[i - 1];
        lngMovingAvg[i] = lngMovingAvg[i - 1];
      }
      latMovingAvg[0] = gps.location.lat();
      lngMovingAvg[0] = gps.location.lng();

      // Compute moving average
      float latSum = 0.0;
      float lngSum = 0.0;
      for (int i = 0; i < MOVING_AVG_SIZE; i++) {
        latSum += latMovingAvg[i];
        lngSum += lngMovingAvg[i];
      }
      float latMovingAvgVal = latSum / MOVING_AVG_SIZE;
      float lngMovingAvgVal = lngSum / MOVING_AVG_SIZE;

      // Compute distance to destination
      float distanceToDest = TinyGPSPlus::distanceBetween(
      latMovingAvgVal, lngMovingAvgVal, destinationLat, destinationLng);
      float Angle = TinyGPSPlus::courseTo(latMovingAvgVal, lngMovingAvgVal, destinationLat, destinationLng);


      Serial.print("Distance to destination: ");
      Serial.print(distanceToDest);
      Serial.println(" meters");
      Serial.print(" Heading: ");
      Serial.println(Angle);

      delay(1000);

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
          ledcWrite(CH1, 2846); //後退→右に旋回→直進
          ledcWrite(CH2, 2846);
          delay(100);
          ledcWrite(CH1, 1250);
          ledcWrite(CH2, 2048);
          delay(100);
          ledcWrite(CH1, 1250);
          ledcWrite(CH2, 1250);
          delay(100);
          count = 0;
        }
        last_millis = millis();
      }

      //Serial.print("Direction = ");                               //目的地Aの方角(°）
      //Serial.print(Angle());
      //Serial.print("deg:Distance = ");                             //目的地A迄の距離(m)
      //Serial.print(Distance());
      //Serial.println("m");

      //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
      //9軸値取得
      sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
      bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
      bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
      bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
      bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

      printEvent(&orientationData);
      printEvent(&angVelocityData);
      printEvent(&linearAccelData);
      printEvent(&magnetometerData);
      printEvent(&accelerometerData);
      printEvent(&gravityData);

      int8_t boardTemp = bno.getTemp();
      Serial.println();
      Serial.print(F("temperature: "));
      Serial.println(boardTemp);

      uint8_t system, gyro, accel, mag = 0;
      bno.getCalibration(&system, &gyro, &accel, &mag);
      Serial.println();
      Serial.print("Calibration: Sys=");
      Serial.print(system);
      Serial.print(" Gyro=");
      Serial.print(gyro);
      Serial.print(" Accel=");
      Serial.print(accel);
      Serial.print(" Mag=");
      Serial.println(mag);

      Serial.println("--");
      delay(BNO055_SAMPLERATE_DELAY_MS);

      imu::Vector<3> magnet = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
      imu::Vector<3> accelermetor = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

      float heading = atan2(magnet.y(), magnet.x()) * 180.0 / M_PI;
      // float heading(){
      // return (atan2(y,x) * 180.0 / M_PI);
      // atan2(mag.y(), mag.x()) * 180.0 / M_PI;

      float i = ((heading - Angle) + M_PI / 4);

      if (i < 0) {
        i = i + 360;
      }

      if (accelermetor.z() > 0) {
        if (i <= THRESHOLD || (360 - THRESHOLD) <= i ) { //直進
          //Serial.println("近いです！");
          ledcWrite(CH1, 1000); //直進
          ledcWrite(CH2, 1000);
          delay(1000);
          if (distanceToDest < 1) {
            while (1) {//GPSRUN停止。画像処理フェーズへ
              //Serial.println("ついたよー");
            }
          } else {
            //Serial.println("うごけうごけ！");
            ledcWrite(CH1, 1000); //直進
            ledcWrite(CH2, 1000);
            delay(1000);
          }
        } else { //右に旋回
          if (i <= 180) {
            Serial.println("まわれひだり");
            ledcWrite(CH1, 1500); //右タイヤ正回転
            ledcWrite(CH2, 2000); //左タイヤブレーキ
            delay(2000);
          } else { //左に旋
            Serial.println("まわれみぎ");
            ledcWrite(CH1, 2048); //右タイヤブレーキ
            ledcWrite(CH2, 2000); //左タイヤ正回転
            delay(2000);
          }
        }
      } else {
        Serial.println("upside down!!");
        ledcWrite(CH1, 1500); //右タイヤブレーキ
        ledcWrite(CH2, 1500); //左タイヤ正回転
        delay(2000);
      }


      Serial.print("i=");
      Serial.print(i);
      Serial.print("heading=");
      Serial.println(heading);

    }
  }

  /*

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
      ledcWrite(CH1, 2846); //後退→右に旋回→直進
      ledcWrite(CH2, 2846);
      delay(100);
      ledcWrite(CH1, 1250);
      ledcWrite(CH2, 2048);
      delay(100);
      ledcWrite(CH1, 1250);
      ledcWrite(CH2, 1250);
      delay(100);
      count = 0;
    }
    last_millis = millis();
    }

    //Serial.print("Direction = ");                               //目的地Aの方角(°）
    //Serial.print(Angle());
    //Serial.print("deg:Distance = ");                             //目的地A迄の距離(m)
    //Serial.print(Distance());
    //Serial.println("m");

    //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
    //9軸値取得
    sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    printEvent(&orientationData);
    printEvent(&angVelocityData);
    printEvent(&linearAccelData);
    printEvent(&magnetometerData);
    printEvent(&accelerometerData);
    printEvent(&gravityData);

    int8_t boardTemp = bno.getTemp();
    Serial.println();
    Serial.print(F("temperature: "));
    Serial.println(boardTemp);

    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.println();
    Serial.print("Calibration: Sys=");
    Serial.print(system);
    Serial.print(" Gyro=");
    Serial.print(gyro);
    Serial.print(" Accel=");
    Serial.print(accel);
    Serial.print(" Mag=");
    Serial.println(mag);

    Serial.println("--");
    delay(BNO055_SAMPLERATE_DELAY_MS);

    imu::Vector<3> magnet = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    imu::Vector<3> accelermetor = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    float heading = atan2(magnet.y(), magnet.x()) * 180.0 / M_PI;
    // float heading(){
    // return (atan2(y,x) * 180.0 / M_PI);
    // atan2(mag.y(), mag.x()) * 180.0 / M_PI;

    float i = ((heading - Angle) + M_PI / 4);

    if (i < 0) {
    i = i + 360;
    }

    if (accelermetor.z() > 0) {
    if (i <= THRESHOLD || (360 - THRESHOLD) <= i ) { //直進
      //Serial.println("近いです！");
      ledcWrite(CH1, 1000); //直進
      ledcWrite(CH2, 1000);
      delay(1000);
      if (Distance() < 1) {
        while (1) {//GPSRUN停止。画像処理フェーズへ
          //Serial.println("ついたよー");
        }
      } else {
        //Serial.println("うごけうごけ！");
        ledcWrite(CH1, 1000); //直進
        ledcWrite(CH2, 1000);
        delay(1000);
      }
    } else { //右に旋回
      if (i <= 180) {
        Serial.println("まわれひだり");
        ledcWrite(CH1, 1500); //右タイヤ正回転
        ledcWrite(CH2, 2000); //左タイヤブレーキ
        delay(2000);
      } else { //左に旋
        Serial.println("まわれみぎ");
        ledcWrite(CH1, 2048); //右タイヤブレーキ
        ledcWrite(CH2, 2000); //左タイヤ正回転
        delay(2000);
      }
    }
    } else {
    Serial.println("upside down!!");
    ledcWrite(CH1, 1500); //右タイヤブレーキ
    ledcWrite(CH2, 1500); //左タイヤ正回転
    delay(2000);
    }


    Serial.print("i=");
    Serial.print(i);
    Serial.print("heading=");
    Serial.println(heading);
  */
}
