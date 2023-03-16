//上下判定、落下判定、超音波センサ、画像処理、スタック判定、キャリ

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>
#include <math.h>
#include "Ambient.h"

WiFiClient client;
Ambient ambient;

const char* ssid = "shiota kyohei";
const char* password = "1234567890";

unsigned int channelId = 62369;
const char *writeKey = "a1ae206c8214f215";
//-------------------------------landing detect----------------------------

#define LPS25H_ADDRESS  0x5c /* SA0 -> GND */
#define LPS25H_WHO_AM_I     0x0f
#define LPS25H_CTRL_REG1    0x20
#define LPS25H_PRESS_OUT_XL 0x28
#define LPS25H_PRESS_OUT_L  0x29
#define LPS25H_PRESS_OUT_H  0x2a
#define LPS25H_TEMP_OUT_L   0x2b
#define LPS25H_TEMP_OUT_H   0x2c

//volatile float altitude;//[m]換算した高度
volatile float gnd_T = 0;//地面の高度入れる
volatile float referencePressure = 1013.15;//高度計算に使用する基準高度での大気圧[hPa]。ここは随時変える。

//------------GPSRUN------------------------------------

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

#define THRESHOLD 20 //閾値の設定

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
  return (TinyGPSPlus::distanceBetween(gps_lat, gps_longt, LatA, LongA ));
}

/////////////////////////////////////////////


/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

//----------------Ultrasonic------------------------------------------------
// Low Pass Filter Class
template <int order> // order is 1 or 2
class LowPass
{
  private:
    float a[order];
    float b[order + 1];
    float omega0;
    float dt;
    bool adapt;
    float tn1 = 0;
    float x[order + 1]; // Raw values
    float y[order + 1]; // Filtered values

  public:
    LowPass(float f0, float fs, bool adaptive) {
      // f0: cutoff frequency (Hz)
      // fs: sample frequency (Hz)
      // adaptive: boolean flag, if set to 1, the code will automatically set
      // the sample frequency based on the time history.

      omega0 = 6.28318530718 * f0;
      dt = 1.0 / fs;
      adapt = adaptive;
      tn1 = -dt;
      for (int k = 0; k < order + 1; k++) {
        x[k] = 0;
        y[k] = 0;
      }
      setCoef();
    }

    void setCoef() {
      if (adapt) {
        float t = micros() / 1.0e6;
        dt = t - tn1;
        tn1 = t;
      }

      float alpha = omega0 * dt;
      if (order == 1) {
        a[0] = -(alpha - 2.0) / (alpha + 2.0);
        b[0] = alpha / (alpha + 2.0);
        b[1] = alpha / (alpha + 2.0);
      }
      if (order == 2) {
        float alphaSq = alpha * alpha;
        float beta[] = {1, sqrt(2), 1};
        float D = alphaSq * beta[0] + 2 * alpha * beta[1] + 4 * beta[2];
        b[0] = alphaSq / D;
        b[1] = 2 * b[0];
        b[2] = b[0];
        a[0] = -(2 * alphaSq * beta[0] - 8 * beta[2]) / D;
        a[1] = -(beta[0] * alphaSq - 2 * beta[1] * alpha + 4 * beta[2]) / D;
      }
    }

    float filt(float xn) {
      // Provide me with the current raw value: x
      // I will give you the current filtered value: y
      if (adapt) {
        setCoef(); // Update coefficients if necessary
      }
      y[0] = 0;
      x[0] = xn;
      // Compute the filtered values
      for (int k = 0; k < order; k++) {
        y[0] += a[k] * y[k + 1] + b[k] * x[k];
      }
      y[0] += b[order] * x[order];

      // Save the historical values
      for (int k = order; k > 0; k--) {
        y[k] = y[k - 1];
        x[k] = x[k - 1];
      }

      // Return the filtered value
      return y[0];
    }
};

// Filter instance
LowPass<2> lp(0.7, 1e3, true);

// Ultrasonic sensor library
#include "NewPing.h"

// Ultrasonic sensor pins
#define TRIGGER_PIN  4
#define ECHO_PIN     5
#define MAX_DISTANCE 400
#define MIN_DISTANCE 2

// Constructor for ultrasonic sensor library
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Globals for ultrasonic sensor
int temp = 27; // Celsius
float soundsp;
float duration, distance, kaldist, lpdist;
int iterations = 3;

// Kalman filter function
double kalman(double U) {
  static const float R = 70;
  static const float H = 1;
  static float Q = 5;
  static float P = 0;
  static float U_hat = 0;
  static float K = 0;
  K = P * H / (H * P * H + R);
  U_hat += + K * (U - H * U_hat);
  P = (1 - K * H) * P + Q;
  return U_hat;
}

//--------------------imageRUN-------------------------
// Camera communication settings
#define CAMERA_START_BYTE 0x80
#define CAMERA_PACKET_SIZE 2//1+1
bool newCameraData = 0;
int heading = -999;


void setup(void)
{
  Serial.begin(9600);
  pinMode(12, OUTPUT);

  Serial2.begin(115200);
  Serial.begin(115200);

  Wire.begin();

  Serial.print("WHO_AM_I = 0x");
  Serial.println(whoAmI(), HEX);

  setCtrlReg1();

  delay(1000);

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
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  
  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());


  ambient.begin(channelId, writeKey, &client);


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


void loop(void)
{
  Serial.print("Pressure = ");
  Serial.print(getPressure());
  Serial.print(" hPa ,Altitude = ");
  Serial.print(altitude());
  Serial.print(" m , Temperature = ");
  Serial.print(getTemperature());
  Serial.println(" degree");

  if (float(altitude()) > float(gnd_T) + 10) {
    Serial.println("over 10meters");
    delay(1000);//10mのとこから1mのところまでおちてくる時間。20秒待機した後につぎのifの判断に移るってなるはず。

    while (1) {


      // * while(1)でくくってるところで無限ループするので気圧とかの値が出力されなくなってしまうのでは？と思ったので実験してそうなった場合は、ここのコメントアウトはずしてください。
      Serial.print("Pressure = ");
      Serial.print(getPressure());
      Serial.print(" hPa ,Altitude = ");
      Serial.print(altitude());
      Serial.print(" m , Temperature = ");
      Serial.print(getTemperature());
      Serial.println(" degree");
      
      ambient.set(1, getPressure());
      ambient.set(2, altitude());
      ambient.set(3, getTemperature());
      ambient.send();
      //*/

      if (float(altitude()) < 1) {
        Serial.println("under 1meter");
        delay(20000);//20秒まって導通
        Serial.println("HIGH");
        digitalWrite(12, HIGH);
        delay(10000);

        Serial.println("LOW");
        digitalWrite(12, LOW);
        //キャリブレーションをここで行う
         Serial.print("1");
  ledcWrite(CH1, 1000); //フル正回転
  ledcWrite(CH2, 1000); //フル正回
  delay(5000);
  Serial.print("2");
  ledcWrite(CH1, 2846); //フル逆回転4096
  ledcWrite(CH2, 2846); //フル逆回転
  delay(5000);
  Serial.print("3");
  ledcWrite(CH1, 1000);
  ledcWrite(CH2, 2048);
  delay(5000);
  Serial.print("4");
  ledcWrite(CH1, 2048);
  ledcWrite(CH2, 1000);
  delay(5000);
        while (1) {
          
          GPSRUN();//voidGPSRUNの呼び出し　うまくいってんのかな？うまくいかんかったらふつうにloopのないようぜんぶいれちゃお

          ultla();//void ultlaの呼び出し

        }
      } else {
        Serial.println("over 1meter");
        delay(500);
      }
    }

  } else {
    Serial.println("under 10meters");
    delay(500);
  }



}


//----------------landing detect--------------------
int whoAmI() {
  Wire.beginTransmission(LPS25H_ADDRESS);
  Wire.write(LPS25H_WHO_AM_I);
  Wire.endTransmission();

  Wire.requestFrom(LPS25H_ADDRESS, 1);
  while (Wire.available() < 1) {
    ;
  }

  return Wire.read();
}

void setCtrlReg1() {
  Wire.beginTransmission(LPS25H_ADDRESS);
  Wire.write(LPS25H_CTRL_REG1);
  Wire.write(0x90);
  Wire.endTransmission();
}

float getPressure() {
  long pData = 0;

  for (int i = 0; i < 3; i++) {
    Wire.beginTransmission(LPS25H_ADDRESS);
    Wire.write(LPS25H_PRESS_OUT_XL + i);
    Wire.endTransmission();

    Wire.requestFrom(LPS25H_ADDRESS, 1);
    while (Wire.available() < 1) {
      ;
    }

    pData |= Wire.read() << (8 * i);
  }

  return pData / 4096.0;
}

float getTemperature() {
  short tData = 0;

  for (int i = 0; i < 2; i++) {
    Wire.beginTransmission(LPS25H_ADDRESS);
    Wire.write(LPS25H_TEMP_OUT_L + i);
    Wire.endTransmission();

    Wire.requestFrom(LPS25H_ADDRESS, 1);
    while (Wire.available() < 1) {
      ;
    }

    tData |= Wire.read() << (8 * i);
  }

  return 42.5 + tData / 480.0;
}

volatile float altitude() {
  //[m]換算した高度
  //return altitude = ((pow(referencePressure / getPressure(), 1 / 5.257) - 1)*(getTemperature() + 273.15)) / 0.0065;
  return ((pow(referencePressure / getPressure(), 1 / 5.257) - 1) * (getTemperature() + 273.15)) / 0.0065;

}

//---------------------------------GPSRUN--------------------
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


void GPSRUN() {

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

  Serial.print("Direction = ");                               //目的地Aの方角(°）
  Serial.print(Angle());
  Serial.print("deg:Distance = ");                             //目的地A迄の距離(m)
  Serial.print(Distance());
  Serial.println("m");

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

  float i = ((heading - Angle()) + M_PI / 4);

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
}

//--------------------ultlasonic------------------
void ultla() {
  // Calculate the speed of sound, duration, and distance
  soundsp = (331.4 + (0.606 * temp)) / 10000;
  duration = sonar.ping_median(iterations);
  distance = (duration / 2) * soundsp;

  if (distance <= MAX_DISTANCE && distance >= MIN_DISTANCE) {
    // Valid distance
    kaldist = kalman(distance);
    lpdist = lp.filt(distance);
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(", ");
    Serial.print("KalDistance: ");
    Serial.print(kaldist);
    Serial.print(", ");
    Serial.print("LPDistance: ");
    Serial.println(lpdist);
  } else {
    // Invalid distance
    distance = -1;
  }
}

//------------------imageRUN-------------------------------
void image() {
  // Loops while the amount of data is >= the packet size
  while (Serial2.available() >= CAMERA_PACKET_SIZE) {
    uint8_t first = Serial2.read();

    // Makes sure the first byte is the start of packet indicator, otherwise keeps looping
    if (first == CAMERA_START_BYTE) {
      newCameraData = true;

      uint8_t dataBuffer[CAMERA_PACKET_SIZE - 1];

      // Put all the data into an array
      for (int i = 0; i < CAMERA_PACKET_SIZE - 1; i++) {
        dataBuffer[i] = Serial2.read();
      }

      heading = (dataBuffer[0] << 7) | dataBuffer[1];

      if (heading <= THRESHOLD || (360 - THRESHOLD) <= heading ) { //直進
        //Serial.println("近いです！");
        ledcWrite(CH1, 1000); //直進
        ledcWrite(CH2, 1000);
        delay(1000);
        if (distance < 0.5) {//ここの判定は難だ？超音波のdistanceでいいかな？GPSとめるから
          while (1) {
            Serial.println("gool!!");
          }
        } else {
          //Serial.println("うごけうごけ！");
          ledcWrite(CH1, 1000); //直進
          ledcWrite(CH2, 1000);
          delay(1000);
        }
      } else { //右に旋回
        if (heading <= 180) {
          Serial.println("まわれみぎ");
          ledcWrite(CH1, 1500); //右タイヤ正回転
          ledcWrite(CH2, 2048); //左タイヤブレーキ
          delay(1000);
        } else { //左に旋
          Serial.println("まわれひだり");
          ledcWrite(CH1, 2048); //右タイヤブレーキ
          ledcWrite(CH2, 1500); //左タイヤ正回転
          delay(1000);
        }
      }
    } else {
      Serial.println("upside down!!");
      //ここに動作を入れる。上下ひっくり返すための動きを実験で確認。
    }
  }
}
