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

#define THRESHOLD 15 //閾値の設定

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


void setup(void){
Serial.begin(9600);
  pinMode(12, OUTPUT);

  Serial2.begin(115200);
  Serial.begin(115200);

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


}

void loop(void){
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

//if(distance<2){
      if (heading <= THRESHOLD || (360 - THRESHOLD) <= heading ) { //直進
        //Serial.println("近いです！");
        ledcWrite(CH1, 1000); //直進
        ledcWrite(CH2, 1000);
        delay(1000);
        if (distance < 0.5) {//ここの判定は難だ？超音波のdistanceでいいかな？GPSとめるから//countいれたい
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
    //}else{
      //GPSRUN();(かくていしたらGPSRUNもいれてコメントアウトはずしてやりたい)
    //}
  }
  
}
