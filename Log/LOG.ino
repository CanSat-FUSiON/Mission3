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

#include <TinyGPS++.h>
#include "Ambient.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "NewPing.h"

//GPS
TinyGPSPlus gps;

float gps_lat; //緯度
float gps_longt; //経度 

int RX_PIN = 16;
int TX_PIN = 17;

WiFiClient client;
Ambient ambient;

//ambient設定
const char* ssid = "Buffalo-G-8B68";
const char* password = "byvf8jr5eierp";

unsigned int channelId = 62369;
const char *writeKey = "a1ae206c8214f215";

//BNO055
/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

//Ultrasonic
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

void setup()
{
  // シリアルポート開始
  Serial.begin(115200);
  
  //GPS
  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  //BNO055
  while (!Serial) delay(10);  // wait for serial port to open!
  Serial.println("Orientation Sensor Test"); Serial.println("");
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);

  //Ultrasonic
  //いらない

  //ambient
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
  //GPS
  // Serial.println("test"); 
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    gps.encode(c);
    if (gps.location.isUpdated()) {

      gps_lat = gps.location.lat();
      gps_longt = gps.location.lng();
      Serial.print("LAT:"); Serial.println(gps_lat,9);
      Serial.print("LONG:"); Serial.println(gps_longt,9);
      delay(1000);
    }
  }
  delay(1000);

  //BNO055
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
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
  Serial.print(F("temperature:"));
  Serial.println(boardTemp);

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.println();
  Serial.print("Calibration: Sys:");
  Serial.print(system);
  Serial.print("Gyro:");
  Serial.print(gyro);
  Serial.print("Accel:");
  Serial.print(accel);
  Serial.print("Mag:");
  Serial.println(mag);

  delay(BNO055_SAMPLERATE_DELAY_MS);

  //ultrasonic
  // Calculate the speed of sound, duration, and distance
  soundsp = (331.4 + (0.606 * temp)) / 10000;
  duration = sonar.ping_median(iterations);
  distance = (duration / 2) * soundsp;

  if (distance <= MAX_DISTANCE && distance >= MIN_DISTANCE) {
    // Valid distance
    kaldist = kalman(distance);
    lpdist = lp.filt(distance);
    Serial.print("Distance:");
    Serial.print(distance);
    Serial.print(", ");
    Serial.print("KalDistance:");
    Serial.print(kaldist);
    Serial.print(",");
    Serial.print("LPDistance:");
    Serial.println(lpdist);
  } else {
    // Invalid distance
    distance = -1;
  }

  //ambient
  ambient.set(1, gps_lat);
  ambient.set(2, gps_longt);
  ambient.set(3, system);
  ambient.set(4, gyro);
  ambient.set(5, accel);
  ambient.set(6, mag);
  ambient.set(7, distance);
  ambient.set(8, kaldist);
  ambient.set(9, lpdist);
  ambient.send();  
}