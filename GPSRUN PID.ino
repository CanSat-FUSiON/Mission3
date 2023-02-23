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

//-----------------------------PID------------



// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);


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
LowPass<2> lp0(3, 1e3, true);
LowPass<2> lp1(3, 1e3, true);

// ループ時間が長くなるほど測定ノイズが多くなるため、滑らかな出力のために遮断周波数を下げる必要がある　＋　PIDをチューニング
//LowPass<2> lp0(0.6, 1e3, true);
//LowPass<2> lp1(0.6, 1e3, true);

// Motor settings
#define FREQ 20000
#define BIT_NUM 12
#define PPR 244.8
#define PID_COUNT 3
#define MOTOR_COUNT 2
#define MAX_ROT_SPEED 100

// Encoder and motor settings
const int encA[MOTOR_COUNT] = {35, 39};
const int encB[MOTOR_COUNT] = {34, 36};
const int pwm[MOTOR_COUNT] = {14, 26};
const int dir[MOTOR_COUNT] = {27, 25};

// Encoder and motor global variables
long currT, prevT = 0;
int pos[PID_COUNT];
float deltaT, power, target[PID_COUNT], rpm[MOTOR_COUNT], measurement[PID_COUNT],  e[PID_COUNT],
      eprev[PID_COUNT], dedt[PID_COUNT], eintegral[PID_COUNT], u[PID_COUNT];
int prevPos[MOTOR_COUNT] = {0, 0};
volatile int pos_i[MOTOR_COUNT] = {0, 0};
float kp[PID_COUNT] = {40, 40, 1};
float ki[PID_COUNT] = {30, 30, 1};
float kd[PID_COUNT] = {1.5, 1.5, 0};

// Interrupt service routine (ISR)
template <int i>
void readEncoder() {
  // Read encoder B when ENCA rises
  bool b = digitalRead(encB[i]);
  if (b > 0) {
    // If B is high, increment backward
    pos_i[i]--;
  }
  else {
    // Otherwise, increment forward
    pos_i[i]++;
  }
}

// PID Controller
// i= 0, 1:
// Motors
// i = 2:
// Heading
float PIDController(int i, int maxVal, int minVal) {
  e[i] = target[i] - measurement[i];
  dedt[i] = (e[i] - eprev[i]) / deltaT;

  if (i == 2 && e[i] < - 180) {
    e[i] += 360;
  } else if (i == 2 && e[i] > 360) {
    e[i] -= 360;
  }

  eprev[i] = e[i];
  eintegral[i] = eintegral[i] + e[i] * deltaT;

  // Integrator anti-windup
  if (ki[i] * eintegral[i] > maxVal) {
    eintegral[i] = maxVal / ki[i];
  } else if (ki[i] * eintegral[i] < -minVal) {
    eintegral[i] = -minVal / ki[i];
  }

  // Compute the input value of actuator
  u[i] = kp[i] * e[i] + ki[i] * eintegral[i] + kd[i] * dedt[i];

  // Specific conditions
  if (i == 0 || i == 1) {
    if (abs(u[i]) <= 2048) {
      return u[i] + 2048;
    } else if (u[i] > 2048) {
      return 4096;
    } else {
      return 0;
    }
  } else if (i == 2) {
    if (u[i] > maxVal) {
      u[i] = maxVal;
    } else if  (u[i] < minVal) {
      u[i] = minVal;
    }
    //return u[i];
  }
  return u[i];
}





//-----------------------------gps run--------------------
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
float LatA = 35.7408;
float LongA = 139.7429;      //コーンの緯度・経度入力

//float Angle() {
//return (90 - (atan2(sin((LongA) - (gps_longt)), (cos(gps_lat) * tan(LatA) - sin(gps_lat) * cos((LongA) - (gps_longt))))));
//}

float Distance() {
  return (TinyGPSPlus::distanceBetween(gps_lat, gps_longt, LatA, LongA ) / 1000.0);
}



/////////////////////////////////////////////


/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;



void setup() {

  //--------------------------pid-----------------------


  Serial.begin(115200);

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  bno.setExtCrystalUse(true);

  // Set up the encoder and motor pins
  for (int i = 0; i < MOTOR_COUNT; i++) {
    pinMode(encA[i], INPUT);
    pinMode(encB[i], INPUT);
    pinMode(pwm[i], OUTPUT);
    digitalWrite(pwm[i], HIGH);
    pinMode(dir[i], OUTPUT);
    ledcSetup(i, FREQ, BIT_NUM);
    ledcAttachPin(dir[i], i);
  }

  // Attach the interrupt pins to encoders
  attachInterrupt(digitalPinToInterrupt(encA[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(encA[1]), readEncoder<1>, RISING);


  //------------------------gpsrun-----------------
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


void loop() {
  //-----pid-----------------------------------------------------

  // Compute the time difference
  currT = micros();
  deltaT = ((float) (currT - prevT)) / 1.0e6;
  prevT = currT;

  // 地磁気センサーから読み取れる東からロボット正面方向の角度をphiとする。東から目標方向の角度をthetaとする。
  // 目標方向を0度基準にして左回りに180度、右回りに-180度）
  imu::Vector<3> magnet = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  //float heading = atan2(y, x) * 180.0 / M_PI;
  int phi = magnet.x();
  //int phi = atan2(y, x) * 180.0 / M_PI;
  int theta = 90 - (atan2(sin((LongA) - (gps_longt)), (cos(gps_lat) * tan(LatA) - sin(gps_lat) * cos((LongA) - (gps_longt)))));

  // float heading(){
  // return (atan2(y,x) * 180.0 / M_PI);
  // atan2(mag.y(), mag.x()) * 180.0 / M_PI;

  //float i = ((heading - Angle())+ M_PI/4);


  if (phi - theta > 180) { // 目標方向を0度基準にしてロボット正面方向から目標方向の偏差角度を算出する
    measurement[2] = phi - theta - 360;
  } else if(phi - theta < -180){
    measurement[2] = phi - theta + 360;
  }else{
    measurement[2] = phi - theta;
  }

  target[2] = 0; // 目標方向を0度基準にする

  // Set the target RPM values
  // Target値の設定について：Target値の変化が急だと、応答が振動する可能性があるため、一次ローパスフィルター（一次遅れ系）を入れた方がいい
  target[0] = PIDController(2, MAX_ROT_SPEED, -MAX_ROT_SPEED);
  target[1] = PIDController(2, MAX_ROT_SPEED, -MAX_ROT_SPEED);

  // Initialize the position variable
  for (int i = 0; i < MOTOR_COUNT; i++) {
    pos[i] = 0;
  }

  // Read the encoders
  noInterrupts(); // Disable interrupts temporarily while reading to avoid misreading
  for (int i = 0; i < MOTOR_COUNT; i++) {
    pos[i] = pos_i[i];
  }
  interrupts();

  // Control the motors
  for (int i = 0; i < MOTOR_COUNT; i++) {
    rpm[i] = ((pos[i] - prevPos[i]) / deltaT) / PPR * 60.0;
    prevPos[i] = pos[i];
    if (i == 0) { // Filter the RPM using a 2nd order low pass filter
      measurement[i] = lp0.filt(rpm[i]);
    } else {
      measurement[i] = lp1.filt(rpm[i]);
    }
    power = PIDController(i, 2048, -2048); // Evaluate the control signal and control the motors
    ledcWrite(i, power); // Input signal to motor plant
  }

  // Print the motors' filtered RPM
  Serial.print("Variable_3:");
  Serial.println(measurement[2]);

  //--------------------------------GPSRUN-------------------------
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

  //Serial.print("Direction = ");                               //目的地Aの方角(°）
  //Serial.print(Angle());
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

  /*
    printEvent(&orientationData);
    printEvent(&angVelocityData);
    printEvent(&linearAccelData);
    printEvent(&magnetometerData);
    printEvent(&accelerometerData);
    printEvent(&gravityData);
  */
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



}
