#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

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
#define N_MOTORS 2

// Encoder and motor settings
const int encA[] = {35, 39};
const int encB[] = {34, 36};
const int pwm[] = {14, 26};
const int dir[] = {27, 25};

// Encoder and motor global variables
long currT, prevT = 0;
int pos[N_MOTORS], power;
float deltaT, target[N_MOTORS], rpm[N_MOTORS], rpmFilt[N_MOTORS],  e[N_MOTORS],
      eprev[N_MOTORS], dedt[N_MOTORS], eintegral[N_MOTORS], u[N_MOTORS];
int prevPos[] = {0, 0};
volatile int pos_i[] = {0, 0};
float kp[] = {40, 40};
float ki[] = {30, 30};
float kd[] = {1.5, 1.5};

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
void PIDController_Motor(int i) {
  e[i] = target[i] - rpmFilt[i];
  dedt[i] = (e[i] - eprev[i]) / deltaT;
  eprev[i] = e[i];
  eintegral[i] = eintegral[i] + e[i] * deltaT;

  // Integrator anti-windup
  if (ki[i] * eintegral[i] > 2048) {
    eintegral[i] = 2048 / ki[i];
  } else if (ki[i] * eintegral[i] < -2048) {
    eintegral[i] = -2048 / ki[i];
  }

  // Compute the input value of actuator
  u[i] = kp[i] * e[i] + ki[i] * eintegral[i] + kd[i] * dedt[i];

  // Set the motor speed and direction
  if (abs(u[i]) <= 2048) {
    power = u[i] + 2048;
  } else if (u[i] > 2048) {
    power = 4096;
  } else {
    power = 0;
  }
}


float eprev2 = 0;
float eintegral2 = 0;

float PIDController(float reference, float measurement, float kp, float ki, float kd) {
  float e2 = reference - measurement;
  float dedt2 = (e2 - eprev2) / deltaT;
  eprev2 = e2;
  eintegral2 = eintegral2 + e2 * deltaT;

  // Integrator anti-windup
  //  if (ki * eintegral2 > maxVal) {
  //    eintegral2 = maxVal / ki;
  //  } else if (ki * eintegral2 < minVal) {
  //    eintegral2 = minVal / ki;
  //  }

  // Compute the input value of actuator
  float u2 = kp * e2 + ki * eintegral2 + kd * dedt2;

  // output the input value
  return u2;
}

void setup() {
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
  for (int i = 0; i < N_MOTORS; i++) {
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
}

void loop() {

  // Compute the time difference
  currT = micros();
  deltaT = ((float) (currT - prevT)) / 1.0e6;
  prevT = currT;


  // XYZ回転方向におけるオイラー角を取得
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  //現時点のx軸回転角度
  float heading = euler.x();
  float headingTarget = 0;
  float headingRPM = PIDController(headingTarget, heading, 1, 1, 0);
  
  // Set the target RPM values
  // Target値の設定について：Target値の変化が急だと、応答が振動する可能性があるため、一次ローパスフィルター（一次遅れ系）を入れた方がいい
  target[0] = headingRPM;
  target[1] = headingRPM;

  // Initialize the position variable
  for (int i = 0; i < N_MOTORS; i++) {
    pos[i] = 0;
  }

  // Read the encoders
  noInterrupts(); // Disable interrupts temporarily while reading to avoid misreading
  for (int i = 0; i < N_MOTORS; i++) {
    pos[i] = pos_i[i];
  }
  interrupts();

  // Compute the RPM
  for (int i = 0; i < N_MOTORS; i++) {
    rpm[i] = ((pos[i] - prevPos[i]) / deltaT) / PPR * 60.0;
    prevPos[i] = pos[i];
  }

  // Filter the RPM using a 2nd order low pass filter
  rpmFilt[0] = lp0.filt(rpm[0]);
  rpmFilt[1] = lp1.filt(rpm[1]);

  // Evaluate the control signal and control the motors
  for (int i = 0; i < N_MOTORS; i++) {
    PIDController_Motor(i);
    ledcWrite(i, power);
  }

  // Print the motors' filtered RPM
  Serial.print("Variable_1:");
  Serial.print(rpmFilt[0]);
  Serial.print(",");
  Serial.print("Variable_2:");
  Serial.println(rpmFilt[1]);
}
